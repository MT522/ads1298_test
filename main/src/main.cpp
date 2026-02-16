#ifdef __cplusplus
extern "C" {
#endif

#include "ads129x_driver.h"
#include "uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstring>

// Create ADS1298 driver instance
ADS1298_Driver ads1298;

// Function declarations
void printSampleData();
void parseSampleData(uint8_t* rawData, float* channelData);

// ADS1298 is at 2 kSPS. Downsample to 250 SPS for UART (avoids Task WDT: 250*27=6.75 KB/s fits 115200).
#define RAW_SAMPLE_RATE_HZ    2000
#define OUTPUT_SAMPLE_RATE_HZ 250
#define DOWNSAMPLE_FACTOR     (RAW_SAMPLE_RATE_HZ / OUTPUT_SAMPLE_RATE_HZ)  // 8
#define SAMPLES_PER_PACKET    50
#define PACKET_BYTES          (SAMPLES_PER_PACKET * RAW_ECG_SAMPLE_SIZE)
#define TASK_PERIOD_MS        1

// Read index for the print task (next sample to consume).
// Data is read in the DRDY ISR into RawECGBuffer; we consume by polling
// RawECGBufferWriteSampleNum (no semaphore).
static uint16_t s_lastSampleCount = 0;
// Downsampled output accumulator (0..SAMPLES_PER_PACKET-1).
static uint16_t s_packetSamplesCount = 0;

// ADS129x: 24-bit two's complement, MSB first. LSB weight = VREF/(2^23 - 1).
// +FS = 0x7FFFFF, -FS = 0x800000. Internal reference typically 2.4 V.
#define ADS1298_VREF_V           2.4f
#define ADS1298_POSITIVE_FS_CODE ((1UL << 23) - 1)   // 8388607

// Send 8 channel floats as raw binary (IEEE 754, 4 bytes each, no spacing). 32 bytes per sample.
static void send_sample_to_serialplot(const float channelFloats[8]) {
    uart_write_bytes(UART_NUM, (const char*)channelFloats, 8 * sizeof(float));
}

#define SAMPLE_PRINT_TASK_STACK  4096
#define SAMPLE_PRINT_TASK_PRIO  (tskIDLE_PRIORITY + 1)
#define SAMPLE_PRINT_TASK_CORE  tskNO_AFFINITY

// Packet buffer for batched UART send (50 samples * 27 bytes = 1350 bytes per packet).
static uint8_t s_packetBuffer[PACKET_BYTES];

static void sample_print_task(void* arg) {
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(100));

    // Do not subscribe this task to the Task WDT. We send one packet at a time
    // so each loop is short; the idle task can run and no task blocks the system.

    for (;;) {
        uint16_t currentSampleNum = ads1298.RawECGBufferWriteSampleNum;
        uint16_t available = 0;
        if (currentSampleNum != s_lastSampleCount) {
            if (currentSampleNum >= s_lastSampleCount) {
                available = currentSampleNum - s_lastSampleCount;
            } else {
                available = (RAW_ECG_BUFFER_FULL_SAMPLE_NUM - s_lastSampleCount) + currentSampleNum;
            }
        }

        // Downsample: take every DOWNSAMPLE_FACTOR-th sample. At 2 kSPS -> 250 SPS (6.75 KB/s).
        for (uint16_t i = 0; i < available && s_packetSamplesCount < SAMPLES_PER_PACKET; i++) {
            if (i % DOWNSAMPLE_FACTOR == 0) {
                uint16_t sampleNum = (s_lastSampleCount + i) % RAW_ECG_BUFFER_FULL_SAMPLE_NUM;
                uint32_t byteIdx = (uint32_t)sampleNum * RAW_ECG_SAMPLE_SIZE;
                memcpy(&s_packetBuffer[s_packetSamplesCount * RAW_ECG_SAMPLE_SIZE],
                       &ads1298.RawECGBuffer[byteIdx], RAW_ECG_SAMPLE_SIZE);
                s_packetSamplesCount++;
            }
        }
        s_lastSampleCount = (s_lastSampleCount + available) % RAW_ECG_BUFFER_FULL_SAMPLE_NUM;

        if (s_packetSamplesCount == SAMPLES_PER_PACKET) {
            // Single write: 1350 bytes at 6.75 KB/s; TX buffer drains between packets.
            uart_write_bytes(UART_NUM, (const char*)s_packetBuffer, PACKET_BYTES);
            s_packetSamplesCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

extern "C" void app_main(void) {
	esp_log_level_set("*", ESP_LOG_NONE);
	
    // Initialize UART0
    init_uart();
    
    uart_printf("Initializing ADS1298...\n");
    
    // Initialize the ADS1298 driver
    if (ads1298.begin()) {
        uart_printf("✓ ADS1298 initialized successfully!\n");
        uart_printf("Starting continuous sampling...\n");
        
        uart_printf("========================================\n");
        uart_printf("UART: raw ADS1298, downsampled %d->%d SPS, %d samples/packet (%u bytes)\n",
                    (int)RAW_SAMPLE_RATE_HZ, (int)OUTPUT_SAMPLE_RATE_HZ, (int)SAMPLES_PER_PACKET, (unsigned)PACKET_BYTES);
        uart_printf("Baud rate: %d\n", UART_BAUD_RATE);
        uart_printf("========================================\n");
        
        // Wait a moment to ensure messages are sent
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Order matters: RDATAC must be sent before starting conversions to avoid SPI conflicts.
        // Data is read in the DRDY ISR (27 bytes per sample); no semaphore/task.
        ads1298.Soft_Start_ADS1x9x();
        ads1298.Start_Read_Data_Continuous();

        // Wait until the ISR has written at least 2 samples (driver sets IsStartedSampling then)
        for (int i = 0; i < 100 && !ads1298.IsStartedSampling; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Start debug print task (prints 27-byte packets as hex)
        BaseType_t created = xTaskCreatePinnedToCore(
            sample_print_task,
            "sample_print",
            SAMPLE_PRINT_TASK_STACK,
            NULL,
            SAMPLE_PRINT_TASK_PRIO,
            NULL,
            SAMPLE_PRINT_TASK_CORE
        );
        if (created != pdPASS) {
            uart_printf("sample_print task create failed\n");
        }
        
        // Main task just idles
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
    } else {
        uart_printf("✗ Failed to initialize ADS1298!\n");
        uart_printf("========================================\n");
        uart_printf("TROUBLESHOOTING CHECKLIST:\n");
        uart_printf("1. Power Supply:\n");
        uart_printf("   - Verify 3.3V power to ADS1298\n");
        uart_printf("   - Check PWDN pin is HIGH (power on)\n");
        uart_printf("   - Ensure proper ground connections\n");
        uart_printf("\n");
        uart_printf("2. SPI Connections:\n");
        uart_printf("   - MOSI: ESP32 GPIO23 -> ADS1298 DIN\n");
        uart_printf("   - MISO: ESP32 GPIO19 -> ADS1298 DOUT\n");
        uart_printf("   - SCLK: ESP32 GPIO18 -> ADS1298 SCLK\n");
        uart_printf("   - CS:   ESP32 GPIO5  -> ADS1298 CS\n");
        uart_printf("\n");
        uart_printf("3. Control Pins:\n");
        uart_printf("   - RST:  ESP32 GPIO2  -> ADS1298 RESET\n");
        uart_printf("   - START: ESP32 GPIO15 -> ADS1298 START\n");
        uart_printf("   - DRDY: ESP32 GPIO4  -> ADS1298 DRDY\n");
        uart_printf("\n");
        uart_printf("4. Check for:\n");
        uart_printf("   - Loose connections\n");
        uart_printf("   - Wrong pin assignments\n");
        uart_printf("   - Damaged components\n");
        uart_printf("   - Insufficient power supply current\n");
        uart_printf("========================================\n");
        
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            uart_printf("System halted. Fix connections and restart.\n");
        }
    }
}

void printSampleData() {
    // Most recent sample: write index is one past last byte, so last sample starts at (index - 27)
    uint32_t sampleIndex = (ads1298.RawECGBufferWriteIndex + RAW_ECG_BUFFER_SIZE - RAW_ECG_SAMPLE_SIZE) % RAW_ECG_BUFFER_SIZE;
    
    // Extract the raw sample data
    uint8_t rawSample[RAW_ECG_SAMPLE_SIZE];
    for (int i = 0; i < RAW_ECG_SAMPLE_SIZE; i++) {
        rawSample[i] = ads1298.RawECGBuffer[sampleIndex + i];
    }
    
    // Parse and print the sample data (floats)
    float channelData[8];
    parseSampleData(rawSample, channelData);
    
    // Print status bytes
    uart_printf("Status: 0x%02X 0x%02X 0x%02X | ", 
                rawSample[0], rawSample[1], rawSample[2]);
    
    // Print channel data
    for (int ch = 0; ch < 8; ch++) {
        uart_printf("Ch%d: %8.2f ", ch + 1, (double)channelData[ch]);
    }
    uart_printf("\n");
}

void parseSampleData(uint8_t* rawData, float* channelData) {
    // Skip the first 3 status bytes
    uint8_t* dataPtr = rawData + 3;
    // LSB weight = VREF / (2^23 - 1); voltage = code * VREF / (2^23 - 1)
    const float lsb_volts = ADS1298_VREF_V / (float)ADS1298_POSITIVE_FS_CODE;

    for (int ch = 0; ch < 8; ch++) {
        // 24-bit two's complement, MSB first: first byte = MSB
        int32_t code = (int32_t)((uint32_t)dataPtr[0] << 16) |
                       (int32_t)((uint32_t)dataPtr[1] << 8) |
                       (int32_t)dataPtr[2];
        // Sign extend 24-bit to 32-bit (0x800000 -> 0x80000000, etc.)
        if (code & 0x800000) {
            code |= 0xFF000000;
        }
        // Convert to voltage; +0x7FFFFF -> +VREF, 0x800000 -> -VREF
        channelData[ch] = (float)code * lsb_volts;
        dataPtr += 3;
    }
}

#ifdef __cplusplus
}
#endif