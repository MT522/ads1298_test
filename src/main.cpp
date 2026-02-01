#ifdef __cplusplus
extern "C" {
#endif

#include "ADS1298_Driver.h"

// Create ADS1298 driver instance
ADS1298_Driver ads1298;

// Function declarations
void printSampleData();
void parseSampleData(uint8_t* rawData, int32_t* channelData);

// Global variables for sample tracking
volatile bool newSampleAvailable = false;
uint32_t lastSampleCount = 0;

static const char* TAG = "MAIN";

void configure_logging() {
    // Set specific tag log levels
    esp_log_level_set("ADS129x", ESP_LOG_DEBUG);
    esp_log_level_set("MAIN", ESP_LOG_INFO);
}

extern "C" void app_main(void) {
    // Register this task with the Task Watchdog Timer
    esp_task_wdt_add(NULL);  // Add current task to TWDT

    // Configure logging (optional - defaults are from menuconfig)
    configure_logging();
  
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32 ADS1298 ECG Monitor");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing ADS1298...");
    
    // Initialize the ADS1298 driver
    if (ads1298.begin()) {
        ESP_LOGI(TAG, "✓ ADS1298 initialized successfully!");
        ESP_LOGI(TAG, "Starting continuous sampling...");
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Sample format: Status1, Status2, Status3, Ch1, Ch2, Ch3, Ch4, Ch5, Ch6, Ch7, Ch8");
        ESP_LOGI(TAG, "Data ready - monitoring samples...");
        ESP_LOGI(TAG, "========================================");

        ads1298.Soft_Start_ADS1x9x();
        ads1298.Start_Read_Data_Continuous();
    } else {
        ESP_LOGE(TAG, "✗ Failed to initialize ADS1298!");
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "TROUBLESHOOTING CHECKLIST:");
        ESP_LOGE(TAG, "1. Power Supply:");
        ESP_LOGE(TAG, "   - Verify 3.3V power to ADS1298");
        ESP_LOGE(TAG, "   - Check PWDN pin is HIGH (power on)");
        ESP_LOGE(TAG, "   - Ensure proper ground connections");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "2. SPI Connections:");
        ESP_LOGE(TAG, "   - MOSI: ESP32 GPIO23 -> ADS1298 DIN");
        ESP_LOGE(TAG, "   - MISO: ESP32 GPIO19 -> ADS1298 DOUT");
        ESP_LOGE(TAG, "   - SCLK: ESP32 GPIO18 -> ADS1298 SCLK");
        ESP_LOGE(TAG, "   - CS:   ESP32 GPIO5  -> ADS1298 CS");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "3. Control Pins:");
        ESP_LOGE(TAG, "   - RST:  ESP32 GPIO2  -> ADS1298 RESET");
        ESP_LOGE(TAG, "   - START: ESP32 GPIO15 -> ADS1298 START");
        ESP_LOGE(TAG, "   - DRDY: ESP32 GPIO4  -> ADS1298 DRDY");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "4. Check for:");
        ESP_LOGE(TAG, "   - Loose connections");
        ESP_LOGE(TAG, "   - Wrong pin assignments");
        ESP_LOGE(TAG, "   - Damaged components");
        ESP_LOGE(TAG, "   - Insufficient power supply current");
        ESP_LOGE(TAG, "========================================");
        
        while(1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGE(TAG, "System halted. Fix connections and restart.");
        }
    }

    // Main loop: parse and log samples as they arrive
    while (1) {
        uint16_t currentSampleNum = ads1298.RawECGBufferWriteSampleNum;
        if (currentSampleNum != lastSampleCount) {
            // New sample(s) arrived - process each one
            do {
                // Index of the sample we haven't processed yet
                uint16_t sampleNum = lastSampleCount;
                lastSampleCount = (lastSampleCount + 1) % RAW_ECG_BUFFER_FULL_SAMPLE_NUM;

                uint32_t sampleByteIndex = (sampleNum * RAW_ECG_SAMPLE_SIZE) % RAW_ECG_BUFFER_SIZE;

                uint8_t rawSample[RAW_ECG_SAMPLE_SIZE];
                for (int i = 0; i < RAW_ECG_SAMPLE_SIZE; i++) {
                    rawSample[i] = ads1298.RawECGBuffer[sampleByteIndex + i];
                }

                int32_t channelData[8];
                parseSampleData(rawSample, channelData);

                ESP_LOGI(TAG, "Sample #%u | Status: 0x%02X 0x%02X 0x%02X | Ch1:%8ld Ch2:%8ld Ch3:%8ld Ch4:%8ld Ch5:%8ld Ch6:%8ld Ch7:%8ld Ch8:%8ld",
                         sampleNum, rawSample[0], rawSample[1], rawSample[2],
                         channelData[0], channelData[1], channelData[2], channelData[3],
                         channelData[4], channelData[5], channelData[6], channelData[7]);
            } while (lastSampleCount != currentSampleNum);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Yield to avoid busy-wait
    }
}


void printSampleData() {
  // Calculate the index of the most recent sample
  uint32_t sampleIndex = (ads1298.RawECGBufferWriteIndex - RAW_ECG_SAMPLE_SIZE) % RAW_ECG_BUFFER_SIZE;
  
  // Extract the raw sample data
  uint8_t rawSample[RAW_ECG_SAMPLE_SIZE];
  for (int i = 0; i < RAW_ECG_SAMPLE_SIZE; i++) {
    rawSample[i] = ads1298.RawECGBuffer[sampleIndex + i];
  }
  
  // Parse and print the sample data
  int32_t channelData[8];
  parseSampleData(rawSample, channelData);
  
  // Print status bytes
  ESP_LOGI(TAG, "Status: 0x%02X 0x%02X 0x%02X | ", 
                rawSample[0], rawSample[1], rawSample[2]);
  
  // Print channel data
  for (int ch = 0; ch < 8; ch++) {
    ESP_LOGI(TAG, "Ch%d: %8ld ", ch + 1, channelData[ch]);
  }
  ESP_LOGI(TAG, "\n");
}

void parseSampleData(uint8_t* rawData, int32_t* channelData) {
  // Skip the first 3 status bytes
  uint8_t* dataPtr = rawData + 3;
  
  // Parse each channel (3 bytes per channel, 24-bit signed)
  for (int ch = 0; ch < 8; ch++) {
    // Combine 3 bytes into 24-bit signed integer
    int32_t sample = 0;
    sample = (int32_t)((uint32_t)dataPtr[0] << 16) | 
             (int32_t)((uint32_t)dataPtr[1] << 8) | 
             (int32_t)dataPtr[2];
    
    // Sign extend if negative (24-bit to 32-bit)
    if (sample & 0x800000) {
      sample |= 0xFF000000;
    }
    
    channelData[ch] = sample;
    dataPtr += 3;
  }
}

#ifdef __cplusplus
}
#endif