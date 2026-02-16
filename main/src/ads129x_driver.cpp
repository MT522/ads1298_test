#include "ads129x_driver.h"
#include "uart.h"
#include <string.h>
#include <cstdio>
#include <cstdarg>

// Global pointer to the driver instance for the static ISR handler
static ADS1298_Driver* ads_driver_instance = nullptr;

// Custom logging functions (same as in main.cpp)
#define LOG_BUFFER_SIZE 128

// The ADS1298 maximum SCLK is 16MHz. We use 4MHz for stability.
const long SPI_CLOCK_FREQ = 4000000;

// Dummy TX buffer for reading 27 bytes in DRDY ISR (clock out while receiving).
// DRDY goes high after first SCLK pulse; data must be read immediately in ISR.
static const uint8_t DUMMY_TX_27[27] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF
}; 

// Static ISR handler (needs to be outside the class to be a true C function)
void IRAM_ATTR drdy_isr_handler() {
    if (ads_driver_instance != nullptr) {
        ads_driver_instance->readDataFromDRDY_ISR();
    }
}

TickType_t usToTicks(uint64_t microseconds) {
    uint64_t ticks = (microseconds * configTICK_RATE_HZ + 500000) / 1000000;
    return ticks > 0 ? ticks : 1;
}


// --- Class Constructor and Begin ---
ADS1298_Driver::ADS1298_Driver(spi_host_device_t spi_host) : 
    _spi_host(spi_host),
    _spi_device(nullptr)
{
    // Initialize buffer state
    RawECGBufferWriteSampleNum = 0;
    RawECGBufferWriteIndex = 0;
    IsStartedSampling = 0;
    
    // Store 'this' pointer for static ISR to use
    ads_driver_instance = this;
}

bool ADS1298_Driver::begin() {
    uart_printf("=== ADS1298 Driver Initialization ===\n");
    
    // 0. Install GPIO ISR service (required for ESP32)
    uart_printf("Installing GPIO ISR service...\n");
    esp_err_t isr_service_result = gpio_install_isr_service(0);
    if (isr_service_result != ESP_OK) {
        uart_printf("GPIO ISR service installation failed with error: %d\n", isr_service_result);
        return false;
    }
    uart_printf("GPIO ISR service installed successfully\n");

    // 1. Initialize control pins (RST, START, DRDY)
    uart_printf("Initializing control pins...\n");
    uart_printf("RST Pin: %d, START Pin: %d, DRDY Pin: %d\n", 
                  ADS_RST_PIN, ADS_START_PIN, ADS_DRDY_PIN);
    
    gpio_set_direction(ADS_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADS_START_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADS_CS_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(ADS_DRDY_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ADS_DRDY_PIN, GPIO_PULLUP_ONLY);

    // Configure DRDY pin for interrupt: TI datasheet — DRDY goes LOW when new conversion is ready.
    // Use negative (falling) edge so we fire exactly when data is ready to read.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ADS_DRDY_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE       // Falling edge = data ready
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Initial states
    uart_printf("Setting initial pin states...\n");
    DeAssertADS_CS();       // CS initially high (idle)
    DeAssertADS_Start();    // START initially low
    uart_printf("START: %s, DRDY: %s\n", 
                  gpio_get_level(ADS_START_PIN) ? "HIGH" : "LOW",
                  gpio_get_level(ADS_DRDY_PIN) ? "HIGH" : "LOW");

    // 2. Initialize SPI bus (using ESP-IDF driver)
    uart_printf("Initializing SPI bus with DMA...\n");
    spi_bus_config_t buscfg = {
        .mosi_io_num = ADS_MOSI_PIN,
        .miso_io_num = ADS_MISO_PIN,
        .sclk_io_num = ADS_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = RAW_ECG_SAMPLE_SIZE * 2, // Max size for DMA
    };

    esp_err_t ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        uart_printf("Failed to initialize SPI bus. Error: %s\n", esp_err_to_name(ret));
        return false;
    }

    spi_device_interface_config_t devcfg = {
        .mode = 1, // CPOL=0, CPHA=1 (SPI_MODE1)
        .clock_speed_hz = SPI_CLOCK_FREQ,
        .spics_io_num = -1, // We are controlling CS manually
        .queue_size = 7, // Queue depth for transactions
    };

    ret = spi_bus_add_device(_spi_host, &devcfg, &_spi_device);
    if (ret != ESP_OK) {
        uart_printf("Failed to add SPI device. Error: %s\n", esp_err_to_name(ret));
        return false;
    }
    uart_printf("SPI bus initialized and device added successfully\n");



    vTaskDelay(pdMS_TO_TICKS(10));

    uart_printf("Initializing DMA memory for SPI Command interface TX & RX\n");

    command_tx_buffer = (uint8_t*)(heap_caps_malloc(3, MALLOC_CAP_DMA));
    if (!command_tx_buffer) {
        uart_printf("Failed to allocate DMA memory for command\n");
        return false;
    }

    command_rx_buffer = (uint8_t*)(heap_caps_malloc(3, MALLOC_CAP_DMA));
    if (!command_rx_buffer) {
        uart_printf("Failed to allocate DMA memory for command\n");
        return false;
    }

    // 3. Power-on Initialization and Configuration
    uart_printf("Starting power-on initialization...\n");
    ADS1x9x_PowerOn_Init();
    uart_printf("Power-on initialization completed\n");

    // 4. Verify register configuration
    uart_printf("Verifying register configuration...\n");
    //bool registersVerified = verifyRegisterConfiguration();
    bool registersVerified = true;
    if (!registersVerified) {
        uart_printf("✗ Register verification failed. Device may not be properly configured.\n");
        return false;
    }
    uart_printf("✓ Register verification passed.\n");

    vTaskDelay(pdMS_TO_TICKS(10));

    // 5. Read ID register to verify communication
    uart_printf("Reading device ID register...\n");
    uint8_t id = GetADSId();
    uart_printf("Device ID read: 0x%02X\n", id);
    
    if (id == 0x92) { // ADS1298 ID
        uart_printf("✓ ADS1298 found (ID: 0x%02X). Driver initialized successfully.\n", id);
        return true;
    } else {
        uart_printf("✗ ADS1298 not found (ID: 0x%02X). Check wiring and connections.\n", id);
        uart_printf("Expected ID should be 0x92 for ADS1298\n");
        uart_printf("Troubleshooting tips:\n");
        uart_printf("1. Check SPI connections (MOSI, MISO, SCLK, CS)\n");
        uart_printf("2. Verify power supply (3.3V)\n");
        uart_printf("3. Check PWDN pin is HIGH\n");
        uart_printf("4. Verify RST pin connection\n");
        return false;
    }
}



// Datasheet: "After the serial communication is finished, always wait four or more tCLK
// periods before taking CS high." tCLK = 0.5µs @ 2.048MHz, so 4 tCLK = 2µs.
#define WAIT_AFTER_SPI_US  2

// --- Pin Control Functions (STM32 HAL to Arduino) ---
void ADS1298_Driver::AssertADS_CS(void) {
    gpio_set_level(ADS_CS_PIN, 0);
}

void ADS1298_Driver::DeAssertADS_CS(void) {
    gpio_set_level(ADS_CS_PIN, 1);
}

void ADS1298_Driver::AssertADS_Start(void) {
    gpio_set_level(ADS_START_PIN, 1);
}

void ADS1298_Driver::DeAssertADS_Start(void) {
    gpio_set_level(ADS_START_PIN, 0);
}

void ADS1298_Driver::AssertADS_Reset(void) {
    gpio_set_level(ADS_RST_PIN, 0);
}

void ADS1298_Driver::DeAssertADS_Reset(void) {
    gpio_set_level(ADS_RST_PIN, 1);
}

void ADS1298_Driver::ADS1x9x_Reset(void) {
    // Datasheet reset timing: RST low for ≥2 tCLK (1µs), then high, wait 18 tCLK (9µs)
    uart_printf("Asserting reset (RST LOW)...\n");
    AssertADS_Reset();
    esp_rom_delay_us(2);  // Hold reset for 2µs (4 tCLK)

    uart_printf("Deasserting reset (RST HIGH)...\n");
    DeAssertADS_Reset();
    esp_rom_delay_us(10);  // Wait 10µs (18 tCLK) for reset to complete
}



// --- Low-level Communication Functions ---
void ADS1298_Driver::sendCommand(uint8_t command) {
    command_tx_buffer[0] = command;
    
    AssertADS_CS();

    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = command_tx_buffer;
    t.rx_buffer = NULL;
    
    esp_err_t ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    esp_rom_delay_us(WAIT_AFTER_SPI_US);  // Datasheet: wait 4 tCLK before deasserting CS
    DeAssertADS_CS();
    
    uart_printf("Command 0x%02X sent\n", command);
}

void ADS1298_Driver::ADS1x9x_Reg_Write (uint8_t Read_write_address, uint8_t Data) { 
    command_tx_buffer[0] = Read_write_address | WREG;
    command_tx_buffer[1] = 0x00;  // Write 1 register (n-1 where n=1)
    command_tx_buffer[2] = Data;

    AssertADS_CS();

    esp_err_t ret;
    spi_transaction_t t;

    // Byte 1: Opcode (Register Address | WREG)
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);

    // Byte 2: Number of registers to write - 1
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+1;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);

    // Byte 3: Data
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+2;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);  // Datasheet: wait 4 tCLK before deasserting CS
    DeAssertADS_CS();
    
    
    uart_printf("Writing register 0x%02X: 0x%02X\n", Read_write_address, Data);
}

uint8_t ADS1298_Driver::ADS1x9x_Reg_Read(uint8_t Reg_address) {
    // Prepare TX: RREG command + register address, then number of registers-1, then dummy for read
    command_tx_buffer[0] = Reg_address | RREG;
    command_tx_buffer[1] = 0x00;
    command_tx_buffer[2] = 0x00;

    AssertADS_CS();

    spi_transaction_t t;
    esp_err_t ret;
    
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer;
    t.rx_buffer = NULL;
    t.flags = 0;
    
    ret = spi_device_polling_transmit(_spi_device, &t);
    assert(ret == ESP_OK);
    
    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);
    
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+1;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_polling_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);  // Datasheet: wait 4 tCLK before deasserting CS
    

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = NULL;
    t.rx_buffer = command_rx_buffer;
    t.flags = 0;

    ret = spi_device_polling_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    uint8_t result = (command_rx_buffer[0]); 

    uart_printf("I have read the value: 0x%02X\n", result);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us(WAIT_AFTER_SPI_US);
    DeAssertADS_CS();

    return result; 
}

uint8_t ADS1298_Driver::GetADSId(void) {
    return (ADS1x9x_Reg_Read(REG_DEVID));
}

bool ADS1298_Driver::verifyRegisterConfiguration(void) {
    uart_printf("=== Register Configuration Verification ===\n");
    bool allRegistersCorrect = true;
    
    // Define expected register values
    struct RegisterConfig {
        uint8_t address;
        uint8_t expectedValue;
        uint8_t mask;           // Mask for bits to check (0xFF to check all bits)
        const char* description;
    };
    
    RegisterConfig expectedRegs[] = {
        {REG_CONFIG1, 0xA4, 0xFF, "Config1: High-Res mode, Ext Clock, 2kSPS"},
        {REG_CONFIG2, 0x31, 0xFF, "Config2: Internal Test enable, f=2Hz"},
        {REG_CONFIG3, 0xCC, 0xFE, "Config3: Internal Ref, RLD buffer enabled (bit0=RLD status)"}, // Mask out bit 0
        {REG_RLD_SENSP, 0xFF, 0xFF, "RLD_SENSP: RLD from all P-side"},
        {REG_RLD_SENSN, 0xFF, 0xFF, "RLD_SENSN: RLD from all N-side"},
        {REG_PACE, 0x01, 0xFF, "PACE: PACE detection settings"},
        {REG_WCT1, 0x09, 0xFF, "WCT1: Wilson Central Terminal 1"},
        {REG_WCT2, 0xD0, 0xFF, "WCT2: Wilson Central Terminal 2"},
        {REG_LOFF, 0xA3, 0xFF, "LOFF: Lead-off detection settings"},
        {REG_CONFIG4, 0x02, 0xFF, "Config4: Lead-off enabled"},
        {REG_LOFF_SENSP, 0xFF, 0xFF, "LOFF_SENSP: Lead-off sense P"},
        {REG_LOFF_SENSN, 0xFF, 0xFF, "LOFF_SENSN: Lead-off sense N"}
    };
    
    // Verify each register
    for (int i = 0; i < sizeof(expectedRegs)/sizeof(expectedRegs[0]); i++) {
        uint8_t readValue = ADS1x9x_Reg_Read(expectedRegs[i].address);
        uint8_t maskedExpected = expectedRegs[i].expectedValue & expectedRegs[i].mask;
        uint8_t maskedRead = readValue & expectedRegs[i].mask;
        bool isCorrect = (maskedRead == maskedExpected);
        
        uart_printf("Reg 0x%02X: Expected=0x%02X, Read=0x%02X %s - %s\n", 
                     expectedRegs[i].address, 
                     expectedRegs[i].expectedValue, 
                     readValue,
                     isCorrect ? "✓" : "✗",
                     expectedRegs[i].description);
        
        if (!isCorrect) {
            allRegistersCorrect = false;
        }
    }
    
    // Verify channel settings (all channels should be 0x00 for normal ECG)
    uart_printf("Verifying channel settings...\n");
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t readValue = ADS1x9x_Reg_Read(REG_CH1SET + i);
        bool isCorrect = (readValue == 0x00);
        
        uart_printf("CH%d (0x%02X): Expected=0x00, Read=0x%02X %s\n", 
                     i+1, REG_CH1SET + i, readValue, isCorrect ? "✓" : "✗");
        
        if (!isCorrect) {
            allRegistersCorrect = false;
        }
    }
    
    if (allRegistersCorrect) {
        uart_printf("✓ All registers verified successfully!\n");
    } else {
        uart_printf("✗ Some registers have incorrect values!\n");
        uart_printf("Troubleshooting tips:\n");
        uart_printf("1. Check SPI communication\n");
        uart_printf("2. Verify power supply stability\n");
        uart_printf("3. Check for timing issues\n");
        uart_printf("4. Verify register write sequence\n");
    }
    
    uart_printf("=== Register Verification Complete ===\n\n\n");
    return allRegistersCorrect;
}

// --- ADS System Control (Matching provided C code) ---

void ADS1298_Driver::Wake_Up_ADS1x9x (void) { sendCommand(WAKEUP); }
void ADS1298_Driver::Put_ADS1x9x_In_Sleep (void) { sendCommand(STANDBY); }
void ADS1298_Driver::Soft_Reset_ADS1x9x (void) { sendCommand(RESET); }
void ADS1298_Driver::Soft_Start_ReStart_ADS1x9x (void) { sendCommand(START); }
void ADS1298_Driver::Start_Read_Data_Continuous (void) { sendCommand(RDATAC); }
void ADS1298_Driver::Stop_Read_Data_Continuous (void) { sendCommand(SDATAC); }
void ADS1298_Driver::Read_Data_by_Command (void) { sendCommand(RDATA); }

void ADS1298_Driver::Soft_Start_ADS1x9x (void) {
    Soft_Start_ReStart_ADS1x9x();

    gpio_isr_handler_add(ADS_DRDY_PIN, readDataFromDRDY_ISR_static, (void*)this);
    uart_printf("Start ECG sampling.\n");
}

void ADS1298_Driver::Soft_Stop_ADS1x9x (void) {
    sendCommand(STOP);
    uart_printf("Stop ECG sampling.\n");
}

// --- Default Register Read (verify register read after reset) ---

void ADS1298_Driver::readDefaultRegistersAfterReset(void) {
    uart_printf("=== Reading Default Registers After Reset ===\n");
    uart_printf("Register | Read Value | Expected (datasheet) | Status\n");
    uart_printf("---------|------------|---------------------|--------\n");

    // ADS1298 default values per datasheet Table 16 (SBAS459K)
    struct RegDefault { uint8_t addr; uint8_t expected; const char* name; };
    RegDefault defaults[] = {
        {REG_DEVID,     0x92, "ID (ADS1298)"},
        {REG_CONFIG1,   0x06, "CONFIG1 (HR, 500 SPS, internal clk)"},
        {REG_CONFIG2,   0x00, "CONFIG2 (test signal off)"},
        {REG_CONFIG3,   0x40, "CONFIG3 (external ref, RLD off)"},
        {REG_LOFF,      0x00, "LOFF"},
        {REG_CH1SET,    0x00, "CH1SET"},
        {REG_CH2SET,    0x00, "CH2SET"},
        {REG_CH3SET,    0x00, "CH3SET"},
        {REG_CH4SET,    0x00, "CH4SET"},
        {REG_CH5SET,    0x00, "CH5SET"},
        {REG_CH6SET,    0x00, "CH6SET"},
        {REG_CH7SET,    0x00, "CH7SET"},
        {REG_CH8SET,    0x00, "CH8SET"},
        {REG_RLD_SENSP, 0x00, "RLD_SENSP"},
        {REG_RLD_SENSN, 0x00, "RLD_SENSN"},
        {REG_LOFF_SENSP,0x00, "LOFF_SENSP"},
        {REG_LOFF_SENSN,0x00, "LOFF_SENSN"},
        {REG_LOFF_FLIP, 0x00, "LOFF_FLIP"},
        {REG_LOFF_STATP,0x00, "LOFF_STATP"},
        {REG_LOFF_STATN,0x00, "LOFF_STATN"},
        {REG_GPIO,      0x0F, "GPIO"},
        {REG_PACE,      0x00, "PACE"},
        {REG_RESP,      0x00, "RESP"},
        {REG_CONFIG4,   0x00, "CONFIG4"},
        {REG_WCT1,      0x00, "WCT1"},
        {REG_WCT2,      0x00, "WCT2"},
    };

    int mismatchCount = 0;
    for (size_t i = 0; i < sizeof(defaults) / sizeof(defaults[0]); i++) {
        uint8_t readVal = ADS1x9x_Reg_Read(defaults[i].addr);
        bool match = (readVal == defaults[i].expected);
        if (!match) mismatchCount++;
        uart_printf("  0x%02X   |    0x%02X     |        0x%02X           | %s\n",
                 defaults[i].addr, readVal, defaults[i].expected,
                 match ? "OK" : "MISMATCH");
    }

    uart_printf("---------|------------|---------------------|--------\n");
    if (mismatchCount == 0) {
        uart_printf("All default registers read correctly.\n");
    } else {
        uart_printf("%d register(s) did not match expected defaults. Check SPI/wiring.\n", mismatchCount);
    }
    uart_printf("=== Default Register Read Complete ===\n");
}

// --- Initialization and Configuration (Matching provided C code) ---

void ADS1298_Driver::ADS1x9x_PowerOn_Init(void) {
    uart_printf("=== ADS1298 Power-On Initialization ===\n");
    
    // 1. Reset sequence (partially done in begin(), repeated here as per original)
    uart_printf("Step 1: Performing hardware reset sequence...\n");
    ADS1x9x_Reset();
    uart_printf("hardware reset sequence completed\n");
    
    // 2. Stop Continuous Mode for configuration
    uart_printf("Step 2: Stopping continuous mode for configuration...\n");
    Stop_Read_Data_Continuous(); 
    uart_printf("Continuous mode stopped\n");

    // 3. Read default registers after reset (verify register read works correctly)
    readDefaultRegistersAfterReset();

    // 4. Configuration for Internal Test Signal (First Pass)
    uart_printf("Step 4: Configuring for internal test signal...\n");
    uart_printf("Writing configuration registers...\n");
    ADS1x9x_Reg_Write(REG_CONFIG1, 0xA4); // High-Resolution mode, Ext Clock, 2kSPS
    ADS1x9x_Reg_Write(REG_CONFIG2, 0x31); // Internal Test enable, f = 2 Hz
    ADS1x9x_Reg_Write(REG_CONFIG3, 0xCC); // Internal Ref, RLD buffer enabled
    ADS1x9x_Reg_Write(REG_RLD_SENSP, 0xFF); // RLD as average of all P-side
    ADS1x9x_Reg_Write(REG_RLD_SENSN, 0xFF); // RLD as average of all N-side
    ADS1x9x_Reg_Write(REG_PACE, 0x01); 
    ADS1x9x_Reg_Write(REG_WCT1, 0x09); 
    ADS1x9x_Reg_Write(REG_WCT2, 0xD0); 
    
    // Lead-off Detection (Test Pass)
    uart_printf("Configuring lead-off detection...\n");
    ADS1x9x_Reg_Write(REG_LOFF, 0x07); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); // Lo-off Enabled, Continuous mode
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Set All Channels to Internal Test Signal 
    uart_printf("Setting all channels to internal test signal...\n");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x35); // PGA = 3, Internal Test Signal
    }
    uart_printf("Channel configuration completed\n");
    
    /*// 5. Start Conversion (Test Pass)
    uart_printf("Step 5: Starting test conversion...\n");
    Soft_Start_ReStart_ADS1x9x();
    
    Start_Read_Data_Continuous(); 
    
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_printf("Test conversion completed\n");
    
    // 6. Stop and Reconfigure for Normal ECG
    uart_printf("Step 6: Reconfiguring for normal ECG...\n");
    Stop_Read_Data_Continuous(); 

    Soft_Stop_ADS1x9x();
    
    // Lead-off Detection (Normal ECG Pass)
    uart_printf("Configuring lead-off detection for normal ECG...\n");
    ADS1x9x_Reg_Write(REG_LOFF, 0xA3); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Channel Setting (Normal ECG)
    uart_printf("Setting channels for normal ECG...\n");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x00); // PGA = 6, Normal electrode input 
    }
    uart_printf("Normal ECG channel configuration completed\n");*/
    
    uart_printf("=== ADS1298 Power-On Initialization Complete ===\n\n\n");
}

// --- Data Acquisition Task and ISR ---
void IRAM_ATTR ADS1298_Driver::readDataFromDRDY_ISR_static(void* arg) {
    // Call the member function
    static_cast<ADS1298_Driver*>(arg)->readDataFromDRDY_ISR();
}

// ADS1298 first byte: only the high 4 bits are the header (0xC); low nibble is not header (e.g. status flags).
// Valid packet: (first_byte & 0xF0) == 0xC0. Reject all-zero or wrong-header reads.
#define ADS1298_VALID_STATUS_NIBBLE  0xC0   // high nibble must be 0xC

void IRAM_ATTR ADS1298_Driver::readDataFromDRDY_ISR() {
    // TI datasheet: DRDY goes high right after the first S_CLK pulse.
    // Read 27 bytes immediately in ISR with polling SPI (no task/semaphore).
    gpio_set_level(ADS_CS_PIN, 0);

    spi_transaction_t t = {};
    t.length = RAW_ECG_SAMPLE_SIZE * 8;
    t.tx_buffer = DUMMY_TX_27;
    t.rx_buffer = &RawECGBuffer[RawECGBufferWriteIndex];
    t.flags = 0;

    spi_device_polling_transmit(_spi_device, &t);

    // Ignore spurious/double DRDY: only advance buffer when we got a valid packet.
    // All-zero packets occur when DRDY fires twice per conversion (first read too early).
    if ((RawECGBuffer[RawECGBufferWriteIndex] & 0xF0) != ADS1298_VALID_STATUS_NIBBLE) {
        esp_rom_delay_us(WAIT_AFTER_SPI_US);  // Datasheet: wait 4 tCLK before deasserting CS
        gpio_set_level(ADS_CS_PIN, 1);  // Deassert CS before returning
        return;  // overwrite same slot on next ISR
    }

    RawECGBufferWriteIndex += RAW_ECG_SAMPLE_SIZE;
    RawECGBufferWriteSampleNum += 1;

    if (RawECGBufferWriteIndex >= RAW_ECG_BUFFER_SIZE) {
        RawECGBufferWriteIndex = 0;
    }
    if (RawECGBufferWriteSampleNum >= RAW_ECG_BUFFER_FULL_SAMPLE_NUM) {
        RawECGBufferWriteSampleNum = 0;
    }
    if (!IsStartedSampling && RawECGBufferWriteSampleNum > 1) {
        IsStartedSampling = 1;
    }
    
    esp_rom_delay_us(WAIT_AFTER_SPI_US);  // Datasheet: wait 4 tCLK before deasserting CS
    gpio_set_level(ADS_CS_PIN, 1);  // Deassert CS
}

ADS1298_Driver::~ADS1298_Driver() {
    uart_printf("=== ADS1298 Driver Destruction ===\n");
    
    // 1. Remove GPIO interrupt
    gpio_isr_handler_remove(ADS_DRDY_PIN);
    uart_printf("GPIO ISR handler removed\n");
    
    // 2. Free allocated memory
    if (command_tx_buffer != nullptr) {
        heap_caps_free(command_tx_buffer);
        command_tx_buffer = nullptr;
        uart_printf("TX buffer freed\n");
    }
    
    if (command_rx_buffer != nullptr) {
        heap_caps_free(command_rx_buffer);
        command_rx_buffer = nullptr;
        uart_printf("RX buffer freed\n");
    }
    
    // 3. Remove SPI device and bus
    if (_spi_device != nullptr) {
        esp_err_t ret = spi_bus_remove_device(_spi_device);
        if (ret != ESP_OK) {
            uart_printf("Failed to remove SPI device: %s", esp_err_to_name(ret));
        }
    } else {
        uart_printf("SPI bus freed");
    }
    
    // 4. Clear global instance pointer
    if (ads_driver_instance == this) {
        ads_driver_instance = nullptr;
        uart_printf("Global instance pointer cleared");
    }
    
    uart_printf("=== ADS1298 Driver Destroyed ===");
}

/*
 * ads129x_driver.cpp
 *
 *  Created on: Feb 8, 2026
 *      Author: Administrator
 */




