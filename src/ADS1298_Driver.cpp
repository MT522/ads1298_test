#include "ADS1298_Driver.h"

static const char* TAG = "ADS129x";

// The ADS1298 maximum SCLK is 16MHz. We use 4MHz for stability.
const long SPI_CLOCK_FREQ = 4000000; 

// Global pointer to the driver instance for the static ISR handler
static ADS1298_Driver* ads_driver_instance = nullptr;

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
    _spi_device(nullptr),
    _drdy_semaphore(nullptr),
    _dma_read_task_handle(nullptr)
{
    // Initialize buffer state
    RawECGBufferWriteSampleNum = 0;
    RawECGBufferWriteIndex = 0;
    IsStartedSampling = 0;
    
    // Store 'this' pointer for static ISR to use
    ads_driver_instance = this;
}

bool ADS1298_Driver::begin() {
    ESP_LOGI(TAG, "=== ADS1298 Driver Initialization ===");
    
    // 0. Install GPIO ISR service (required for ESP32)
    ESP_LOGI(TAG, "Installing GPIO ISR service...");
    esp_err_t isr_service_result = gpio_install_isr_service(0);
    if (isr_service_result != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR service installation failed with error: %d\n", isr_service_result);
        return false;
    }
    ESP_LOGI(TAG, "GPIO ISR service installed successfully");

    // 1. Initialize control pins (RST, START, DRDY)
    ESP_LOGI(TAG, "Initializing control pins...");
    ESP_LOGI(TAG, "RST Pin: %d, START Pin: %d, DRDY Pin: %d\n", 
                  ADS_RST_PIN, ADS_START_PIN, ADS_DRDY_PIN);
    
    gpio_set_direction(ADS_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADS_START_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADS_CS_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(ADS_DRDY_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ADS_DRDY_PIN, GPIO_PULLUP_ONLY);

    // Configure DRDY pin for interrupt
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ADS_DRDY_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    // Enable pull-up (important!)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE       // FALLING edge
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Initial states
    ESP_LOGI(TAG, "Setting initial pin states...");
    AssertADS_CS();    // CS always low (Gnd)
    DeAssertADS_Start();    // START initially low
    ESP_LOGI(TAG, "START: %s, DRDY: %s\n", 
                  gpio_get_level(ADS_START_PIN) ? "HIGH" : "LOW",
                  gpio_get_level(ADS_DRDY_PIN) ? "HIGH" : "LOW");

    // 2. Initialize SPI bus (using ESP-IDF driver)
    ESP_LOGI(TAG, "Initializing SPI bus with DMA...");
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
        ESP_LOGE(TAG, "Failed to initialize SPI bus. Error: %s\n", esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "Failed to add SPI device. Error: %s\n", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SPI bus initialized and device added successfully");



    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Initializing DMA memory for SPI Command interface TX & RX");

    command_tx_buffer = (uint8_t*)(heap_caps_malloc(3, MALLOC_CAP_DMA));
    if (!command_tx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA memory for command");
        return false;
    }

    command_rx_buffer = (uint8_t*)(heap_caps_malloc(3, MALLOC_CAP_DMA));
    if (!command_rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA memory for command");
        return false;
    }

    // 3. Create FreeRTOS components
    _drdy_semaphore = xSemaphoreCreateBinary();
    if (_drdy_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create DRDY semaphore.");
        return false;
    }

    BaseType_t task_created = xTaskCreate(
        _dma_read_task_wrapper,
        "dma_read_task",
        4096, // Stack size
        this, // Pass instance pointer
        configMAX_PRIORITIES - 1, // High priority
        &_dma_read_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DMA read task.");
        return false;
    }
    ESP_LOGI(TAG, "FreeRTOS components created successfully.");
    
    // 5. Power-on Initialization and Configuration
    ESP_LOGI(TAG, "Starting power-on initialization...");
    ADS1x9x_PowerOn_Init();
    ESP_LOGI(TAG, "Power-on initialization completed");

    // // 6. Verify register configuration
    ESP_LOGI(TAG, "Verifying register configuration...");
    bool registersVerified = verifyRegisterConfiguration();
    if (!registersVerified) {
        ESP_LOGE(TAG, "✗ Register verification failed. Device may not be properly configured.");
        return false;
    }
    ESP_LOGI(TAG, "✓ Register verification passed.");

    vTaskDelay(pdMS_TO_TICKS(10));

    // 7. Read ID register to verify communication
    ESP_LOGI(TAG, "Reading device ID register...");
    uint8_t id = GetADSId();
    ESP_LOGI(TAG, "Device ID read: 0x%02X\n", id);
    
    if (id == 0x92) { // ADS1298 ID
        ESP_LOGI(TAG, "✓ ADS1298 found (ID: 0x%02X). Driver initialized successfully.\n", id);
        return true;
    } else {
        ESP_LOGE(TAG, "✗ ADS1298 not found (ID: 0x%02X). Check wiring and connections.\n", id);
        ESP_LOGE(TAG, "Expected ID should be 0x92 for ADS1298");
        ESP_LOGE(TAG, "Troubleshooting tips:");
        ESP_LOGE(TAG, "1. Check SPI connections (MOSI, MISO, SCLK, CS)");
        ESP_LOGE(TAG, "2. Verify power supply (3.3V)");
        ESP_LOGE(TAG, "3. Check PWDN pin is HIGH");
        ESP_LOGE(TAG, "4. Verify RST pin connection");
        return false;
    }
}



// --- Pin Control Functions (STM32 HAL to Arduino) ---
void ADS1298_Driver::AssertADS_CS(void) {
    if (!is_cs_asserted) {
        ESP_LOGI(TAG, "Asserting CS PIN (Gnd)");
        gpio_set_level(ADS_CS_PIN, 0);
        ESP_LOGI(TAG, "CS Pin state after assertion: %s", gpio_get_level(ADS_CS_PIN) ?  "HIGH" : "LOW");
        vTaskDelay(pdMS_TO_TICKS(1));
        is_cs_asserted = true;
    }
}

void ADS1298_Driver::DeAssertADS_CS(void) {
    if (is_cs_asserted) {
        gpio_set_level(ADS_CS_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        is_cs_asserted = false;
    }
}

void ADS1298_Driver::AssertADS_Start(void) {
    if (!is_start_asserted) {
        gpio_set_level(ADS_START_PIN, 1);
        is_start_asserted = true;
    }
}

void ADS1298_Driver::DeAssertADS_Start(void) {
    if (is_start_asserted) {
        gpio_set_level(ADS_START_PIN, 0);
        is_start_asserted = false;
    }
}

void ADS1298_Driver::AssertADS_Reset(void) {
    if (!is_reset_asserted) {
        gpio_set_level(ADS_RST_PIN, 0);
        is_reset_asserted = true;
    }
}

void ADS1298_Driver::DeAssertADS_Reset(void) {
    if (is_reset_asserted) {
        gpio_set_level(ADS_RST_PIN, 1);
        is_reset_asserted = false;
    }
}

void ADS1298_Driver::ADS1x9x_Reset(void) {
    ESP_LOGD(TAG, "RST pin state before reset: %s\n", gpio_get_level(ADS_RST_PIN) ? "HIGH" : "LOW");
    
    DeAssertADS_Reset();
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "Asserting reset (RST LOW)...");
    AssertADS_Reset();
    esp_rom_delay_us((uint32_t)(4 * ADS_INTERNAL_CLK_PERIOD));

    ESP_LOGI(TAG, "Deasserting reset (RST HIGH)...");
    DeAssertADS_Reset();
    esp_rom_delay_us((uint32_t)(18 * ADS_INTERNAL_CLK_PERIOD));
    
    ESP_LOGD(TAG, "RST pin state after reset: %s\n", gpio_get_level(ADS_RST_PIN) ? "HIGH" : "LOW");
}



// --- Low-level Communication Functions ---
void ADS1298_Driver::sendCommand(uint8_t command) {
    command_tx_buffer[0] = command;
    
    AssertADS_CS();

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer;
    t.rx_buffer = NULL;
    t.flags = 0;
    
    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    esp_rom_delay_us((uint32_t)(10 * ADS_INTERNAL_CLK_PERIOD));
    ESP_LOGI(TAG, "Command 0x%02X sent\n", command);
}

void ADS1298_Driver::ADS1x9x_Reg_Write (uint8_t Read_write_address, uint8_t Data) { 
    command_tx_buffer[0] = Read_write_address | WREG;
    command_tx_buffer[1] = 0x00;
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
    esp_rom_delay_us((uint32_t)(4 * ADS_INTERNAL_CLK_PERIOD));

    // Byte 2: Number of registers to write - 1
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+1;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us((uint32_t)(4 * ADS_INTERNAL_CLK_PERIOD));

    // Byte 3: Data
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+2;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us((uint32_t)(4 * ADS_INTERNAL_CLK_PERIOD));
    
    ESP_LOGI(TAG, "Writing register 0x%02X: 0x%02X\n", Read_write_address, Data);
}

uint8_t ADS1298_Driver::ADS1x9x_Reg_Read(uint8_t Reg_address) {
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
    esp_rom_delay_us((uint32_t)(10 * ADS_INTERNAL_CLK_PERIOD));
    
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = command_tx_buffer+1;
    t.rx_buffer = NULL;
    t.flags = 0;

    ret = spi_device_polling_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    // // Short delay for instrction decode in between transmissions
    esp_rom_delay_us((uint32_t)(10 * ADS_INTERNAL_CLK_PERIOD));

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = NULL;
    t.rx_buffer = command_rx_buffer;
    t.flags = 0;

    ret = spi_device_polling_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    uint8_t result = (command_rx_buffer[0]); 
    // uint16_t result = command_rx_buffer[1];
    ESP_LOGI(TAG, "Reg 0x%02X: Read=0x%02X\n", Reg_address, result);

    // Short delay for instrction decode in between transmissions
    esp_rom_delay_us((uint32_t)(10 * ADS_INTERNAL_CLK_PERIOD));

    return result; 
}

uint8_t ADS1298_Driver::GetADSId(void) {
    return (ADS1x9x_Reg_Read(REG_DEVID));
}

bool ADS1298_Driver::verifyRegisterConfiguration(void) {
    ESP_LOGI(TAG, "=== Register Configuration Verification ===");
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
        
        ESP_LOGD(TAG, "Reg 0x%02X: Expected=0x%02X, Read=0x%02X %s - %s", 
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
    ESP_LOGI(TAG, "Verifying channel settings...");
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t readValue = ADS1x9x_Reg_Read(REG_CH1SET + i);
        bool isCorrect = (readValue == 0x00);
        
        ESP_LOGI(TAG, "CH%d (0x%02X): Expected=0x00, Read=0x%02X %s\n", 
                     i+1, REG_CH1SET + i, readValue, isCorrect ? "✓" : "✗");
        
        if (!isCorrect) {
            allRegistersCorrect = false;
        }
    }
    
    if (allRegistersCorrect) {
        ESP_LOGI(TAG, "✓ All registers verified successfully!");
    } else {
        ESP_LOGE(TAG, "✗ Some registers have incorrect values!");
        ESP_LOGE(TAG, "Troubleshooting tips:");
        ESP_LOGE(TAG, "1. Check SPI communication");
        ESP_LOGE(TAG, "2. Verify power supply stability");
        ESP_LOGE(TAG, "3. Check for timing issues");
        ESP_LOGE(TAG, "4. Verify register write sequence");
    }
    
    ESP_LOGI(TAG, "=== Register Verification Complete ===\n\n\n");
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
    ESP_LOGI(TAG, "Start ECG sampling.");
}

void ADS1298_Driver::Soft_Stop_ADS1x9x (void) {
    sendCommand(STOP);
    ESP_LOGI(TAG, "Stop ECG sampling.");
}

// --- Default Register Read (verify register read after reset) ---

void ADS1298_Driver::readDefaultRegistersAfterReset(void) {
    ESP_LOGI(TAG, "=== Reading Default Registers After Reset ===");
    ESP_LOGI(TAG, "Register | Read Value | Expected (datasheet) | Status");
    ESP_LOGI(TAG, "---------|------------|---------------------|--------");

    // ADS1298 default values per datasheet Table 16 (SBAS459K)
    struct RegDefault { uint8_t addr; uint8_t expected; const char* name; };
    RegDefault defaults[] = {
        {REG_DEVID,     0x92, "ID (ADS1298)"},
        {REG_CONFIG1,   0x06, "CONFIG1 (HR, 500 SPS, internal clk)"},
        {REG_CONFIG2,   0x40, "CONFIG2 (test signal off)"},
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
        ESP_LOGI(TAG, "  0x%02X   |    0x%02X     |        0x%02X           | %s",
                 defaults[i].addr, readVal, defaults[i].expected,
                 match ? "OK" : "MISMATCH");
    }

    ESP_LOGI(TAG, "---------|------------|---------------------|--------");
    if (mismatchCount == 0) {
        ESP_LOGI(TAG, "All default registers read correctly.");
    } else {
        ESP_LOGW(TAG, "%d register(s) did not match expected defaults. Check SPI/wiring.", mismatchCount);
    }
    ESP_LOGI(TAG, "=== Default Register Read Complete ===\n");
}

// --- Initialization and Configuration (Matching provided C code) ---

void ADS1298_Driver::ADS1x9x_PowerOn_Init(void) {
    ESP_LOGI(TAG, "=== ADS1298 Power-On Initialization ===");
    
    // 1. Reset sequence (partially done in begin(), repeated here as per original)
    ESP_LOGI(TAG, "Step 1: Performing hardware reset sequence...");
    ADS1x9x_Reset();
    ESP_LOGI(TAG, "hardware reset sequence completed");
    
    // 2. Stop Continuous Mode for configuration
    ESP_LOGI(TAG, "Step 2: Stopping continuous mode for configuration...");
    Stop_Read_Data_Continuous(); 
    ESP_LOGI(TAG, "Continuous mode stopped");

    // 3. Read default registers after reset (verify register read works correctly)
    readDefaultRegistersAfterReset();

    // 4. Configuration for Internal Test Signal (First Pass)
    ESP_LOGI(TAG, "Step 4: Configuring for internal test signal...");
    ESP_LOGI(TAG, "Writing configuration registers...");
    ADS1x9x_Reg_Write(REG_CONFIG1, 0xA4); // High-Resolution mode, Ext Clock, 2kSPS
    ADS1x9x_Reg_Write(REG_CONFIG2, 0x31); // Internal Test enable, f = 2 Hz
    ADS1x9x_Reg_Write(REG_CONFIG3, 0xCC); // Internal Ref, RLD buffer enabled
    ADS1x9x_Reg_Write(REG_RLD_SENSP, 0xFF); // RLD as average of all P-side
    ADS1x9x_Reg_Write(REG_RLD_SENSN, 0xFF); // RLD as average of all N-side
    ADS1x9x_Reg_Write(REG_PACE, 0x01); 
    ADS1x9x_Reg_Write(REG_WCT1, 0x09); 
    ADS1x9x_Reg_Write(REG_WCT2, 0xD0); 
    
    // Lead-off Detection (Test Pass)
    ESP_LOGI(TAG, "Configuring lead-off detection...");
    ADS1x9x_Reg_Write(REG_LOFF, 0x07); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); // Lo-off Enabled, Continuous mode
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Set All Channels to Internal Test Signal 
    ESP_LOGI(TAG, "Setting all channels to internal test signal...");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x35); // PGA = 3, Internal Test Signal
    }
    ESP_LOGI(TAG, "Channel configuration completed");
    
    // 5. Start Conversion (Test Pass)
    ESP_LOGI(TAG, "Step 5: Starting test conversion...");
    Soft_Start_ReStart_ADS1x9x();
    
    Start_Read_Data_Continuous(); 
    
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Test conversion completed");
    
    // 6. Stop and Reconfigure for Normal ECG
    ESP_LOGI(TAG, "Step 6: Reconfiguring for normal ECG...");
    Stop_Read_Data_Continuous(); 

    Soft_Stop_ADS1x9x();
    
    // Lead-off Detection (Normal ECG Pass)
    ESP_LOGI(TAG, "Configuring lead-off detection for normal ECG...");
    ADS1x9x_Reg_Write(REG_LOFF, 0xA3); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Channel Setting (Normal ECG)
    ESP_LOGI(TAG, "Setting channels for normal ECG...");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x00); // PGA = 6, Normal electrode input 
    }
    ESP_LOGI(TAG, "Normal ECG channel configuration completed");
    
    ESP_LOGI(TAG, "=== ADS1298 Power-On Initialization Complete ===\n\n\n");
}

// --- Data Acquisition Task and ISR ---
void IRAM_ATTR ADS1298_Driver::readDataFromDRDY_ISR_static(void* arg) {
    // Call the member function
    static_cast<ADS1298_Driver*>(arg)->readDataFromDRDY_ISR();
}

void IRAM_ATTR ADS1298_Driver::readDataFromDRDY_ISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_drdy_semaphore, &xHigherPriorityTaskWoken);
    
    // If giving the semaphore woke a higher priority task, request a context switch.
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void ADS1298_Driver::_dma_read_task_wrapper(void* arg) {
    // Register this task with the Task Watchdog Timer
    esp_task_wdt_add(NULL);  // Add current task to TWDT
    
    // Call the member function
    static_cast<ADS1298_Driver*>(arg)->_dma_read_task();
}

void ADS1298_Driver::_dma_read_task() {
    spi_transaction_t spi_trans;
    esp_err_t ret;

    // Prepare the transaction once
    memset(&spi_trans, 0, sizeof(spi_trans));
    spi_trans.length = RAW_ECG_SAMPLE_SIZE * 8; // Length in bits
    // spi_trans.tx_buffer is NULL because we only want to receive
    
    ESP_LOGI(TAG, "DMA read task started and waiting for DRDY.");

    for (;;) {
        // Wait for the DRDY interrupt to give the semaphore
        if (xSemaphoreTake(_drdy_semaphore, portMAX_DELAY) == pdTRUE) {
            
            // The rx_buffer will point to the correct position in the circular buffer
            spi_trans.rx_buffer = &RawECGBuffer[RawECGBufferWriteIndex];

            // Queue the transaction
            ret = spi_device_queue_trans(_spi_device, &spi_trans, portMAX_DELAY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to queue SPI transaction. Error: %s\n", esp_err_to_name(ret));
                continue;
            }
            
            // This is not a polling transaction, so we need to wait for it to complete.
            spi_transaction_t* rtrans;
            ret = spi_device_get_trans_result(_spi_device, &rtrans, portMAX_DELAY);
            if (ret != ESP_OK) {
                 ESP_LOGE(TAG, "Failed to get transaction result. Error: %s\n", esp_err_to_name(ret));
                 continue;
            }

            // Update indices and counts (using volatile members)
            RawECGBufferWriteIndex += RAW_ECG_SAMPLE_SIZE;
            RawECGBufferWriteSampleNum++;
            
            // Circular buffer reset
            if (RawECGBufferWriteIndex >= RAW_ECG_BUFFER_SIZE) {
                RawECGBufferWriteIndex = 0; 
            }
            if (RawECGBufferWriteSampleNum >= RAW_ECG_BUFFER_FULL_SAMPLE_NUM) {
                RawECGBufferWriteSampleNum = 0; 
            }

            // Sampling started flag logic
            if (!IsStartedSampling) {
                if (RawECGBufferWriteSampleNum > 1) {
                    IsStartedSampling = 1; 
                }
            }
        }
    }
}

ADS1298_Driver::~ADS1298_Driver() {
    ESP_LOGI(TAG, "=== ADS1298 Driver Destruction ===");
    
    // 1. Stop tasks and interrupts
    if (_dma_read_task_handle != nullptr) {
        vTaskDelete(_dma_read_task_handle);
        _dma_read_task_handle = nullptr;
        ESP_LOGI(TAG, "DMA read task deleted");
    }
    
    // 2. Remove GPIO interrupt
    gpio_isr_handler_remove(ADS_DRDY_PIN);
    ESP_LOGI(TAG, "GPIO ISR handler removed");
    
    // 3. Free allocated memory
    if (command_tx_buffer != nullptr) {
        heap_caps_free(command_tx_buffer);
        command_tx_buffer = nullptr;
        ESP_LOGI(TAG, "TX buffer freed");
    }
    
    if (command_rx_buffer != nullptr) {
        heap_caps_free(command_rx_buffer);
        command_rx_buffer = nullptr;
        ESP_LOGI(TAG, "RX buffer freed");
    }
    
    // 4. Delete semaphore
    if (_drdy_semaphore != nullptr) {
        vSemaphoreDelete(_drdy_semaphore);
        _drdy_semaphore = nullptr;
        ESP_LOGI(TAG, "Semaphore deleted");
    }
    
    // 5. Remove SPI device and bus
    if (_spi_device != nullptr) {
        esp_err_t ret = spi_bus_remove_device(_spi_device);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SPI device removed");
        }
        _spi_device = nullptr;
    }
    
    // 6. Free SPI bus
    esp_err_t ret = spi_bus_free(_spi_host);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI bus freed");
    }
    
    // 7. Clear global instance pointer
    if (ads_driver_instance == this) {
        ads_driver_instance = nullptr;
        ESP_LOGI(TAG, "Global instance pointer cleared");
    }
    
    ESP_LOGI(TAG, "=== ADS1298 Driver Destroyed ===");
}