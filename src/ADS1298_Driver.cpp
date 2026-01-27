#include "ADS1298_Driver.h"

static const char* TAG = "ADS129x_Driver";

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
    gpio_set_direction(ADS_DRDY_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ADS_DRDY_PIN, GPIO_PULLUP_ONLY);

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

    // 4. Hardware Reset
    ESP_LOGI(TAG, "Performing hardware reset...");
    ADS1x9x_Reset();
    ESP_LOGI(TAG, "Hardware reset completed");
    
    // 5. Power-on Initialization and Configuration
    Serial.println("Starting power-on initialization...");
    ADS1x9x_PowerOn_Init();
    Serial.println("Power-on initialization completed");

    // // 6. Verify register configuration
    Serial.println("Verifying register configuration...");
    bool registersVerified = verifyRegisterConfiguration();
    if (!registersVerified) {
        Serial.println("✗ Register verification failed. Device may not be properly configured.");
        return false;
    }
    Serial.println("✓ Register verification passed.");

    sendCommand(SDATAC);
    delay(10);

    // 7. Read ID register to verify communication
    Serial.println("Reading device ID register...");
    uint8_t id = GetADSId();
    Serial.printf("Device ID read: 0x%02X\n", id);
    
    if (id == 0x92) { // ADS1298 ID
        Serial.printf("✓ ADS1298 found (ID: 0x%02X). Driver initialized successfully.\n", id);
        return true;
    } else {
        Serial.printf("✗ ADS1298 not found (ID: 0x%02X). Check wiring and connections.\n", id);
        Serial.println("Expected ID should be 0x92 for ADS1298");
        Serial.println("Troubleshooting tips:");
        Serial.println("1. Check SPI connections (MOSI, MISO, SCLK, CS)");
        Serial.println("2. Verify power supply (3.3V)");
        Serial.println("3. Check PWDN pin is HIGH");
        Serial.println("4. Verify RST pin connection");
        return false;
    }
}



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
    ESP_LOGI(TAG, "Performing ADS1298 hardware reset sequence...");
    ESP_LOGD(TAG, "RST pin state before reset: %s\n", gpio_get_level(ADS_RST_PIN) ? "HIGH" : "LOW");
    
    DeAssertADS_Reset();
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "Asserting reset (RST LOW)...");
    AssertADS_Reset();
    vTaskDelay(pdMS_TO_TICKS(4 * ADS_INTERNAL_CLK_PERIOD));

    ESP_LOGI(TAG, "Deasserting reset (RST HIGH)...");
    DeAssertADS_Reset();
    vTaskDelay(pdMS_TO_TICKS(4 * ADS_INTERNAL_CLK_PERIOD));
    
    ESP_LOGD(TAG, "RST pin state after reset: %s\n", gpio_get_level(ADS_RST_PIN) ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "Hardware reset sequence completed");
}



// --- Low-level Communication Functions ---
void ADS1298_Driver::sendCommand(uint8_t command) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = command;
    t.flags = SPI_TRANS_USE_TXDATA; // Use tx_data member instead of a buffer
    
    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);
    Serial.printf("Command 0x%02X sent\n", command);
}

void ADS1298_Driver::ADS1x9x_Reg_Write (uint8_t Read_write_address, uint8_t Data) { 
    AssertADS_CS();
    // Delay copied from STM32 implementation (HAL_Delay(10))
    // Using vTaskDelay for RTOS-friendly delay
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t ret;
    spi_transaction_t t;

    // Byte 1: Opcode (Register Address | WREG)
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Read_write_address | WREG;
    t.flags = SPI_TRANS_USE_TXDATA;
    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);
    // Short delay mimicking STM32's busy-wait loop
    delayMicroseconds(5);

    // Byte 2: Number of registers to write - 1
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = 0; // We write one register at a time
    t.flags = SPI_TRANS_USE_TXDATA;
    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);
    delayMicroseconds(5);

    // Byte 3: Data
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_data[0] = Data;
    t.flags = SPI_TRANS_USE_TXDATA;
    ret = spi_device_transmit(_spi_device, &t);
    assert(ret == ESP_OK);

    DeAssertADS_CS();
    
    Serial.printf("Writing register 0x%02X: 0x%02X\n", Read_write_address, Data);
}

uint8_t ADS1298_Driver::ADS1x9x_Reg_Read(uint8_t Reg_address) {
    AssertADS_CS();
    vTaskDelay(pdMS_TO_TICKS(1));

    spi_transaction_t t;
    esp_err_t ret;
    
    memset(&t, 0, sizeof(t));
    t.length = 1 * 8; // 1 byte
    t.tx_data[0] = Reg_address | RREG; // RREG Opcode + Address
    t.rx_buffer = NULL;
    t.flags = SPI_TRANS_USE_TXDATA;
    
    ret = spi_device_polling_transmit(_spi_device, &t);
    if (ret != ESP_OK) {
        // ... error handling
    }
    
    // Delay required for ADS129x command decode (1.9 us at 2MHz MCLK)
    delayMicroseconds(2); 
    
    memset(&t, 0, sizeof(t));
    t.length = 1 * 8; // 1 byte (8 bits)
    t.tx_data[0] = 0; // N-1 = 0 (read one register)
    t.flags = SPI_TRANS_USE_TXDATA;

    ret = spi_device_polling_transmit(_spi_device, &t);
    if (ret != ESP_OK) {
        // ... error handling
    }

    // Delay required for ADS129x command decode (1.9 us at 2MHz MCLK)
    delayMicroseconds(2); 

    memset(&t, 0, sizeof(t));
    t.length = 1 * 8; // 1 byte (8 bits)
    t.tx_data[0] = 0;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    ret = spi_device_polling_transmit(_spi_device, &t);
    if (ret != ESP_OK) {
        // ... error handling
    }
    uint8_t result = t.rx_data[0]; // The data is in the first (and only) byte received
    Serial.printf("Reg 0x%02X: Read=0x%02X\n", Reg_address, result);

    // Delay required for ADS129x command decode (1.9 us at 2MHz MCLK)
    delayMicroseconds(2);

    DeAssertADS_CS();
    return result; 
}

uint8_t ADS1298_Driver::GetADSId(void) {
    return (ADS1x9x_Reg_Read(REG_DEVID));
}

bool ADS1298_Driver::verifyRegisterConfiguration(void) {
    Serial.println("=== Register Configuration Verification ===");
    bool allRegistersCorrect = true;
    
    // Define expected register values (based on ADS1x9x_PowerOn_Init configuration)
    struct RegisterConfig {
        uint8_t address;
        uint8_t expectedValue;
        const char* description;
    };
    
    RegisterConfig expectedRegs[] = {
        {REG_CONFIG1, 0xA4, "Config1: High-Res mode, Ext Clock, 2kSPS"},
        {REG_CONFIG2, 0x31, "Config2: Internal Test enable, f=2Hz"},
        {REG_CONFIG3, 0xCC, "Config3: Internal Ref, RLD buffer enabled"},
        {REG_RLD_SENSP, 0xFF, "RLD_SENSP: RLD from all P-side"}, // Address updated from 0x1D to 0x0D
        {REG_RLD_SENSN, 0xFF, "RLD_SENSN: RLD from all N-side"}, // Address updated from 0x1E to 0x0E
        {REG_PACE, 0x01, "PACE: PACE detection settings"}, // Address updated from 0x13/0x15
        {REG_WCT1, 0x09, "WCT1: Wilson Central Terminal 1"}, // Address updated from 0x20 to 0x18
        {REG_WCT2, 0xD0, "WCT2: Wilson Central Terminal 2"}, // Address updated from 0x21 to 0x19
        {REG_LOFF, 0xA3, "LOFF: Lead-off detection settings"},
        {REG_CONFIG4, 0x02, "Config4: Lead-off enabled"}, // Address updated from 0x06 to 0x17
        {REG_LOFF_SENSP, 0xFF, "LOFF_SENSP: Lead-off sense P"}, // Address updated from 0x15 to 0x0F
        {REG_LOFF_SENSN, 0xFF, "LOFF_SENSN: Lead-off sense N"}  // Address updated from 0x16 to 0x10
    };
    
    // Verify each register
    for (int i = 0; i < sizeof(expectedRegs)/sizeof(expectedRegs[0]); i++) {
        uint8_t readValue = ADS1x9x_Reg_Read(expectedRegs[i].address);
        bool isCorrect = (readValue == expectedRegs[i].expectedValue);
        
        Serial.printf("Reg 0x%02X: Expected=0x%02X, Read=0x%02X %s - %s\n", 
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
    Serial.println("Verifying channel settings...");
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t readValue = ADS1x9x_Reg_Read(REG_CH1SET + i);
        bool isCorrect = (readValue == 0x00);
        
        Serial.printf("CH%d (0x%02X): Expected=0x00, Read=0x%02X %s\n", 
                     i+1, REG_CH1SET + i, readValue, isCorrect ? "✓" : "✗");
        
        if (!isCorrect) {
            allRegistersCorrect = false;
        }
    }
    
    if (allRegistersCorrect) {
        Serial.println("✓ All registers verified successfully!");
    } else {
        Serial.println("✗ Some registers have incorrect values!");
        Serial.println("Troubleshooting tips:");
        Serial.println("1. Check SPI communication");
        Serial.println("2. Verify power supply stability");
        Serial.println("3. Check for timing issues");
        Serial.println("4. Verify register write sequence");
    }
    
    Serial.println("=== Register Verification Complete ===\n\n\n");
    return allRegistersCorrect;
}

// --- ADS System Control (Matching provided C code) ---

void ADS1298_Driver::Wake_Up_ADS1x9x (void) { sendCommand(WAKEUP); }
void ADS1298_Driver::Put_ADS1x9x_In_Sleep (void) { sendCommand(STANDBY); }
void ADS1298_Driver::Soft_Reset_ADS1x9x (void) { sendCommand(RESET); }
void ADS1298_Driver::Soft_Start_ReStart_ADS1x9x (void) { sendCommand(START); }
// void ADS1298_Driver::Soft_Stop_ADS1x9x (void) { sendCommand(STOP); }
void ADS1298_Driver::Start_Read_Data_Continuous (void) { sendCommand(RDATAC); }
void ADS1298_Driver::Stop_Read_Data_Continuous (void) { sendCommand(SDATAC); }
void ADS1298_Driver::Read_Data_by_Command (void) { sendCommand(RDATA); }

void ADS1298_Driver::Hard_Start_ReStart_ADS1x9x(void) {
    ADS1x9x_Enable_Start(); // Set Start pin to High (matching C code logic)
}

void ADS1298_Driver::Hard_Stop_ADS1x9x (void) {
    ADS1x9x_Disable_Start(); 
    delay(14); // Replaces osDelay(14)
}

void ADS1298_Driver::Soft_Start_ADS1x9x (void) {
    Soft_Start_ReStart_ADS1x9x();
    // Replaces Enable_ADS1x9x_DRDY_Interrupt() by attaching the ISR
    attachInterrupt(digitalPinToInterrupt(ADS_DRDY_PIN), drdy_isr_handler, FALLING); 
    // Serial.println("Start ECG sampling."); // Replace SendDebugMessage
}

void ADS1298_Driver::Soft_Stop_ADS1x9x (void) {
    sendCommand(STOP);
    // Replaces Disable_ADS1x9x_DRDY_Interrupt() by detaching the ISR
    detachInterrupt(digitalPinToInterrupt(ADS_DRDY_PIN)); 
    // Serial.println("Stop ECG sampling."); // Replace SendDebugMessage
}

// --- Initialization and Configuration (Matching provided C code) ---

void ADS1298_Driver::ADS1x9x_PowerOn_Init(void) {
    Serial.println("=== ADS1298 Power-On Initialization ===");
    
    // 1. Reset sequence (partially done in begin(), repeated here as per original)
    Serial.println("Step 1: Additional reset sequence...");
    DeAssertADS_Reset();
    delay(1000);
    AssertADS_Reset();
    delay(10);
    DeAssertADS_Reset();
    delay(70); 
    Serial.println("Additional reset sequence completed");
    
    // 2. Stop Continuous Mode for configuration
    Serial.println("Step 2: Stopping continuous mode for configuration...");
    Stop_Read_Data_Continuous(); 
    delay(10);
    Serial.println("Continuous mode stopped");

    // 4. Configuration for Internal Test Signal (First Pass)
    Serial.println("Step 4: Configuring for internal test signal...");
    Serial.println("Writing configuration registers...");
    ADS1x9x_Reg_Write(REG_CONFIG1, 0xA4); // High-Resolution mode, Ext Clock, 2kSPS
    ADS1x9x_Reg_Write(REG_CONFIG2, 0x31); // Internal Test enable, f = 2 Hz
    ADS1x9x_Reg_Write(REG_CONFIG3, 0xCC); // Internal Ref, RLD buffer enabled
    ADS1x9x_Reg_Write(REG_RLD_SENSP, 0xFF); // RLD as average of all P-side
    ADS1x9x_Reg_Write(REG_RLD_SENSN, 0xFF); // RLD as average of all N-side
    ADS1x9x_Reg_Write(REG_PACE, 0x01); 
    ADS1x9x_Reg_Write(REG_WCT1, 0x09); 
    ADS1x9x_Reg_Write(REG_WCT2, 0xD0); 
    
    // Lead-off Detection (Test Pass)
    Serial.println("Configuring lead-off detection...");
    ADS1x9x_Reg_Write(REG_LOFF, 0x07); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); // Lo-off Enabled, Continuous mode
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Set All Channels to Internal Test Signal 
    Serial.println("Setting all channels to internal test signal...");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x35); // PGA = 3, Internal Test Signal
    }
    Serial.println("Channel configuration completed");
    
    // 5. Start Conversion (Test Pass)
    Serial.println("Step 5: Starting test conversion...");
    Soft_Start_ReStart_ADS1x9x();
    delay(10);
    Start_Read_Data_Continuous(); 
    delay(500);
    Serial.println("Test conversion completed");
    
    // 6. Stop and Reconfigure for Normal ECG
    Serial.println("Step 6: Reconfiguring for normal ECG...");
    Stop_Read_Data_Continuous(); 
    delay(10);
    Soft_Stop_ADS1x9x();
    delay(10);
    
    // Lead-off Detection (Normal ECG Pass)
    Serial.println("Configuring lead-off detection for normal ECG...");
    ADS1x9x_Reg_Write(REG_LOFF, 0xA3); 
    ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); 
    ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); 
    
    // Channel Setting (Normal ECG)
    Serial.println("Setting channels for normal ECG...");
    for(uint8_t i = 0; i < 8; i++) {
        ADS1x9x_Reg_Write(REG_CH1SET + i, 0x00); // PGA = 6, Normal electrode input 
    }
    Serial.println("Normal ECG channel configuration completed");
    
    Serial.println("=== ADS1298 Power-On Initialization Complete ===\n\n\n");
}

// --- Data Acquisition Task and ISR ---

void ADS1298_Driver::_dma_read_task_wrapper(void* arg) {
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
    
    Serial.println("DMA read task started and waiting for DRDY.");

    for (;;) {
        // Wait for the DRDY interrupt to give the semaphore
        if (xSemaphoreTake(_drdy_semaphore, portMAX_DELAY) == pdTRUE) {
            
            // The rx_buffer will point to the correct position in the circular buffer
            spi_trans.rx_buffer = &RawECGBuffer[RawECGBufferWriteIndex];

            // Queue the transaction
            ret = spi_device_queue_trans(_spi_device, &spi_trans, portMAX_DELAY);
            if (ret != ESP_OK) {
                Serial.printf("Failed to queue SPI transaction. Error: %s\n", esp_err_to_name(ret));
                continue;
            }
            
            // This is not a polling transaction, so we need to wait for it to complete.
            spi_transaction_t* rtrans;
            ret = spi_device_get_trans_result(_spi_device, &rtrans, portMAX_DELAY);
            if (ret != ESP_OK) {
                 Serial.printf("Failed to get transaction result. Error: %s\n", esp_err_to_name(ret));
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

void IRAM_ATTR ADS1298_Driver::readDataFromDRDY_ISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_drdy_semaphore, &xHigherPriorityTaskWoken);
    
    // If giving the semaphore woke a higher priority task, request a context switch.
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}