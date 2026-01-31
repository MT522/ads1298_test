#ifndef ADS1298_DRIVER_H
#define ADS1298_DRIVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- Pin Definitions (Customize for your ESP32 board) ---
// Note: Using ESP-IDF driver, so we need to define all SPI pins.
// Default VSPI pins are used here.
#define ADS_MOSI_PIN    (gpio_num_t)23
#define ADS_MISO_PIN    (gpio_num_t)19
#define ADS_SCLK_PIN    (gpio_num_t)18
#define ADS_CS_PIN      (gpio_num_t)5   // Chip Select (CS)
#define ADS_DRDY_PIN    (gpio_num_t)4   // Data Ready (/DRDY) - Connect to an interrupt-capable pin
#define ADS_RST_PIN     (gpio_num_t)2   // Reset (/RESET)
#define ADS_START_PIN   (gpio_num_t)15  // Start Conversion (/START)

#define ADS_INTERNAL_CLK_PERIOD 0.5 //ADS1298 Internal Master Clock at 2.048MHz => 0.488 us

// --- Data & Buffer Definitions (Matching the STM32 code) ---
#define RAW_ECG_SAMPLE_SIZE      27 // 3 Status bytes + (8 channels * 3 bytes/channel)
#define RAW_ECG_BUFFER_FULL_SAMPLE_NUM 512 // Example buffer size (adjust as needed)
#define RAW_ECG_BUFFER_SIZE      (RAW_ECG_BUFFER_FULL_SAMPLE_NUM * RAW_ECG_SAMPLE_SIZE) 

// --- ADS1298 Command Definitions ---
#define WAKEUP          0x02 // Wake-up from standby mode
#define STANDBY         0x04 // Enter standby mode
#define RESET           0x06 // Reset all registers
#define START           0x08 // Start data conversions
#define STOP            0x0A // Stop data conversions
#define RDATAC          0x10 // Enable Read Data Continuous mode
#define SDATAC          0x11 // Stop Read Data Continuous mode
#define RDATA           0x12 // Read Data by Command (single shot)
#define RREG            0x20 // Read N registers (0x20 + RADDRESS)
#define WREG            0x40 // Write N registers (0x40 + RADDRESS)

// --- ADS1298 Register Map Addresses ---
#define REG_DEVID         (0x00u)
#define REG_CONFIG1       (0x01u)
#define REG_CONFIG2       (0x02u)
#define REG_CONFIG3       (0x03u)
#define REG_LOFF       	  (0x04u)
#define REG_CH1SET        (0x05u)
#define REG_CH2SET        (0x06u)
#define REG_CH3SET        (0x07u)
#define REG_CH4SET        (0x08u)
#define REG_CH5SET        (0x09u)
#define REG_CH6SET        (0x0Au)
#define REG_CH7SET        (0x0Bu)
#define REG_CH8SET        (0x0Cu)
#define REG_RLD_SENSP     (0x0Du)
#define REG_RLD_SENSN     (0x0Eu)
#define REG_LOFF_SENSP	  (0x0Fu)
#define REG_LOFF_SENSN    (0x10u)
#define REG_LOFF_FLIP     (0x11u)
#define REG_LOFF_STATP    (0x12u)
#define REG_LOFF_STATN    (0x13u)
#define REG_GPIO          (0x14u)
#define REG_PACE          (0x15u)
#define REG_RESP          (0x16u)
#define REG_CONFIG4       (0x17u)
#define REG_WCT1          (0x18u)
#define REG_WCT2          (0x19u)

class ADS1298_Driver {
public:
    ADS1298_Driver(spi_host_device_t spi_host = VSPI_HOST);
    bool begin();

    // Raw data buffer and indexing (Public for ISR access)
    uint8_t RawECGBuffer[RAW_ECG_BUFFER_SIZE];
    volatile uint16_t RawECGBufferWriteSampleNum;
    volatile uint32_t RawECGBufferWriteIndex;
    volatile uint8_t IsStartedSampling;
    
    // Low-level SPI commands
    void sendCommand(uint8_t command);
    uint8_t ADS1x9x_Reg_Read(uint8_t Reg_address);
    void ADS1x9x_Reg_Write(uint8_t Read_write_address, uint8_t Data);

    // ADS System Control Functions (Matching provided names)
    void AssertADS_CS(void);
    void DeAssertADS_CS(void);
    void AssertADS_Reset(void);
    void DeAssertADS_Reset(void);
    void ADS1x9x_Reset(void);
    void AssertADS_Start(void);
    void DeAssertADS_Start(void);
    void Hard_Stop_ADS1x9x (void);
    void Hard_Start_ReStart_ADS1x9x(void);
    void Wake_Up_ADS1x9x (void);
    void Put_ADS1x9x_In_Sleep (void);
    void Soft_Reset_ADS1x9x (void);
    void Soft_Start_ReStart_ADS1x9x (void);
    void Soft_Start_ADS1x9x (void);
    void Soft_Stop_ADS1x9x (void);
    void Start_Read_Data_Continuous (void);
    void Stop_Read_Data_Continuous (void);
    void Read_Data_by_Command (void);
    uint8_t GetADSId(void);
    void ADS1x9x_PowerOn_Init(void);
    bool verifyRegisterConfiguration(void);

    // Data Acquisition (Called by ISR, needs to be public)
    void readDataFromDRDY_ISR();
    static void readDataFromDRDY_ISR_static(void* arg);
    
private:
    bool is_cs_asserted = false;
    bool is_start_asserted = false;
    bool is_reset_asserted = true;

    // ESP-IDF SPI handle
    spi_host_device_t _spi_host;
    spi_device_handle_t _spi_device;

    // FreeRTOS handles
    SemaphoreHandle_t _drdy_semaphore;
    TaskHandle_t _dma_read_task_handle;

    // Static task wrapper
    static void _dma_read_task_wrapper(void* arg);
    // Member function for the task logic
    void _dma_read_task();
};

#endif // ADS1298_DRIVER_H