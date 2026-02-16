/*
 * uart.h
 *
 *  Created on: Feb 8, 2026
 *      Author: Administrator
 */

#ifndef UART_H_
#define UART_H_

#include "driver/uart.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>

// UART for SerialPlot and logging (UART0)
#define UART_NUM              UART_NUM_0
#define UART_BAUD_RATE        115200
#define UART_TX_PIN           1      // GPIO1 is typically TX for UART0
#define UART_RX_PIN           3      // GPIO3 is typically RX for UART0

// Buffer sizes - TX buffer must hold at least one full packet (~1350 bytes) to avoid
// blocking uart_write_bytes and starving the Idle task (Task WDT).
#define UART_BUFFER_SIZE      1536
#define LOG_BUFFER_SIZE       128

// Custom logging function
static void uart_printf(const char* format, ...) {
    char buffer[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        uart_write_bytes(UART_NUM, buffer, (size_t)len);
    }
}

static void uart_println(const char* message) {
    uart_write_bytes(UART_NUM, message, strlen(message));
    uart_write_bytes(UART_NUM, "\n", 1);
}

// Initialize UART
static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);
    
    // Set UART pins
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Install UART driver - TX buffer 1536 bytes to fit one packet and avoid blocking (Task WDT)
    uart_driver_install(UART_NUM, 256, UART_BUFFER_SIZE, 0, NULL, 0);
    
    // Clear any pending data
    uart_flush(UART_NUM);
    
    // Print initialization message
    uart_printf("\n\n========================================\n");
    uart_printf("ESP32 ADS1298 ECG Monitor\n");
    uart_printf("========================================\n");
    uart_printf("UART0 initialized at %d baud\n", UART_BAUD_RATE);
}


#endif
