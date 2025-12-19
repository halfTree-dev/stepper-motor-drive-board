#pragma once

#include "driver/uart.h"
#include "esp_timer.h"
#include <string.h>
#include "global_params.h"

#define UART_BUF_SIZE 256
#define UART_CONTENT_BUFFER_SIZE 10
#define UART_TIMER_INTERVAL_US 50000
#define UART_PORTS_MAX 6

typedef struct {
    uart_port_t uart_num;
    uint8_t data[UART_BUF_SIZE];
    int length;
} uart_content_t;

void uart_timer_callback(void* arg);
void uart_timer_service_init(void);
void uart_init(uart_port_t uart_num);
uart_content_t* uart_read(void);