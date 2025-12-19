#include "uart_device.h"

esp_timer_handle_t uart_timer;
uint8_t *uart_data;
uart_content_t uart_content_buffer[UART_CONTENT_BUFFER_SIZE];
volatile int write_index = 0;
volatile int read_index = 0;
volatile int content_count = 0;

uart_port_t opened_uart_ports[UART_PORTS_MAX];
volatile int opened_uart_count = 0;

void uart_timer_callback(void* arg)
{
    for (int i = 0; i < opened_uart_count; ++i) {
        uart_port_t uart_num = opened_uart_ports[i];
        int len = uart_read_bytes(uart_num, uart_data, UART_BUF_SIZE - 1, 0);
        if (len > 0 && content_count < UART_CONTENT_BUFFER_SIZE) {
            memcpy(uart_content_buffer[write_index].data, uart_data, len);
            uart_content_buffer[write_index].uart_num = uart_num;
            uart_content_buffer[write_index].data[len] = '\0';
            uart_content_buffer[write_index].length = len;
            write_index = (write_index + 1) % UART_CONTENT_BUFFER_SIZE;
            content_count++;
        }
    }
}

void uart_timer_service_init(void)
{
    uart_data = (uint8_t *) malloc(UART_BUF_SIZE);
    const esp_timer_create_args_t timer_args = {
        .callback = &uart_timer_callback,
        .name = "uart_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &uart_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(uart_timer, UART_TIMER_INTERVAL_US));
}

void uart_init(uart_port_t uart_num)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (opened_uart_count < UART_PORTS_MAX) {
        opened_uart_ports[opened_uart_count++] = uart_num;
    }
}

uart_content_t* uart_read(void)
{
    if (content_count <= 0) {
        return NULL;
    }
    uart_content_t* cmd = &uart_content_buffer[read_index];
    read_index = (read_index + 1) % UART_CONTENT_BUFFER_SIZE;
    content_count--;
    return cmd;
}