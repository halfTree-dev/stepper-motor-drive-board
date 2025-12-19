#include "step_motor.h"
#include "uart_device.h"
#include "global_params.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

const static char* TAG = "main";

step_motor_config_t curr_config = {
    .enabled = ENABLE_MOTOR,
    .micro_step_mode = MICRO_STEP_FULL,
    .decay_mode = DECAY_MODE_SLOW,
    .sleep_mode = WAKE_MOTOR,
    .reset_mode = ACTIVATE_MOTOR
};

static char* str_trim(char* s)
{
    if (s == NULL) {
        return NULL;
    }
    while (*s != '\0' && isspace((unsigned char)*s)) {
        s++;
    }
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        n--;
    }
    s[n] = '\0';
    return s;
}

static int parse_u32(const char* s, uint32_t* out)
{
    if (s == NULL || out == NULL) {
        return 0;
    }
    errno = 0;
    char* end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || errno != 0 || *str_trim(end) != '\0') {
        return 0;
    }
    *out = (uint32_t)v;
    return 1;
}

static int parse_f32(const char* s, float* out)
{
    if (s == NULL || out == NULL) {
        return 0;
    }

    errno = 0;
    char* end = NULL;
    float v = strtof(s, &end);
    if (end == s || errno != 0 || *str_trim(end) != '\0') {
        return 0;
    }
    *out = v;
    return 1;
}

static int parse_direction(const char* s, uint8_t* out_direction)
{
    if (s == NULL || out_direction == NULL) {
        return 0;
    }
    if (strcmp(s, "CW") == 0) {
        *out_direction = CLOCKWISE;
        return 1;
    }
    if (strcmp(s, "CCW") == 0) {
        *out_direction = COUNTER_CLOCKWISE;
        return 1;
    }
    return 0;
}

static void do_reset_pulse(void)
{
    ESP_ERROR_CHECK(gpio_set_level(NRESET_PIN, RESET_MOTOR));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(gpio_set_level(NRESET_PIN, ACTIVATE_MOTOR));
}

static void do_buzzer_beep(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BUZZER_PIN, 0);
}

static void handle_set_config(uart_port_t uart_num, char* param, char* value)
{
    if (param == NULL || value == NULL) {
        ESP_LOGE(TAG, "UART%d: SET_CONFIG <PARAM> <VALUE>", uart_num);
        return;
    }

    uint32_t v = 0;
    if (!parse_u32(value, &v)) {
        ESP_LOGE(TAG, "UART%d: BAD_VALUE", uart_num);
        return;
    }

    if (strcmp(param, "ENABLE") == 0) {
        if (v > 1) {
            ESP_LOGE(TAG, "UART%d: ENABLE 0|1", uart_num);
            return;
        }
        curr_config.enabled = (uint8_t)v;
    } else if (strcmp(param, "MICRO_STEP_MODE") == 0) {
        if (v > 5) {
            ESP_LOGE(TAG, "UART%d: MICRO_STEP_MODE 0-5", uart_num);
            return;
        }
        curr_config.micro_step_mode = (uint8_t)v;
    } else if (strcmp(param, "DECAY_MODE") == 0) {
        if (v > 1) {
            ESP_LOGE(TAG, "UART%d: DECAY_MODE 0|1", uart_num);
            return;
        }
        curr_config.decay_mode = (uint8_t)v;
    } else if (strcmp(param, "SLEEP_MODE") == 0) {
        if (v > 1) {
            ESP_LOGE(TAG, "UART%d: SLEEP_MODE 0|1", uart_num);
            return;
        }
        curr_config.sleep_mode = (uint8_t)v;
    } else if (strcmp(param, "RESET_MODE") == 0) {
        if (v > 1) {
            ESP_LOGE(TAG, "UART%d: RESET_MODE 0|1", uart_num);
            return;
        }
        curr_config.reset_mode = (uint8_t)v;
    } else {
        ESP_LOGE(TAG, "UART%d: UNKNOWN_PARAM", uart_num);
        return;
    }

    apply_motor_config(&curr_config);
    ESP_LOGI(TAG, "UART%d: OK", uart_num);
}

static void handle_move(uart_port_t uart_num, char* dir_s, char* angle_or_forever, char* speed_s)
{
    uint8_t direction = CLOCKWISE;
    if (!parse_direction(dir_s, &direction)) {
        ESP_LOGE(TAG, "UART%d: MOVE <CW|CCW> ...", uart_num);
        return;
    }

    if (angle_or_forever == NULL || speed_s == NULL) {
        ESP_LOGE(TAG, "UART%d: MOVE <CW|CCW> <ANGLE|FOREVER> <SPEED>", uart_num);
        return;
    }

    float speed = 0.0f;
    if (!parse_f32(speed_s, &speed) || speed <= 0.0f) {
        ESP_LOGE(TAG, "UART%d: BAD_SPEED", uart_num);
        return;
    }

    if (strcmp(angle_or_forever, "FOREVER") == 0) {
        add_new_motor_action(direction, ROTATE_FOREVER, speed, 0.0f);
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        ESP_LOGI(TAG, "ACTION SPEED: %.2f MODE: ROTATE_FOREVER", speed);
        return;
    }

    float angle = 0.0f;
    if (!parse_f32(angle_or_forever, &angle) || angle <= 0.0f) {
        ESP_LOGE(TAG, "UART%d: BAD_ANGLE", uart_num);
        return;
    }

    add_new_motor_action(direction, ROTATE_BY_ANGLE, speed, angle);
    ESP_LOGI(TAG, "ACTION SPEED: %.2f ANGLE: %.2f MODE: ROTATE_BY_ANGLE", speed, angle);
    ESP_LOGI(TAG, "UART%d: OK", uart_num);
}

static void dispatch_uart_command(uart_port_t uart_num, uint8_t* buf, int len)
{
    if (buf == NULL || len <= 0) {
        return;
    }

    char line[UART_BUF_SIZE];
    int n = len;
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1;
    }
    memcpy(line, buf, n);
    line[n] = '\0';

    char* cmdline = str_trim(line);
    if (cmdline[0] == '\0') {
        return;
    }

    char* saveptr = NULL;
    char* cmd = strtok_r(cmdline, " \t", &saveptr);
    if (cmd == NULL) {
        return;
    }

    if (strcmp(cmd, "RESET") == 0) {
        ESP_LOGI(TAG, "UART%d: RESET", uart_num);
        do_reset_pulse();
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        return;
    }

    if (strcmp(cmd, "BUZZER") == 0) {
        ESP_LOGI(TAG, "UART%d: BUZZER", uart_num);
        do_buzzer_beep();
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        return;
    }

    if (strcmp(cmd, "SET_CONFIG") == 0) {
        char* param = strtok_r(NULL, " \t", &saveptr);
        char* value = strtok_r(NULL, " \t", &saveptr);
        handle_set_config(uart_num, param, value);
        return;
    }

    if (strcmp(cmd, "MOVE") == 0) {
        char* dir_s = strtok_r(NULL, " \t", &saveptr);
        char* angle_or_forever = strtok_r(NULL, " \t", &saveptr);
        char* speed_s = strtok_r(NULL, " \t", &saveptr);
        handle_move(uart_num, dir_s, angle_or_forever, speed_s);
        return;
    }

    if (strcmp(cmd, "ACTIVATE") == 0) {
        activate_motor_actions();
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        return;
    }

    if (strcmp(cmd, "CLEAR") == 0) {
        clear_motor_actions();
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        return;
    }

    if (strcmp(cmd, "STOP") == 0) {
        stop_motor_actions();
        ESP_LOGI(TAG, "UART%d: OK", uart_num);
        return;
    }

    ESP_LOGE(TAG, "UART%d: UNKNOWN_CMD '%s' (ONLY UPPERCASE)", uart_num, cmd);
}

void app_main(void)
{
    uart_init(UART_NUM_0);
    uart_timer_service_init();

    init_motor_control_pins();
    apply_motor_config(&curr_config);
    encoder_pcnt_init();
    stop_motor_actions();

    while (1) {
        uart_content_t* content = uart_read();
        if (content != NULL) {
            dispatch_uart_command(content->uart_num, content->data, content->length);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}