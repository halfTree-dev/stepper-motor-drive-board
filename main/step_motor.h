#pragma once

#include <stdint.h>
#include "esp_timer.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_params.h"

#define STEP_MOTOR_TIMER_INTERVAL_US 1000

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define ROTATE_BY_ANGLE 0
#define ROTATE_FOREVER 1

#define DEFAULT_STEP_ANGLE 1.8f
#define DEFAULT_ENCODER_RESOLUTION 1000

#define MOTOR_QUEUE_SIZE 10

typedef struct {
    uint8_t direction;

    uint8_t rotate_mode;
    uint32_t pulse_freq_hz;

    uint32_t target_pulse_cnt;
    uint32_t passed_pulse_cnt;
} step_motor_action_t;


#define MICRO_STEP_FULL        0
#define MICRO_STEP_HALF        1
#define MICRO_STEP_QUARTER     2
#define MICRO_STEP_EIGHTH      3
#define MICRO_STEP_SIXTEENTH   4
#define MICRO_STEP_THIRTY2ND   5

#define DECAY_MODE_SLOW        0
#define DECAY_MODE_FAST        1

#define ENABLE_MOTOR           0
#define DISABLE_MOTOR          1

#define SLEEP_MOTOR            0
#define WAKE_MOTOR             1

#define RESET_MOTOR            0
#define ACTIVATE_MOTOR         1

typedef struct {
    uint8_t enabled;
    uint8_t micro_step_mode;
    uint8_t decay_mode;
    uint8_t sleep_mode;
    uint8_t reset_mode;
} step_motor_config_t;

#define STEP_PWM_TIMER      LEDC_TIMER_0
#define STEP_PWM_MODE       LEDC_LOW_SPEED_MODE
#define STEP_PWM_CHANNEL    LEDC_CHANNEL_0
#define STEP_PWM_FREQ_HZ    50
#define STEP_PWM_DUTY       8192

void init_motor_control_pins();
void encoder_pcnt_init();
void encoder_set_upper_limit(uint32_t upper_limit);
void encoder_reach_limit_task(void* arg);
void apply_motor_config(step_motor_config_t* config);
void apply_motor_action(step_motor_action_t* action);
void add_new_motor_action(uint8_t direction, uint8_t rotate_mode, float rotate_angular_speed, float target_angle);
step_motor_action_t* read_current_motor_action();
void activate_motor_actions();
void clear_motor_actions();
void stop_motor_actions();