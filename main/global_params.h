#pragma once

#include "driver/gpio.h"
#include "esp_log.h"

#define STEP_PIN GPIO_NUM_7
#define DIR_PIN GPIO_NUM_16

#define BUZZER_PIN GPIO_NUM_3
#define LED_PIN GPIO_NUM_1

#define DECAY_PIN GPIO_NUM_17
#define NENAL_PIN GPIO_NUM_15
#define NSLEEP_PIN GPIO_NUM_18
#define NRESET_PIN GPIO_NUM_8

#define MODE2_PIN GPIO_NUM_4
#define MODE1_PIN GPIO_NUM_5
#define MODE0_PIN GPIO_NUM_6

#define ENCODER_PA_PIN GPIO_NUM_42
#define ENCODER_PB_PIN GPIO_NUM_40