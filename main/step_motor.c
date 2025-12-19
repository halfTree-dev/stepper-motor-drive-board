#include "step_motor.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"

esp_timer_handle_t step_motor_timer;

step_motor_action_t motor_action_queue[MOTOR_QUEUE_SIZE] = {0};
volatile int insert_index = 0;
volatile int action_count = 0;

volatile int current_index = 0;
volatile int activated = 0;

static const char* TAG = "step_motor";

static pcnt_unit_handle_t s_pcnt_unit;
static pcnt_channel_handle_t s_pcnt_chan;
static int s_pcnt_watch_point = -1;

static bool IRAM_ATTR encoder_pcnt_on_reach(pcnt_unit_handle_t unit,
                                           const pcnt_watch_event_data_t* edata,
                                           void* user_ctx)
{
    (void)unit;
    (void)edata;
    (void)user_ctx;

    encoder_reach_limit_task(NULL);
    return false;
}

static step_motor_config_t motor_config = {
    .enabled = ENABLE_MOTOR,
    .micro_step_mode = MICRO_STEP_FULL,
    .decay_mode = DECAY_MODE_SLOW,
    .sleep_mode = WAKE_MOTOR,
    .reset_mode = ACTIVATE_MOTOR
};

int8_t ignore_encoder_input = 0;

static float step_angle = DEFAULT_STEP_ANGLE;
static uint16_t encoder_resolution = DEFAULT_ENCODER_RESOLUTION;

// 初始化部分
void _gpio_init(gpio_num_t target_gpio, gpio_mode_t mode, uint32_t level)
{
    ESP_ERROR_CHECK(gpio_reset_pin(target_gpio));
    ESP_ERROR_CHECK(gpio_set_direction(target_gpio, mode));
    ESP_ERROR_CHECK(gpio_set_level(target_gpio, level));
}

void init_motor_control_pins()
{
    _gpio_init(BUZZER_PIN, GPIO_MODE_OUTPUT, 0);
    _gpio_init(LED_PIN, GPIO_MODE_OUTPUT, 0);

    _gpio_init(DECAY_PIN, GPIO_MODE_OUTPUT, motor_config.decay_mode);
    _gpio_init(NENAL_PIN, GPIO_MODE_OUTPUT, motor_config.enabled);
    _gpio_init(NSLEEP_PIN, GPIO_MODE_OUTPUT, motor_config.sleep_mode);
    _gpio_init(NRESET_PIN, GPIO_MODE_OUTPUT, motor_config.reset_mode);

    _gpio_init(MODE0_PIN, GPIO_MODE_OUTPUT, (motor_config.micro_step_mode >> 0) & 0x01);
    _gpio_init(MODE1_PIN, GPIO_MODE_OUTPUT, (motor_config.micro_step_mode >> 1) & 0x01);
    _gpio_init(MODE2_PIN, GPIO_MODE_OUTPUT, (motor_config.micro_step_mode >> 2) & 0x01);

    _gpio_init(DIR_PIN, GPIO_MODE_OUTPUT, CLOCKWISE);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = STEP_PWM_MODE,
        .timer_num        = STEP_PWM_TIMER,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = STEP_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_LOW_SPEED_MODE
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = STEP_PIN,
        .speed_mode     = STEP_PWM_MODE,
        .channel        = STEP_PWM_CHANNEL,
        .timer_sel      = STEP_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    encoder_pcnt_init();
}

// 电机操作部分
void _step_pwm_set(uint32_t freq_hz, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_freq(STEP_PWM_MODE, STEP_PWM_TIMER, freq_hz));
    ESP_ERROR_CHECK(ledc_set_duty(STEP_PWM_MODE, STEP_PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(STEP_PWM_MODE, STEP_PWM_CHANNEL));
}

void apply_motor_action(step_motor_action_t* action)
{
    ESP_ERROR_CHECK(gpio_set_level(DIR_PIN, action->direction));
    _step_pwm_set(action->pulse_freq_hz, STEP_PWM_DUTY);
}

// 编码器部分
void encoder_pcnt_init(void)
{
    if (s_pcnt_unit != NULL) {
        return;
    }

    pcnt_unit_config_t unit_config = {
        .low_limit = -100,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = ENCODER_PA_PIN,
        .level_gpio_num = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_config, &s_pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        s_pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        s_pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 2000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_config));

    pcnt_event_callbacks_t cbs = {
        .on_reach = encoder_pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(s_pcnt_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_stop(s_pcnt_unit));

}

void encoder_set_upper_limit(uint32_t upper_limit)
{
    encoder_pcnt_init();

    if (upper_limit > 32767) {
        ESP_LOGW(TAG, "PCNT upper limit too large: %lu", (unsigned long)upper_limit);
        upper_limit = 32767;
    }

    if (s_pcnt_watch_point >= 0) {
        pcnt_unit_remove_watch_point(s_pcnt_unit, s_pcnt_watch_point);
        s_pcnt_watch_point = -1;
    }

    s_pcnt_watch_point = (int)upper_limit;
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt_unit, s_pcnt_watch_point));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));
}

void encoder_reach_limit_task(void* arg)
{
    if (motor_action_queue[current_index].rotate_mode != ROTATE_FOREVER) {
        pcnt_unit_clear_count(s_pcnt_unit);
        current_index++;
        if (current_index >= action_count) {
            current_index = 0;
            activated = 0;
            _step_pwm_set(STEP_PWM_FREQ_HZ, 0);
        } else {
            apply_motor_action(&motor_action_queue[current_index]);
        }
    }
}

// 配置部分

void apply_motor_config(step_motor_config_t* config)
{
    motor_config = *config;

    ESP_ERROR_CHECK(gpio_set_level(DECAY_PIN, motor_config.decay_mode));
    ESP_ERROR_CHECK(gpio_set_level(NENAL_PIN, motor_config.enabled));
    ESP_ERROR_CHECK(gpio_set_level(NSLEEP_PIN, motor_config.sleep_mode));
    ESP_ERROR_CHECK(gpio_set_level(NRESET_PIN, motor_config.reset_mode));

    ESP_ERROR_CHECK(gpio_set_level(MODE0_PIN, (motor_config.micro_step_mode >> 0) & 0x01));
    ESP_ERROR_CHECK(gpio_set_level(MODE1_PIN, (motor_config.micro_step_mode >> 1) & 0x01));
    ESP_ERROR_CHECK(gpio_set_level(MODE2_PIN, (motor_config.micro_step_mode >> 2) & 0x01));

}

// 队列操作部分

// 参数中填写公制单位（角度制），但是在结构体中将其转化为步数单位
void add_new_motor_action(uint8_t direction, uint8_t rotate_mode, float rotate_angular_speed, float target_angle)
{
    step_motor_action_t curr_action = {0};

    curr_action.direction = direction;
    curr_action.rotate_mode = rotate_mode;

    // 换算到编码器应当接受的脉冲数
    if (rotate_mode == ROTATE_BY_ANGLE) {
        curr_action.target_pulse_cnt = (uint32_t)(target_angle / 360.0f * encoder_resolution);
    } else {
        curr_action.target_pulse_cnt = 0;
    }
    curr_action.passed_pulse_cnt = 0;

    // 换算到编码器应当接受的上升脉冲频率
    // 以 step_angle 考虑，每一个上升脉冲代表转过 step_angle 角度
    curr_action.pulse_freq_hz = (uint32_t)(rotate_angular_speed / step_angle);

    motor_action_queue[insert_index] = curr_action;
    insert_index = (insert_index + 1) % MOTOR_QUEUE_SIZE;
    action_count++;
    ESP_LOGI("step_motor", "Added motor action: DIR=%d MODE=%d FREQ=%d TARGET_PULSE=%d",
             curr_action.direction,
             curr_action.rotate_mode,
             curr_action.pulse_freq_hz,
             curr_action.target_pulse_cnt);
}

step_motor_action_t* read_current_motor_action()
{
    if (action_count <= 0) {
        return NULL;
    }
    return &motor_action_queue[current_index];
}

void activate_motor_actions()
{
    if (action_count <= 0) {
        return;
    }
    current_index = 0;
    activated = 1;
    apply_motor_action(&motor_action_queue[current_index]);
    encoder_set_upper_limit(motor_action_queue[current_index].target_pulse_cnt);
}

void clear_motor_actions()
{
    insert_index = 0;
    current_index = 0;
    action_count = 0;
    activated = 0;
}

void stop_motor_actions()
{
    activated = 0;
    _step_pwm_set(STEP_PWM_FREQ_HZ, 0);
}