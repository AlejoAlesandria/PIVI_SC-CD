#include <pwm_control_ledc.h>

// Defines


// Handles


// Function prototypes
void ledc_timer_configuration(void);
void ledc_channel_configuration(void);

void gpio_pin_configuration(void);
void motor_forward(void);
void motor_backward(void);
void motor_stop(void);

// Functions
void ledc_init(void){
    ledc_timer_configuration();
    ledc_channel_configuration();
    gpio_pin_configuration();
}

void ledc_timer_configuration(void){
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 100,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
}

void ledc_channel_configuration(void){
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_ENA,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4095, 0));
}

void gpio_pin_configuration(void){
    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);
}

void motor_forward(void){
    gpio_set_level(PIN_IN1, 1);
    gpio_set_level(PIN_IN2, 0);
}

void motor_backward(void){
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 1);
}

void motor_stop(void){
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);
}