#include <pwm_control.h>

// Defines
#define MOTOR_TIMEBASE_RESOLUTION_HZ    10000000
#define MOTOR_TIMEBASE_PERIOD_TICKS     1000

// Handles
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;
mcpwm_cmpr_handle_t comparator;
mcpwm_gen_handle_t generator;

// Function prototypes
void mcpwm_create_timer(void);
void mcpwm_create_operator(void);
void mcpwm_connect_operator_with_timer(void);
void mcpwm_create_comparator(void);
void mcpwm_create_generator(void);
void mcpwm_start_timer(void);
void gpio_pin_configuration(void);
void motor_forward(void);
void motor_backward(void);
void motor_stop(void);

// Functions
void mcpwm_init(void){
    mcpwm_create_timer();
    mcpwm_create_operator();
    mcpwm_connect_operator_with_timer();
    mcpwm_create_comparator();
    mcpwm_create_generator();
    gpio_pin_configuration();
    mcpwm_start_timer();
}

void mcpwm_create_timer(void){
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
        .resolution_hz = MOTOR_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = MOTOR_TIMEBASE_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
}

void mcpwm_create_operator(void){
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, 
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator));
}

void mcpwm_connect_operator_with_timer(void){
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, timer));
}

void mcpwm_create_comparator(void){
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &comparator));
}

void mcpwm_create_generator(void){
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PIN_ENA,
    };

    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &generator));

    // Set the generator to start high and go low when the comparator matches
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
}

void mcpwm_start_timer(void){
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void mcpwm_set_value_to_compare(uint32_t value){
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, value));
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