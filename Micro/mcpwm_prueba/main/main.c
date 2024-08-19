#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/gpio.h"
#include "time.h"

// Handlers
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;
mcpwm_cmpr_handle_t comparator;
mcpwm_gen_handle_t generator;

// Parámetros para la señal PN
#define PN_SEQ_LENGTH 1000  // Longitud de la secuencia PN
#define PIN_IN1 6
#define PIN_IN2 5

// Función para generar una señal PN simple (Ejemplo)
void generate_pn_sequence(uint32_t *sequence, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        // Genera una secuencia binaria pseudoaleatoria (0 o 1)
        sequence[i] = rand() % 2;
    }
}

void mcpwm_create_timer(void){
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 1000,
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
        .gen_gpio_num = GPIO_NUM_7,
    };

    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &generator));
}

void mcpwm_start_timer(void){
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void mcpwm_set_value_to_compare(uint32_t value){
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, value));
}

void app_main(void){
    uint32_t pn_sequence[PN_SEQ_LENGTH];
    size_t pn_index = 0;

    // Inicializa la secuencia PN
    srand((unsigned int)time(NULL)); // Inicializa el generador de números aleatorios
    generate_pn_sequence(pn_sequence, PN_SEQ_LENGTH);

    mcpwm_create_timer();
    mcpwm_create_operator();
    mcpwm_connect_operator_with_timer();
    mcpwm_create_comparator();
    mcpwm_create_generator();
    mcpwm_start_timer();
    mcpwm_set_value_to_compare(0);

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);

    while(true){
        // Actualiza el valor de comparación con la secuencia PN
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        mcpwm_set_value_to_compare(1000);
        //mcpwm_set_value_to_compare(pn_sequence[pn_index] * 1000); // Escala la secuencia PN
        //pn_index = (pn_index + 1) % PN_SEQ_LENGTH;  // Avanza al siguiente valor en la secuencia
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Ajusta el retraso según sea necesario
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        mcpwm_set_value_to_compare(1000);
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Ajusta el retraso según sea necesario
        mcpwm_set_value_to_compare(0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Ajusta el retraso según sea necesario
    }
}
