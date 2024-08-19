/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief Programación para la identificación del sistema FLYWHEEL INVERTED PENDULUM
 * @version 0.1
 * @date 2024-08-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief Programación para la identificación del sistema FLYWHEEL INVERTED PENDULUM
 * @version 0.1
 * @date 2024-08-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "encoder.h"
#include "pwm_control.h"
#include "esp_timer.h"
#include "math.h"
// Parámetros de PRBS
#define PRBS_LENGTH 15
#define PRBS_PERIOD_MS 1000
#define PWM_HIGH 1000
#define PWM_LOW 0

// Variables
int pwm_value = 0;
int prbs_sequence[PRBS_LENGTH] = {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0}; // Ejemplo de secuencia PRBS
int prbs_index = 0;

// Function Prototypes
void vMotorTask(void *pvParameters);
void vEncoderTask(void *pvParameters);

void app_main(void){
    encoder_init();
    ledc_init();
    xTaskCreatePinnedToCore(vMotorTask, 
                            "Motor Task", 
                            configMINIMAL_STACK_SIZE*3, 
                            NULL, 
                            tskIDLE_PRIORITY+1, 
                            NULL, 
                            0);
    xTaskCreatePinnedToCore(vEncoderTask,
                            "Encoder Task",
                            configMINIMAL_STACK_SIZE*3,
                            NULL,
                            tskIDLE_PRIORITY+1,
                            NULL,
                            1);
}

void vMotorTask(void *pvParameters){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (true){
        motor_forward();
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 20); // valor entre 75 Hz y 40 kHz, sacado de chat
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        motor_backward();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        motor_stop();
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 20);
        vTaskDelayUntil(&xLastWakeTime, 7000 / portTICK_PERIOD_MS);
    }
}

void vEncoderTask(void *pvParameters){
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    int encoder_value = 0;
    xLastWakeTime = xTaskGetTickCount();
    while (true){
        encoder_value = read_as5600_position();

        float time_us = esp_timer_get_time();
        float time_s = time_us / 1000000;

        // Enviar datos en formato CSV por serial
        printf("%f,%d,%d\n", time_s, encoder_value, pwm_value);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}