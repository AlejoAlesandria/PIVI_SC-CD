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

// Defines
#define INDEX               3000
#define SAMPLE_TIME_US      10000
#define DUTY_CYCLE          4095
#define FROM_LOW            0
#define FROM_HIGH           4095
#define TO_LOW              2457
#define TO_HIGH             4095
// duty cycle desde 0 a 4095
// Variables

int encoder_value[INDEX] = {0};
int prbs_index = 0;
int new_frequency = 0;
int map_duty = 0;
// handles
esp_timer_handle_t timer_handle;

// functions
void timer_callback(void* arg);
void print_values();
long map();

void app_main(void){
    encoder_init();
    ledc_init();

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, DUTY_CYCLE, 0);    
    
    // Configure Timer
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);

    vTaskDelay(pdMS_TO_TICKS(5000));
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US); 

    while(true){
        if(!esp_timer_is_active(timer_handle)){
            print_values();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void timer_callback(void* arg){
    if(prbs_index == INDEX){
        esp_timer_stop(timer_handle);
        motor_stop();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
        return;
    }

    // Impulso
    /*motor_stop();
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
    if (prbs_index > 50 && prbs_index < 60){
        motor_counterclockwise();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(DUTY_CYCLE, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
    }*/

    // Escalón
    if (prbs_index > 50){
        motor_counterclockwise();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(DUTY_CYCLE, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
    }
    

    encoder_value[prbs_index] = read_as5600_position();
    prbs_index++;
}

void print_values(){
    printf("INICIO\n");
    for(int i = 0; i < INDEX; i++){
        printf("%d\n", encoder_value[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}