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
//#include "pwm_control.h"
#include "pwm_control_ledc.h"
#include "esp_timer.h"
#include "math.h"
// Parámetros de PRBS
#define PRBS_LENGTH 15
#define PRBS_PERIOD_MS 1000
#define PWM_HIGH 1000
#define PWM_LOW 0
#define SET_POINT_VALUE 333
#define SAMPLE_TIME_US 10000
#define INDEX               4200

#define FROM_LOW            0
#define FROM_HIGH           4095
#define TO_LOW              2457
#define TO_HIGH             4095

// Variables
int pwm_value = 0;
int prbs_index = 0;
int angle = 0;
long pwm_output_bits = 0;
int output[INDEX];


// PID constants and variables
const float Kp = -3982.54;//-4.4597; // -4.1051
const float Ki = -0.00274315;//-0.042412; // -0.0091758
const float Kd = 0.00001;//982.2039;//104.0459;//274.7715; // 133.4577
const float Nc = 0.1667184;//61.6205; // 32.5156
const float Ts = SAMPLE_TIME_US/1000000.0;
const int Ts_ms = Ts * 1000;
int setpoint_angle = SET_POINT_VALUE;

// handles
esp_timer_handle_t timer_handle;

// functions
void timer_callback(void* arg);
void print_values();

const float a_coefficients[3] = {
    1,
    -2 + Nc * Ts,
    1 - Nc * Ts
};
const float b_coefficients[3] = {
    Kp + Kd * Nc,
    -2 * Kp - 2 * Kd * Nc + Ki * Ts + Kp * Nc * Ts,
    Kp + Kd * Nc - Ki * Ts - Kp * Nc * Ts + Ki * Nc * Ts * Ts
};
float input_array[3] = {0, 0, 0};
float output_array[3] = {0, 0, 0};

// Function Prototypes
void vMotorTask(void *pvParameters);
void vEncoderTask(void *pvParameters);
long map();

void app_main(void){
    encoder_init();
    ledc_init();    
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 3276);
    esp_timer_create(&timer_args, &timer_handle);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);

    while (true){
        if(!esp_timer_is_active(timer_handle)){
            printf("Setpoint sequence completed, printing output values:\n");
            print_values();
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

void timer_callback(void* arg){
    if(prbs_index == INDEX){
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
        esp_timer_stop(timer_handle);
        return;
    }

    setpoint_angle = SET_POINT_VALUE;
    output[prbs_index] = read_as5600_position();

    input_array[0] = setpoint_angle - read_as5600_position();
    output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];       
    if(output_array[0] > 0){
        motor_clockwise();
    } else if (output_array[0] < 0){
        motor_counterclockwise();
    }else{
    //if(output_array[0] == 0){
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH), 0);
    }
    pwm_output_bits = abs((int)output_array[0]);
    //pwm_output_bits += 2047; 
    
    if (pwm_output_bits > 4095){
        pwm_output_bits = 4095;
    }
    if (pwm_output_bits < 0){
        pwm_output_bits = 0;
    }
    pwm_output_bits = map(pwm_output_bits, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits);
    //ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, pwm_output_bits);
    input_array[2] = input_array[1];
    input_array[1] = input_array[0];
    output_array[2] = output_array[1];
    output_array[1] = output_array[0];

    /*  angle = read_as5600_position();
    printf("%d\n", angle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //motor_backward();*/
    //prbs_index++;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void print_values(){
    printf("INICIO\n");
    for(int i = 0; i < INDEX; i++){
        printf("%d\n", output[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");
}