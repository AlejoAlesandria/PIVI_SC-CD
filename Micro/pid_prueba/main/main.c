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
#define SET_POINT_VALUE 143
#define SAMPLE_TIME_US 10000

// Variables
int pwm_value = 0;
int prbs_index = 0;
int angle = 0;
int pwm_output_bits = 0;


// PID constants and variables
const float Kp = -4.82; // 1
const float Ki = -0.005049; // 0.075612
const float Kd = 160.1248; // 0.0000001
const float Nc = 33.2155;
const float Ts = SAMPLE_TIME_US/1000000.0;
const int Ts_ms = Ts * 1000;
int setpoint_angle = SET_POINT_VALUE;

// handles
esp_timer_handle_t timer_handle;

// functions
void timer_callback(void* arg);

// coeficientes viejos
/*const float a_coefficients[3] = {0, 0, -1};
const float b_coefficients[3] = {
    Kp + (Ki * Ts / 2) + (2 * Kd / Ts),
    Ki * Ts - (4 * Kd / Ts),
    -Kp + (Ki * Ts / 2) + (2 * Kd / Ts)
};*/
// coeficientes nuevos
/*const float a_coefficients[3] = {0, 0, -1};
const float b_coefficients[3] = {
    Kp + Ki*Ts + Kd*Ts,
    -Kp - Kd*2/Ts,
    Kd/Ts
};*/
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

void app_main(void){
    encoder_init();
    //mcpwm_init();
    ledc_init();    
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);
}

void timer_callback(void* arg){
        setpoint_angle = SET_POINT_VALUE;
        input_array[0] = setpoint_angle - read_as5600_position();
        output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];       
        if(output_array[0] < 0){
            motor_backward();
        } else if (output_array[0] > 0){
            motor_forward();
        }else{
        //if(output_array[0] == 0){
            motor_stop();
        }
        pwm_output_bits = abs((int)output_array[0]);
        /*if (pwm_output_bits > 1000){
            pwm_output_bits = 1000;
        }
        if (pwm_output_bits < 0){
            pwm_output_bits = 0;
        }*/
        if (pwm_output_bits > 4095){
            pwm_output_bits = 4095;
        }
        if (pwm_output_bits < 0){
            pwm_output_bits = 0;
        }
        //printf("%d\n", pwm_output_bits);
        //mcpwm_set_value_to_compare(pwm_output_bits);
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
}