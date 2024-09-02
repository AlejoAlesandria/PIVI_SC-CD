/**
 * @file main.c
 * @authors Alejo Alesandria (alealesandria@gmail.com), Ramiro Delfino (pokramiro12@gmail.com)
 * @brief Código principal del proyecto. PID con filtro derivativo para el control de Péndulo Invertido con Volante de Inercia
 * @version 1.0
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

#define SET_POINT_VALUE     330     // Pendulum angular position in degrees
#define SAMPLE_TIME_US      10000   // Sample time in microseconds (us)

#define SAMPLE_INDEX        2200    // Samples to be taken

// Constants for PWM mapping
#define FROM_LOW            0       // Initial PWM range minimum value (0 %)
#define FROM_HIGH           4095    // Initial PWM range maximum value (100 %)
#define TO_LOW              2457    // Final PWM range minimum value (60 %)
#define TO_HIGH             4095    // Final PWM range maximum value (100 %)

// Variables
int index_value = 0;                // Index for output array
long pwm_output_bits = 0;           // Last PID output value to apply to the motor
long pwm_output_bits_mapped = 0;    // PWM value mapped to the motor
int output[SAMPLE_INDEX];           // Output values for plotting
int output_pwm[SAMPLE_INDEX];           // Output values for plotting
bool is_clockwise = false;

// PID constants and variables
int setpoint_angle = SET_POINT_VALUE;       // Setpoint angle in degrees
const float Kp = -34.268019;                   // Porportional constant
const float Ki = 0;                // Integral constant
const float Kd = -0.001238441;                  // Derivative constant
const float N = 18.19876;                    // Derivative filter constant
const float Ts = SAMPLE_TIME_US/1000000.0;  // Sample time in seconds
const int Ts_ms = Ts * 1000;                // Sample time in milliseconds
float input_array[3] = {0, 0, 0};           // Input array for PID
float output_array[3] = {0, 0, 0};          // Output array for PID

const float a_coefficients[3] = {           // Output coefficients for the PID
    1,
    -2 + N * Ts,
    1 - N * Ts
};
const float b_coefficients[3] = {           // Input coefficients for the PID
    Kp + Kd * N,
    -2 * Kp - 2 * Kd * N + Ki * Ts + Kp * N * Ts,
    Kp + Kd * N - Ki * Ts - Kp * N * Ts + Ki * N * Ts * Ts
};

// Handles
esp_timer_handle_t timer_handle;

// Function prototypes
void timer_callback(void* arg);                                             // Timer callback function - periodic task Ts = 0.01 s
long map(long x, long in_min, long in_max, long out_min, long out_max);     // Map function for PWM values
void print_values();                                                        // Print output values for plotting

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
    if(index_value == SAMPLE_INDEX){
        motor_stop();
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH));
        esp_timer_stop(timer_handle);
        return;
    }
    pwm_output_bits_mapped = 0;
    setpoint_angle = SET_POINT_VALUE;
    output[index_value] = read_as5600_position(); // Save data for plotting

    input_array[0] = setpoint_angle - read_as5600_position();

    output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];       

    if(output_array[0] > 0){
        motor_clockwise();
        is_clockwise = true;
    } else if (output_array[0] < 0){
        motor_counterclockwise();
        is_clockwise = false;
    } else{
        motor_stop();
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH));
        is_clockwise = true;
    }

    if (output_array[0] < -4095){
        output_array[0] = -4095;
    } else if (output_array[0] > 4095){
        output_array[0] = 4095;
    }

    pwm_output_bits = abs((int)output_array[0]);
    
    pwm_output_bits_mapped = map(pwm_output_bits, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH);
    if(!is_clockwise){
        output_pwm[index_value] = -pwm_output_bits_mapped;
    } else{
        output_pwm[index_value] = pwm_output_bits_mapped;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits_mapped);

    input_array[2] = input_array[1];
    input_array[1] = input_array[0];
    output_array[2] = output_array[1];
    output_array[1] = output_array[0];

    // Comment index++ for continuous operation
    //index_value++; 
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to print output values for plotting
void print_values(){
    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%d\n", output[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");
    
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%d\n", output_pwm[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");
}