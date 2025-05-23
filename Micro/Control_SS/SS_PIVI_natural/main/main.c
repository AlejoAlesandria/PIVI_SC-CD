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

#define SET_POINT_VALUE     150     // Pendulum angular position in degrees
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
bool is_clockwise = false;          // Motor direction flag for plotting
int output_angle[SAMPLE_INDEX];     // Output values for plotting
int output_pwm[SAMPLE_INDEX];       // output values for plotting

// State Space constants and variables
int setpoint_angle = SET_POINT_VALUE;       // Setpoint angle in degrees
int error = 0;                              // Error value
int acumulated_error = 0;                   // Previous error value

float a11 = 1.0; //anda
float a12 = 0.009998;
float a13 = 0.00004934;
float a21 = -0.00002264;
float a22 = 0.9994;
float a23 = 0.009801;
float a31 = -0.004497;
float a32 = -0.1258;
float a33 = 0.9603;

float b11 = -0.000003522;
float b21 = -0.0005367;
float b31 = 0.0001951;

float c11 = 1.0;
float c12 = 0.0;
float c13 = 0.0;

/*float a11 = 0.0;
float a12 = 1.0;
float a21 = -0.9983;
float a22 = 1.998;

float b11 = 0.003422;
float b21 = 0.003975;

float c11 = 1.0;
float c12 = 0.0;*/

// Luenberger observer gains
float l11 = 0.0656; //andan
float l21 = -0.2094;
float l31 = 0.0782;

/*float l11 = 0.1031;
float l21 = 0.1038;*/

// State variables
float x1_hat[2] = {0.0, 0.0};         // State vector
float x2_hat[2] = {0.0, 0.0};     // State vector
float x3_hat[2] = {0.0, 0.0};     // State vector

// Control signal
float u_signal = 0;                         // Control signal

// Pole placement controller gains
const float K_new[3] = {-260.4563, -355.1584, 162.3762};//{-265.7193, -266.8060, 183.6805};
const float ki = 0.0;//183.6805;

int angle_value_degree = 0;

// Output signal of the system
int y_value_degree = 0;

// Handles
esp_timer_handle_t timer_handle;

// Function prototypes
void timer_callback(void* arg);                                             // Timer callback function - periodic task Ts = 0.01 s
long map(long x, long in_min, long in_max, long out_min, long out_max);     // Map function for PWM values
void print_values();

void app_main(void){
    encoder_init();
    ledc_init();

    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);

    while (true) { 
        if (!esp_timer_is_active(timer_handle)) {
            printf("Setpoint sequence completed, printing output values:\n");
            print_values();
            break;
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void timer_callback(void* arg){
    if (index_value == SAMPLE_INDEX) {
        motor_stop();
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH));
        esp_timer_stop(timer_handle);
        return;
    }
    
    // Read encoder value
    angle_value_degree = read_as5600_position();
    y_value_degree = angle_value_degree;
    output_angle[index_value] = angle_value_degree;

    error = setpoint_angle - y_value_degree;
   // State space model
    x1_hat[0] = x1_hat[1];
    x2_hat[0] = x2_hat[1];
    x3_hat[0] = x3_hat[1];
    //printf("x1 = %.2f \n", x1_hat[0]);
    //printf("x2 = %.2f \n", x2_hat[0]);
    //printf("x3 = %.2f \n", x3_hat[0]);

    /*---------- 3rd order -------------*/
    x1_hat[1] = (a11-l11*c11) * x1_hat[0] + (a12-l11*c12) * x2_hat[0] + (a13-l11*c13) * x3_hat[0] + b11 * u_signal + l11 * y_value_degree;
    x2_hat[1] = (a21-l21*c11) * x1_hat[0] + (a22-l21*c12) * x2_hat[0] + (a23-l21*c13) * x3_hat[0] + b21 * u_signal + l21 * y_value_degree;
    x3_hat[1] = (a31-l31*c11) * x1_hat[0] + (a32-l31*c12) * x2_hat[0] + (a33-l31*c13) * x3_hat[0] + b31 * u_signal + l31 * y_value_degree;
    printf("x1 = %.2f \n", x1_hat[1]);
    printf("x2 = %.2f \n", x2_hat[1]);
    printf("x3 = %.2f \n", x3_hat[1]);
    u_signal = -ki * acumulated_error - (K_new[0] * x1_hat[0] + K_new[1] * x2_hat[0] + K_new[2] * x3_hat[0]);
    //u_signal = -(K_new[0] * x1_hat[0] + K_new[1] * x2_hat[0] + K_new[2] * x3_hat[0]);
    //printf("u_signal = %.2f \n", u_signal);

    /*---------- 2nd order -------------*/

    /*x1_hat[1] = (a11-l11*c11)*x1_hat[0] + (a12-l11*c12)*x2_hat[0] + b11 * u_signal + l11 * y_value_degree;
    x2_hat[1] = (a21-l21*c11)*x1_hat[0] + (a22-l21*c12)*x2_hat[0] + b21 * u_signal + l21 * y_value_degree;
    printf("x1 = %.2f \n", x1_hat[1]);
    printf("x2 = %.2f \n", x2_hat[1]);
    printf("x2 = %.2f \n", x3_hat[1]);

    u_signal = -ki * acumulated_error -(K_new[0] * x1_hat[0] + K_new[1] * x2_hat[0]);*/

    if(u_signal < 0){
        motor_clockwise();
        is_clockwise = true;
    } else if (u_signal > 0){
        motor_counterclockwise();
        is_clockwise = false;
    } else{
        motor_stop();
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, map(0, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH));
        is_clockwise = true;
    } 
    if (u_signal > 4095){
        u_signal = 4095;
    } else if (u_signal < -4095){
        u_signal = -4095;
    }
    //if (u_signal > -1.98 && u_signal < 0){
    //    u_signal = -1.98;
    //} else if (u_signal < 1.98 && u_signal > 0){
    //    u_signal = 1.98;
    //}
    printf("u_signal = %.2f \n", u_signal);
    pwm_output_bits = abs((long)u_signal);
    pwm_output_bits_mapped = map(pwm_output_bits, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH);
    
    if(!is_clockwise){
        output_pwm[index_value] = pwm_output_bits_mapped;
    } else{
        output_pwm[index_value] = -pwm_output_bits_mapped;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits_mapped);
    //printf("PWM: %ld\n", pwm_output_bits_mapped);
    acumulated_error += error;

    // Comment index++ for continuous operation
    //index_value++;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to print output values for plotting
void print_values(){
    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%d\n", output_angle[i]);
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