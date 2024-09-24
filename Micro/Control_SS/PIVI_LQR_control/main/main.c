/**
 * @file main.c
 * @authors Alejo Alesandria (alealesandria@gmail.com)
 * @brief Código principal del proyecto. Control mediante realimentación de estados Péndulo Invertido con Volante de Inercia
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

#define PRINT_TASK_PERIOD_MS 10
#define SET_POINT_VALUE     0     // Pendulum angular position in degrees
#define SAMPLE_TIME_US      10000   // Sample time in microseconds (us)

// Constants for PWM mapping
#define FROM_LOW            0       // Initial PWM range minimum value (0 %)
#define FROM_HIGH           4095    // Initial PWM range maximum value (100 %)
#define TO_LOW              1000    // Final PWM range minimum value (60 %)
#define TO_HIGH             4095    // Final PWM range maximum value (100 %)

// LQR controller gains
const float K_new[2] = {32.2661, 36.6317};//{842.2070, 256.0781};//{3636.6, 2235.8};//{842.2070, 256.0781}; // ANDAN {32.2661, 36.6317}
const float ki = -0.9976;//0.0;//-9.7885;//-26.2541;

// Kalman filter parameters
float q11 = 10;
float q22 = 10;

float R_kalman = 0.1;

// Variables
int pwm_output_bits = 0;           // Last PID output value to apply to the motor
long pwm_output_bits_mapped = 0;    // PWM value mapped to the motor
int setpoint_angle = SET_POINT_VALUE;       // Setpoint angle in degrees
int error = 0;                              // Error value
int accumulated_error = 0;                  // Accumulated error value

// State Space matrix
float a11 = 0.9998;
float a12 = 0.009989;
float a21 = -0.03509;
float a22 = 0.9978;

float b11 = 0.00001584;
float b21 = 0.0001143;

float c11 = 1.0;
float c12 = 0.0;

// Control signal
float u_signal = 0;     // Control signal

int angle_value_degree = 0;

// Output signal of the system
int y_value_degree = 0;

// Kalman filter variables
float x1_hat = 0.0, x2_hat = 0.0; // Estimated states
float P_k11 = 1.0, P_k12 = 0.0, P_k21 = 0.0, P_k22 = 1.0; // Covariance matrix
float P_k_pred11, P_k_pred12, P_k_pred21, P_k_pred22; // Predicted covariance matrix
float x1_pred, x2_pred; // Predicted states
float S = 0.0;
float K11 = 0.0, K21 = 0.0; // Gain matrix
float y_hat = 0.0, y_error = 0.0; // Estimated output and error

// Handles
esp_timer_handle_t timer_handle;
TaskHandle_t xTaskPrint_handle = NULL;
portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes
void timer_callback(void* arg);     // Timer callback function - periodic task Ts = 0.01 s
void vTaskPrint();

void app_main(void){
    encoder_init();
    ledc_init();

    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "State Space Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);

    // Tasks creation
    xTaskCreate(&vTaskPrint,
                "vTaskPrint",
                configMINIMAL_STACK_SIZE * 5,
                NULL,
                1,
                &xTaskPrint_handle);
    //
}

void timer_callback(void* arg){  
    // Read encoder value
    angle_value_degree = read_as5600_position();
    y_value_degree = angle_value_degree - 150;

    // Error and control signal calculation
    error = setpoint_angle - y_value_degree;

    u_signal = - ki * accumulated_error - (K_new[0] * x1_hat + K_new[1] * x2_hat);
    
    if (u_signal > 4095){
        u_signal = 4095;
    } else if (u_signal < -4095){
        u_signal = -4095;
    }

    // Kalman filter
    // State prediction
    x1_pred = a11 * x1_hat + a12 * x2_hat + b11 * u_signal;
    x2_pred = a21 * x1_hat + a22 * x2_hat + b21 * u_signal;

    // Covariance prediction
    P_k_pred11 = (a11 * P_k11 + a12 * P_k21) * a11 + (a11 * P_k12 + a12 * P_k22) * a12 + q11;
    P_k_pred12 = (a11 * P_k11 + a12 * P_k21) * a21 + (a11 * P_k12 + a12 * P_k22) * a22;
    P_k_pred21 = (a21 * P_k11 + a22 * P_k21) * a11 + (a21 * P_k12 + a22 * P_k22) * a12;
    P_k_pred22 = (a21 * P_k11 + a22 * P_k21) * a21 + (a21 * P_k12 + a22 * P_k22) * a22 + q22;

    S = c11 * (c11 * P_k_pred11 + c12 * P_k_pred21) + c12 * (c11 * P_k_pred12 + c12 * P_k_pred22) + R_kalman;

    // Kalman gain
    K11 = (P_k_pred11 * c11 + P_k_pred12 * c12) / S;
    K21 = (P_k_pred21 * c11 + P_k_pred22 * c12) / S;

    // Update states
    y_hat = c11 * x1_pred + c12 * x2_pred;  
    y_error = y_value_degree - y_hat;

    x1_hat = x1_pred + K11 * y_error;
    x2_hat = x2_pred + K21 * y_error;

    // Update covariance matrix
    P_k11 = (1 - K11 * c11) * P_k_pred11 - K11 * c12 * P_k_pred12;
    P_k12 = (1 - K11 * c11) * P_k_pred12 - K11 * c12 * P_k_pred22;
    P_k21 = (1 - K21 * c12) * P_k_pred21 - K21 * c11 * P_k_pred11;
    P_k22 = (1 - K21 * c12) * P_k_pred22 - K21 * c11 * P_k_pred12;   
    
    if(u_signal < 0){
        motor_clockwise();
    } else if (u_signal > 0){
        motor_counterclockwise();
    } else {
        motor_stop();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs((int) u_signal), 0);
    } 
    pwm_output_bits = abs((int) u_signal);
    
    // Set PWM duty cycle
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
    
    // Anti-windup
    if (u_signal == -4095) {
        if(error > 0) {
            accumulated_error += error;
        }
    } else if (u_signal == 4095){
        if (error < 0) {
            accumulated_error += error;
        }
    } else {
        accumulated_error += error;
    }

    // Limit accumulated error
    if (accumulated_error > 1000){
        accumulated_error = 1000;
    } else if (accumulated_error < -1000){
        accumulated_error = -1000;
    }
}

void vTaskPrint(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float fb_value_v_l = 0.0;
    float error_l = 0.0;
    float accumulated_error_l = 0.0;
    float x1_hat_l = 0.0;
    float x2_hat_l = 0.0;
    int pwm_output_bits_l = 0;

    while(true){
        taskENTER_CRITICAL(&_spinlock);
        fb_value_v_l = y_value_degree;
        error_l = error;
        accumulated_error_l = accumulated_error;
        x1_hat_l = x1_hat;
        x2_hat_l = x2_hat;
        pwm_output_bits_l = pwm_output_bits;
        taskEXIT_CRITICAL(&_spinlock);

        printf("Feedback: %.2f Grados \n", fb_value_v_l);
        printf("Error: %.2f Grados \n", error_l);
        printf("q: %.2f \n", accumulated_error_l);
        printf("x1: %.2f \n",x1_hat_l);
        printf("x2: %.2f \n",x2_hat_l);
        printf("PWM: %d \n", pwm_output_bits_l);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PRINT_TASK_PERIOD_MS));
    }
}