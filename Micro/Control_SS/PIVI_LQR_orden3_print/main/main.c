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
#define SAMPLE_INDEX        3000

#define SET_POINT_VALUE     0     // Pendulum angular position in degrees
#define SAMPLE_TIME_US      10000   // Sample time in microseconds (us)

// Constants for PWM mapping
#define FROM_LOW            0       // Initial PWM range minimum value (0 %)
#define FROM_HIGH           4095    // Initial PWM range maximum value (100 %)
#define TO_LOW              0    // Final PWM range minimum value (60 %)
#define TO_HIGH             4095    // Final PWM range maximum value (100 %)

// LQR controller gains
const float K_new[3] = {-805.7053, -66.7527, -0.2154};//{842.2070, 256.0781};//{3636.6, 2235.8};//{842.2070, 256.0781}; // ANDAN {32.2661, 36.6317}
const float ki = 8.1492;//0.0;//-9.7885;//-26.2541;
const float Ts = SAMPLE_TIME_US / 1000000.0;

// Kalman filter parameters

float q11 = 100;
float q22 = 100;
float q33 = 100;

float R_kalman = 0.00001;

// Variables
int pwm_output_bits = 0;           // Last PID output value to apply to the motor
long pwm_output_bits_mapped = 0;    // PWM value mapped to the motor
int setpoint_angle = SET_POINT_VALUE;       // Setpoint angle in degrees
int error = 0;                              // Error value
int accumulated_error = 0;                  // Accumulated error value

// State Space matrix
float a11 = 0.9999; //anda
float a12 = 0.009991;
float a13 = 0.00002296;
float a21 = -0.02553;
float a22 = 0.9976;
float a23 = 0.0032;
float a31 = -3.559;
float a32 = -0.3435;
float a33 = 0.05097;

float b11 = 0.0002288;
float b21 = -0.1154;
float b31 = 33.75;

float c11 = 1.0;
float c12 = 0.0;
float c13 = 0.0;

// Control signal
float u_signal = 0;

int angle_value_degree = 0;

// Output signal of the system
int y_value_degree = 0;

// Kalman filter variables
float q1, q2, q3; // Covarianzas
float x1_hat = 0.0, x2_hat = 0.0, x3_hat = 0.0; // Estados estimados
float P_k11 = 1.0, P_k12 = 0.0, P_k13, P_k21 = 0.0, P_k22 = 1.0, P_k23, P_k31, P_k32, P_k33; // Matriz de covarianza
float P_k_pred11 = 1.0, P_k_pred12 = 0.0, P_k_pred13, P_k_pred21 = 0.0, P_k_pred22 = 1.0, P_k_pred23, P_k_pred31, P_k_pred32, P_k_pred33;; // Matriz de covarianza predicha
float x1_pred, x2_pred, x3_pred; // Estados predichos
float S = 0.0;
float K11 = 0.0, K21 = 0.0, K31 = 0.0; // Coeficientes de corrección
float y_hat = 0.0, y_error = 0.0; // Variables de error

// Print variables
int index_value = 0;                // Index for output array
bool is_clockwise = false;          // Motor direction flag for plotting
int y_signal_print[SAMPLE_INDEX];     // Output values for plotting
int u_signal_print[SAMPLE_INDEX];       // output values for plotting
float x1_pred_print[SAMPLE_INDEX];
float x2_pred_print[SAMPLE_INDEX];
float x3_pred_print[SAMPLE_INDEX];

// Handles
esp_timer_handle_t timer_handle;
TaskHandle_t xTaskPrint_handle = NULL;
portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes
void timer_callback(void* arg);     // Timer callback function - periodic task Ts = 0.01 s
long map(long x, long in_min, long in_max, long out_min, long out_max); 
void print_values();

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
    vTaskDelay(pdMS_TO_TICKS(5000));
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
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        esp_timer_stop(timer_handle);
        return;
    }
    // Read encoder value
    angle_value_degree = read_as5600_position();
    y_value_degree = angle_value_degree - 150;

    // Error and control signal calculation
    error = setpoint_angle - y_value_degree;

    x1_pred = a11*x1_hat + a12*x2_hat + a13*x3_hat + b11*u_signal;
    x2_pred = a21*x1_hat + a22*x2_hat + a23*x3_hat + b21*u_signal;
    x3_pred = a31*x1_hat + a32*x2_hat + a33*x3_hat + b31*u_signal;

    u_signal = - ki * accumulated_error - (K_new[0] * x1_hat + K_new[1] * x2_hat + K_new[2] * x3_hat);
    
    if (u_signal > 4095){
        u_signal = 4095;
    } else if (u_signal < -4095){
        u_signal = -4095;
    }

    // Kalman filter
    // State prediction

    P_k_pred11 = (a11*P_k11 + a12*P_k21 + a13*P_k31)*a11 + (a11*P_k12 + a12*P_k22 + a13*P_k32)*a12 + (a11*P_k13 + a12*P_k23 + a13*P_k33)*a13 + q11;
    P_k_pred12 = (a11*P_k11 + a12*P_k21 + a13*P_k31)*a21 + (a11*P_k12 + a12*P_k22 + a13*P_k32)*a22 + (a11*P_k13 + a12*P_k23 + a13*P_k33)*a23;
    P_k_pred13 = (a11*P_k11 + a12*P_k21 + a13*P_k31)*a31 + (a11*P_k12 + a12*P_k22 + a13*P_k32)*a32 + (a11*P_k13 + a12*P_k23 + a13*P_k33)*a33;

    P_k_pred21 = (a21*P_k11 + a22*P_k21 + a23*P_k31)*a11 + (a21*P_k12 + a22*P_k22 + a23*P_k32)*a12 + (a21*P_k13 + a22*P_k23 + a23*P_k33)*a13;
    P_k_pred22 = (a21*P_k11 + a22*P_k21 + a23*P_k31)*a21 + (a21*P_k12 + a22*P_k22 + a23*P_k32)*a22 + (a21*P_k13 + a22*P_k23 + a23*P_k33)*a23 + q22;
    P_k_pred23 = (a21*P_k11 + a22*P_k21 + a23*P_k31)*a31 + (a21*P_k12 + a22*P_k22 + a23*P_k32)*a32 + (a21*P_k13 + a22*P_k23 + a23*P_k33)*a33;

    P_k_pred31 = (a31*P_k11 + a32*P_k21 + a33*P_k31)*a11 + (a31*P_k12 + a32*P_k22 + a33*P_k32)*a12 + (a31*P_k13 + a32*P_k23 + a33*P_k33)*a13;
    P_k_pred32 = (a31*P_k11 + a32*P_k21 + a33*P_k31)*a21 + (a31*P_k12 + a32*P_k22 + a33*P_k32)*a22 + (a31*P_k13 + a32*P_k23 + a33*P_k33)*a23;
    P_k_pred33 = (a31*P_k11 + a32*P_k21 + a33*P_k31)*a31 + (a31*P_k12 + a32*P_k22 + a33*P_k32)*a32 + (a31*P_k13 + a32*P_k23 + a33*P_k33)*a33 + q33;

    S = (c11*P_k_pred11 + c12*P_k_pred21 + c13*P_k_pred31)*c11 + (c11*P_k_pred12 + c12*P_k_pred22 + c13*P_k_pred32)*c12 + (c11*P_k_pred13 + c12*P_k_pred23 + c13*P_k_pred33)*c13 + R_kalman;
    
    // Kalman gain
    K11 = (P_k_pred11*c11 + P_k_pred12*c12 + P_k_pred13*c13)/S;
    K21 = (P_k_pred21*c11 + P_k_pred22*c12 + P_k_pred23*c13)/S;
    K31 = (P_k_pred31*c11 + P_k_pred32*c12 + P_k_pred33*c13)/S;

    // Update states
    y_hat = c11*x1_pred + c12*x2_pred + c13*x3_pred;

    y_error = y_value_degree - y_hat;

    x1_hat = x1_pred + K11*y_error;
    x2_hat = x2_pred + K21*y_error;
    x3_hat = x3_pred + K31*y_error;
    // Update covariance matrix
    P_k11 = (1 - K11*c11)*P_k_pred11 - K11*c12*P_k_pred21 - K11*c13*P_k_pred31;
    P_k12 = (1 - K11*c11)*P_k_pred12 - K11*c12*P_k_pred22 - K11*c13*P_k_pred32;
    P_k13 = (1 - K11*c11)*P_k_pred13 - K11*c12*P_k_pred23 - K11*c13*P_k_pred33;

    P_k21 = -K21*c11*P_k_pred11 + (1 - K21*c12)*P_k_pred21 - K21*c13*P_k_pred31;
    P_k22 = -K21*c11*P_k_pred12 + (1 - K21*c12)*P_k_pred22 - K21*c13*P_k_pred32;
    P_k23 = -K21*c11*P_k_pred13 + (1 - K21*c12)*P_k_pred23 - K21*c13*P_k_pred33;

    P_k31 = -K31*c11*P_k_pred11 - K31*c12*P_k_pred21 + (1 - K31*c13)*P_k_pred31;
    P_k32 = -K31*c11*P_k_pred12 - K31*c12*P_k_pred22 + (1 - K31*c13)*P_k_pred32;
    P_k33 = -K31*c11*P_k_pred13 - K31*c12*P_k_pred23 + (1 - K31*c13)*P_k_pred33;

    if(u_signal > 0){
        motor_clockwise();
        is_clockwise = true;
    } else if (u_signal < 0){
        motor_counterclockwise();
        is_clockwise = false;
    } else {
        motor_stop();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs((int) u_signal), 0);
        is_clockwise = true;
    } 
    pwm_output_bits = map(abs((long) u_signal), FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH);
    
    // Set PWM duty cycle
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
    
    if(!is_clockwise){
        u_signal_print[index_value] = pwm_output_bits;
    } else{
        u_signal_print[index_value] = -pwm_output_bits;
        is_clockwise = true;
    }
    y_signal_print[index_value] = y_value_degree;
    x1_pred_print[index_value] = x1_hat;
    x2_pred_print[index_value] = x2_hat;
    x3_pred_print[index_value] = x3_hat;

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

    index_value++;
}

// Function to print output values for plotting
void print_values(){
    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++) {
        printf("%.2f,%d,%d,%.2f,%.2f,%.2f\n", 
            i * Ts, u_signal_print[i], y_signal_print[i], x1_pred_print[i], x2_pred_print[i], x3_pred_print[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    printf("FIN\n");
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}