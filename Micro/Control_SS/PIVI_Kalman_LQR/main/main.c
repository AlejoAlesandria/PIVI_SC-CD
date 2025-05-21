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

#define SET_POINT_VALUE     0     // Pendulum angular position in degrees
#define SAMPLE_TIME_US      10000   // Sample time in microseconds (us)

#define SAMPLE_INDEX        2200    // Samples to be taken

// Constants for PWM mapping
#define FROM_LOW            0       // Initial PWM range minimum value (0 %)
#define FROM_HIGH           4095    // Initial PWM range maximum value (100 %)
#define TO_LOW              2457    // Final PWM range minimum value (60 %)
#define TO_HIGH             4095    // Final PWM range maximum value (100 %)

// Variables
int index_value = 0;                // Index for output array
int pwm_output_bits = 0;           // Last PID output value to apply to the motor
long pwm_output_bits_mapped = 0;    // PWM value mapped to the motor
bool is_clockwise = false;          // Motor direction flag for plotting
int output_angle[SAMPLE_INDEX];     // Output values for plotting
int output_pwm[SAMPLE_INDEX];       // output values for plotting
float x1_print[SAMPLE_INDEX];
float x2_print[SAMPLE_INDEX];
float x1_pred_print[SAMPLE_INDEX];
float x2_pred_print[SAMPLE_INDEX];
float error_print[SAMPLE_INDEX];

// State Space constants and variables
int setpoint_angle = SET_POINT_VALUE;       // Setpoint angle in degrees
int error = 0;                              // Error value
int acumulated_error = 0;                   // Previous error value

float a11 = 0.9998;
float a12 = 0.009989;
float a21 = -0.03509;
float a22 = 0.9978;

float b11 = 0.00001584;
float b21 = 0.0001143;

float c11 = 1.0;
float c12 = 0.0;

// Pole placement controller gains
const float K_new[2] = {32.2661, 36.6317};//{842.2070, 256.0781};//{3636.6, 2235.8};//{842.2070, 256.0781}; // ANDAN {32.2661, 36.6317}
const float ki = -0.9976;//0.0;//-9.7885;//-26.2541;

// Control signal
float u_signal = 0;                         // Control signal

int angle_value_degree = 0;

// Output signal of the system
int y_value_degree = 0;

// Supposedly Kalman
float q1, q2; // Covarianzas
float x1_hat = 0.0, x2_hat = 0.0; // Estados estimados
float P_k11 = 1.0, P_k12 = 0.0, P_k21 = 0.0, P_k22 = 1.0; // Matriz de covarianza
float P_k_pred11, P_k_pred12, P_k_pred21, P_k_pred22; // Matriz de covarianza predicha
float x1_pred, x2_pred; // Estados predichos
float S = 0.0;
float K11 = 0.0, K21 = 0.0; // Coeficientes de corrección
float y_hat = 0.0, y_error = 0.0; // Variables de error

float q11 = 10;
float q22 = 10;

float R_kalman = 0.0001;
//

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

    //vTaskDelay(5000 / portTICK_PERIOD_MS);

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
    output_angle[index_value] = y_value_degree;

    error = setpoint_angle - y_value_degree;
    error_print[index_value] = error;

    printf("error = %d\n", error);
    // State space model
    u_signal = - ki * acumulated_error -(K_new[0] * x1_hat + K_new[1] * x2_hat);
    
    if (u_signal > 4095){
        u_signal = 4095;
    } else if (u_signal < -4095){
        u_signal = -4095;
    }

    // Kalman filter
    // Prediccion estados
    x1_pred = a11 * x1_hat + a12 * x2_hat + b11 * u_signal;
    x2_pred = a21 * x1_hat + a22 * x2_hat + b21 * u_signal;
    x1_pred_print[index_value] = x1_pred;
    x2_pred_print[index_value] = x2_pred;
    P_k_pred11 = (a11 * P_k11 + a12 * P_k21) * a11 + (a11 * P_k12 + a12 * P_k22) * a12 + q11;
    P_k_pred12 = (a11 * P_k11 + a12 * P_k21) * a21 + (a11 * P_k12 + a12 * P_k22) * a22;
    P_k_pred21 = (a21 * P_k11 + a22 * P_k21) * a11 + (a21 * P_k12 + a22 * P_k22) * a12;
    P_k_pred22 = (a21 * P_k11 + a22 * P_k21) * a21 + (a21 * P_k12 + a22 * P_k22) * a22 + q22;

    // Cálculo del término S para la ganancia de Kalman
    float S = c11 * (c11 * P_k_pred11 + c12 * P_k_pred21) + c12 * (c11 * P_k_pred12 + c12 * P_k_pred22) + R_kalman;

    float K11 = (P_k_pred11 * c11 + P_k_pred12 * c12) / S;
    float K21 = (P_k_pred21 * c11 + P_k_pred22 * c12) / S;

    float y_hat = c11 * x1_pred + c12 * x2_pred;  
    float y_error = y_value_degree - y_hat;

    x1_hat = x1_pred + K11 * y_error;
    x2_hat = x2_pred + K21 * y_error;
    x1_print[index_value] = x1_hat;
    x2_print[index_value] = x2_hat;

    printf("x1_hat = %.2f\n", x1_hat);
    printf("x2_hat = %.2f\n", x2_hat);


    // 5. Actualización de la covarianza del error
    P_k11 = (1 - K11 * c11) * P_k_pred11 - K11 * c12 * P_k_pred12;
    P_k12 = (1 - K11 * c11) * P_k_pred12 - K11 * c12 * P_k_pred22;
    P_k21 = (1 - K21 * c12) * P_k_pred21 - K21 * c11 * P_k_pred11;
    P_k22 = (1 - K21 * c12) * P_k_pred22 - K21 * c11 * P_k_pred12;   
    
    if(u_signal < 0){
        motor_clockwise();
        is_clockwise = true;
    } else if (u_signal > 0){
        motor_counterclockwise();
        is_clockwise = false;
    } else{
        motor_stop();
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        is_clockwise = true;
    } 
    pwm_output_bits = abs((int) u_signal);
    printf("u = %.2f\n", u_signal);
    //pwm_output_bits_mapped = map(pwm_output_bits, FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH);
    
    if(!is_clockwise){
        output_pwm[index_value] = pwm_output_bits;
    } else{
        output_pwm[index_value] = -pwm_output_bits;
    }

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
    //printf("PWM: %ld\n", pwm_output_bits_mapped);
    
    if (u_signal == -4095) {
        if(error < 0) {
            acumulated_error += error;
        }
    } else if (u_signal == 4095){
        if (error > 0) {
            acumulated_error += error;
        }
    } else {
        acumulated_error += error;
    }

    if (acumulated_error > 1000){
        acumulated_error = 1000;
    } else if (acumulated_error < -1000){
        acumulated_error = -1000;
    }

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

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%.2f\n", x1_print[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%.2f\n", x2_print[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%.2f\n", x1_pred_print[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%.2f\n", x2_pred_print[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("INICIO\n");
    for(int i = 0; i < SAMPLE_INDEX; i++){
        printf("%.2f\n", error_print[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("FIN\n");
}