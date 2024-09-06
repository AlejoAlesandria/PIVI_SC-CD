/**
 * @file encoder.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "time.h"

// Pines utilizados para l298n
#define PIN_IN1 GPIO_NUM_6 
#define PIN_IN2 GPIO_NUM_5
#define PIN_ENA GPIO_NUM_7

void ledc_init(void);

void motor_counterclockwise(void);
void motor_clockwise(void);
void motor_stop(void);

// No se que otra función debe ser pública, seguramente la que se utilice para comunicar el ángulo con el duty del pwm.
#endif /* PWM_CONTROL_H */