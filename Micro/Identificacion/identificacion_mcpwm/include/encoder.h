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
#ifndef ENCODER_H
#define ENCODER_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO          GPIO_NUM_39               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          GPIO_NUM_40               /*!< gpio number for I2C master data  */

void encoder_init(void);
int read_as5600_position(void);

#endif /* ENCODER_H */