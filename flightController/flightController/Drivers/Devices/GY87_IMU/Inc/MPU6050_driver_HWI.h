/*
 * MIT License
 * Copyright (c) 2023 Francesco Cavina <francescocavina98@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or, sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of he Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAS PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY. WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE
 */

/*
 * @file:    MPU6050_driver_HWI.h
 * @date:    03/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:   This is a driver for the GY87 IMU module.
 *           It is divided in three parts: One high level abstraction layer
 *           (MPU6050_driver_UAI.c and MPU6050_driver_UAI.h) for interface with the user
 *           application, one low level abstraction layer (MPU6050_driver_HWI.c and
 *           MPU6050_driver_HWI.h) for interface with the hardware (also known as port)
 *           and register maps (MPU6050_driver_register_map.h, QMC5883L_driver_register_map.h
 *           and BMP180_driver_register_map.h). In case of need to port this driver to another
 *           platform, please only modify the low layer abstraction layer files where the
 *           labels indicate it.
 *
 * @details: This driver uses I2C for the communication with the GY87 module, in standard
 *           mode with no interrupts or DMA.
 */

#ifndef INC_MPU6050_DRIVER_HWI_H
#define INC_MPU6050_DRIVER_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/* BEGIN MODIFY 1 */
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
/* END MODIFY 1 */
#include "LoggingSystem_UAI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */

/**
 * @brief MPU6050 handle structure definition
 */
typedef struct {
    uint8_t            instance;
    I2C_HandleTypeDef *hi2c;
    uint8_t            address;
    uint8_t           *buffer;
    uint8_t            bufferSize;
} GY87_HandleTypeDef_t;

/**
 * @brief bool_t type definition
 */
typedef bool bool_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Initializes the I2C peripheral.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval true:  If communication could be initialized.
 *         false: If communication couldn't be initialized.
 */
bool_t I2C_Init(GY87_HandleTypeDef_t *hgy87);

/*
 * @brief  Reads I2C device registers.
 * @param  hi2c:     Pointer to a I2C_HandleTypeDef structure that contains
 *                   the configuration information for the I2C communication.
 *         address:  I2C device address to read.
 *         reg:		 I2C device register to read.
 *         data:     Pointer to variable that holds the data to be read.
 *         dataSize: Size of the data to be read.
 * @retval true:     I2C device register could be read.
 *         false:    I2C device register couldn't be read.
 */
bool_t I2C_Read(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint8_t *data, uint8_t dataSize);

/*
 * @brief  Writes to I2C device register.
 * @param  hi2c:    Pointer to a I2C_HandleTypeDef structure that contains
 *                  the configuration information for the I2C communication.
 *         address: I2C device address to write.
 *         reg:		I2C device register to write.
 *         data:    Pointer to variable that holds the data to be written.
 * @retval true:    I2C device register could be written.
 *         false:   I2C device register couldn't be written.
 */
bool_t I2C_Write(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint8_t *data);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_DRIVER_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
