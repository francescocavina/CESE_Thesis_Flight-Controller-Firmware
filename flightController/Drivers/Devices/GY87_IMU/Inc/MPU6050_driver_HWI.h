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
 * @date:    22/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.4.0
 *
 * @brief:   TODO
 */

#ifndef INC_MPU6050_DRIVER_HWI_H
#define INC_MPU6050_DRIVER_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* BEGIN MODIFY 1 */
#include "stm32f4xx_hal.h"
/* END MODIFY 1 */
// TODO
#include "LoggingSystem_UAI.h"
#include "cmsis_os.h"

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
    uint8_t instance;
    I2C_HandleTypeDef * hi2c;
    uint8_t address;
    uint8_t * buffer;
    uint8_t bufferSize;
} GY87_HandleTypeDef_t;

/**
 * @brief bool_t type definition
 */
typedef bool bool_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
bool_t I2C_Init(GY87_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
bool_t I2C_Read(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
bool_t I2C_Write(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_DRIVER_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
