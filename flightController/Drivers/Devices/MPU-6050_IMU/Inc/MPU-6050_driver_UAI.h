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
 * @file:    MPU-6050_driver_UAI.h
 * @date:    16/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for header files.
 */

#ifndef INC_MPU_6050_DRIVER_UAI_H
#define INC_MPU_6050_DRIVER_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU-6050_driver_HWI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
typedef struct gyroscopeValues {
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;
} gyroscopeValues_t;

typedef struct accelerometerValues {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
} accelerometerValues_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
MPU6050_HandleTypeDef_t * MPU6050_IMU_Init(I2C_HandleTypeDef * hi2c);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
void MPU6050_IMU_Reset(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
void MPU6050_IMU_ReadGyroscope(MPU6050_HandleTypeDef_t * hmpu6050,
                               gyroscopeValues_t * gyroscopeValues);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
void MPU6050_IMU_ReadAccelerometer(MPU6050_HandleTypeDef_t * hmpu6050,
                                   accelerometerValues_t * accelerometerValues);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU_6050_DRIVER_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
