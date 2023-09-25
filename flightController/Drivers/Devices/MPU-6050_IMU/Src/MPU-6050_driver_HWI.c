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
 * @file:    MPU-6050_driver_HWI.c
 * @date:    25/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.1.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU-6050_driver_HWI.h"
#include "MPU-6050_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MPU_6050_I2C_READ_TIMEOUT  (100) // 100 ms
#define MPU_6050_I2C_WRITE_TIMEOUT (100) // 100 ms
#define MPU_6050_ADDR_SIZE         (1)   // 1 Byte
#define MPU_6050_MIN_REG_ADDR      (0x00)
#define MPU_6050_MAX_REG_ADDR      (0x75)
#define MPU_6050_MIN_DATA_SIZE     (1)
#define MPU_6050_MAX_DATA_SIZE     (10)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
bool_t I2C_Init(MPU6050_HandleTypeDef_t * hmpu6050) {

    uint8_t who_am_I_value;

    /* Check parameter */
    if (NULL == hmpu6050) {
        return false;
    }

    /* Read IMU device ID */
    I2C_Read(hmpu6050, MPU_6050_REG_WHO_AM_I_MPU6050, &who_am_I_value, 1);
    /* Check IMU device ID */
    if (who_am_I_value == MPU_6050_VALUE_WHO_AM_I) {
        /* Right IMU device ID */
        return true;
    } else {
        /* Wrong IMU device ID */
        return false;
    }
}

bool_t I2C_Read(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg, uint8_t * data, uint8_t dataSize) {

    /* Check parameters */
    if (NULL == hmpu6050) {
        return false;
    }
    if (reg < MPU_6050_MIN_REG_ADDR || reg > MPU_6050_MAX_REG_ADDR) {
        return false;
    }
    if (NULL == data) {
        return false;
    }
    if (dataSize < MPU_6050_MIN_DATA_SIZE || dataSize > MPU_6050_MAX_DATA_SIZE) {
        return false;
    }

    /* Read IMU data by passing a data register */
    if (HAL_OK != HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->address, reg, MPU_6050_ADDR_SIZE, data,
                                   dataSize, MPU_6050_I2C_READ_TIMEOUT)) {

        /* Data couldn't be read */
        return false;
    } else {

        /* Data read successfully */
        return true;
    }
}

bool_t I2C_Write(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg, uint8_t * data,
                 uint8_t dataSize) {

    /* Check parameters */
    if (NULL == hmpu6050) {
        return false;
    }
    if (reg < MPU_6050_MIN_REG_ADDR || reg > MPU_6050_MAX_REG_ADDR) {
        return false;
    }
    if (NULL == data) {
        return false;
    }
    if (dataSize < MPU_6050_MIN_DATA_SIZE || dataSize > MPU_6050_MAX_DATA_SIZE) {
        return false;
    }

    /* Write to IMU */
    if (HAL_OK != HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->address, reg, MPU_6050_ADDR_SIZE,
                                    data, dataSize, MPU_6050_I2C_WRITE_TIMEOUT)) {

        /* Data couldn't be written */
        return false;
    } else {

        /* Data written successfully */
        return true;
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
