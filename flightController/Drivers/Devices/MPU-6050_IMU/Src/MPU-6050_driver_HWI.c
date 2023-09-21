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
 * @date:    16/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU-6050_driver_HWI.h"
#include "MPU-6050_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MPU_6050_I2C_READ_TIMEOUT     (100) // 100 ms
#define MPU_6050_I2C_WRITE_TIMEOUT    (100) // 100 ms
#define MPU_6050_ADDR_SIZE            (1)   // 1 Byte
#define MPU_6050_READ_READ_DATA_SIZE  (1)   // 1 Byte
#define MPU_6050_READ_WRITE_DATA_SIZE (1)   // 1 Byte

/* --- Private data type declarations ---------------------------------------------------------- */
static I2C_HandleTypeDef hi2c;

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  I2C1 Initialization Function
 * @param  TODO
 * @retval TODO
 */
static bool_t MX_I2C_Init(I2C_HandleTypeDef * hi2c);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t MX_I2C_Init(I2C_HandleTypeDef * hi2c) {
    /* BEGIN MODIFY 1*/
    hi2c->Instance = I2C1;
    hi2c->Init.ClockSpeed = 400000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    /* END MODIFY 1 */

    /* BEGIN MODIFY 2 */
    if (HAL_OK != HAL_I2C_Init(hi2c)) {
        /* END MODIFY 2 */
        return false;
    }

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */
bool_t i2c_Init(MPU6050_HandleTypeDef_t * hmpu6050) {
    uint8_t who_am_I_value;

    /* Set I2C_HandleTypeDef to MPU6050 instance */
    if (hmpu6050->instance == 1) {

        hmpu6050->hi2c = &hi2c;
    } else if (hmpu6050->instance == 2) {
        // TODO
        return true;
    }

    /* Initialize I2C */
    if (!MX_I2C_Init(hmpu6050->hi2c)) {
        /* I2C initialization was unsuccessful */
        return false;
    }

    /* Read IMU device ID */
    i2c_Read(hmpu6050, MPU_6050_REG_WHO_AM_I_MPU6050, &who_am_I_value, 1);
    /* Check IMU device ID */
    if (who_am_I_value == MPU_6050_VALUE_WHO_AM_I) {
        /* Right IMU device ID */
        return true;
    } else {
        /* Wrong IMU device ID */
        return false;
    }
}

void i2c_Read(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg, uint8_t * data, uint8_t dataSize) {

    /* Read IMU data by passing a data register */
    HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->address, reg, MPU_6050_ADDR_SIZE, data, dataSize,
                     MPU_6050_I2C_READ_TIMEOUT);
}

void i2c_Write(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg, uint8_t * data, uint8_t dataSize) {

    HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->address, reg, MPU_6050_ADDR_SIZE, data, dataSize,
                      MPU_6050_I2C_WRITE_TIMEOUT);
}

/* --- End of file ----------------------------------------------------------------------------- */
