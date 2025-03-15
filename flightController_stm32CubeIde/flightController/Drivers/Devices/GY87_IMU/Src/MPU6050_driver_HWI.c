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
 * @file:    MPU6050_driver_HWI.c
 * @date:    10/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.5.0
 *
 * @brief:  This is a driver for the GY87 IMU module.
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

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_HWI.h"
#include "MPU6050_driver_register_map.h"

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
bool_t I2C_Init(GY87_HandleTypeDef_t *hgy87) {

    uint8_t who_am_I_value;

    /* Check parameter */
    if (NULL == hgy87) {
        return false;
    }

    /* Read IMU device ID */
    I2C_Read(hgy87->hi2c, hgy87->address, MPU_6050_REG_WHO_AM_I, &who_am_I_value, 1);

    /* Check IMU device ID */
    if (who_am_I_value == MPU_6050_BIT_WHO_AM_I) {
        /* Right IMU device ID */
        return true;
    } else {
        /* Wrong IMU device ID */
        return false;
    }
}

bool_t I2C_Read(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint8_t *data, uint8_t dataSize) {

    /* Check parameters */
    if (NULL == hi2c) {
        return false;
    }
    if (NULL == data) {
        return false;
    }

    /* Read I2C device data by passing a data register */
    /* BEGIN MODIFY 1 */
    if (HAL_OK != HAL_I2C_Mem_Read(hi2c, address, reg, MPU_6050_ADDR_SIZE, data, dataSize, MPU_6050_I2C_READ_TIMEOUT)) {
        /* END MODIFY 1 */
        /* Data couldn't be read */
        return false;
    } else {
        /* Data read successfully */
        return true;
    }
}

bool_t I2C_Write(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint8_t *data) {

    /* Check parameters */
    if (NULL == hi2c) {
        return false;
    }
    if (NULL == data) {
        return false;
    }

    /* Write to I2C device register */
    /* BEGIN MODIFY 2 */
    if (HAL_OK != HAL_I2C_Mem_Write(hi2c, address, reg, MPU_6050_ADDR_SIZE, data, sizeof(*data), MPU_6050_I2C_WRITE_TIMEOUT)) {
        /* END MODIFY 2 */
        /* Data couldn't be written */
        return false;
    } else {

        /* Data written successfully */
        return true;
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
