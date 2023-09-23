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
 * @file:    MPU-6050_driver_UAI.c
 * @date:    16/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU-6050_driver_UAI.h"
#include "MPU-6050_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MPU6050_I2C_ADDR1             (0xD0) // First address for first MPU-6050 device
#define MPU6050_I2C_ADDR2             (0xD1) // Second address for second MPU-6050 device
#define MPU6050_MAX_NUMBER_INSTANCES  (2) // Maximum number of possible IMUs connected to the i2c bus
#define MPU6050_GYROSCOPE_SAMPLE_RATE (8000) // 8 kHz
#define MPU6050_SAMPLE_RATE           (500)  // 5OO Hz

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static MPU6050_HandleTypeDef_t * MPU6050_IMU_InstanceInit();

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_IMU_Config();

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_IMU_ReadRegister(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg,
                                     uint8_t * data, uint8_t dataSize);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_IMU_WriteRegister(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg,
                                      uint8_t * data, uint8_t dataSize);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static MPU6050_HandleTypeDef_t * MPU6050_IMU_InstanceInit() {

    /* Check if driver was already once or twice initialized */
    if (MPU6050_MAX_NUMBER_INSTANCES == instancesNumber) {
        return NULL;
    }

#ifdef USE_FREERTOS
    /* Allocate dynamic memory for the MPU6050_HandleTypeDef_t structure */
    MPU6050_HandleTypeDef_t * hmpu6050 = pvPortmalloc(sizeof(MPU6050_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = pvortMalloc(sizeof(1));
#else
    /* Allocate dynamic memory for the MPU6050_HandleTypeDef_t structure */
    MPU6050_HandleTypeDef_t * hmpu6050 = malloc(sizeof(MPU6050_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = malloc(sizeof(1));
#endif

    /* Check if dynamic memory allocation was successful */
    if (NULL == hmpu6050 || NULL == buffer) {
        /* Dynamic memory allocation was not successful */
#ifdef USE_FREERTOS
        /* Free up dynamic allocated memory */
        vPortFree(hmpu6050->buffer);
        vPortFree(hmpu6050);
#else
        /* Free up dynamic allocated memory */
        hmpu6050->buffer = 0;
        free(hmpu6050->buffer);
        free(hmpu6050);
#endif
        return NULL;
    } else {
        /* Dynamic memory allocation was successful */

        /* Initialize MPU6050_HandleTypeDef_t structure */
        if (instancesNumber == 0) {
            hmpu6050->instance = 1;
            hmpu6050->address = MPU6050_I2C_ADDR1;
        } else if (instancesNumber == 1) {
            hmpu6050->instance = 2;
            hmpu6050->address = MPU6050_I2C_ADDR2;
        }
        hmpu6050->buffer = buffer;
    }

    /* Return created instance */
    return hmpu6050;
}

static void MPU6050_IMU_Config(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure MPU6050 device */
    uint8_t regValue;

    /* --- Wake up device ---------------------------------------------------------------------- */
    regValue = 0x00;
    /* Write '0' to PWR_MGMT_1 register on SLEEP_MODE bit (6) */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_PWR_MGMT_1, &regValue, sizeof(regValue));

    /* --- Set clock source -------------------------------------------------------------------- */
    regValue = 0x01;
    /* Write '001' to PWR_MGMT_1 register on CLKSEL bits [2:0] */
    /* Set clock source to be PLL with X axis gyroscope reference */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_PWR_MGMT_1, &regValue, sizeof(regValue));

    /* --- Set sample rate divider ------------------------------------------------------------- */
    regValue = (MPU6050_GYROSCOPE_SAMPLE_RATE - MPU6050_SAMPLE_RATE) / MPU6050_SAMPLE_RATE;
    /*
     * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
     * SMPLRT_DIV = (Gyroscope Output Rate - Sample Rate) / Sample Rate
     * Gyroscope Output Rate = 8 kHz when the DLPF is disabled, for sample rate = 500 Hz,
     * SAMPLRT_DIVE = Gyroscope Output Rate = 1 kHz when the DLPF is enabled, for sample rate = 500
     * Hz, S
     */
    /* Write regValue to SMPLRT_DIV register bits [7:0] */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_SMPLRT_DIV, &regValue, sizeof(regValue));

    /* --- Configure gyroscope ----------------------------------------------------------------- */
    regValue = 11 << 3;
    /* Write '11' to GYRO_CONFIG register on FS_SEL bits [4:3] */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_GYRO_CONFIG, &regValue, sizeof(regValue));

    /* --- Configure accelerometer ------------------------------------------------------------- */
    regValue = 11 << 3;
    /* Write '11' to ACCEL_CONFIG register on FS_SEL bits [4:3] */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_ACCEL_CONFIG, &regValue, sizeof(regValue));
}

static void MPU6050_IMU_ReadRegister(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg,
                                     uint8_t * data, uint8_t dataSize) {

    i2c_Read(hmpu6050, reg, data, dataSize);
}

static void MPU6050_IMU_WriteRegister(MPU6050_HandleTypeDef_t * hmpu6050, uint8_t reg,
                                      uint8_t * data, uint8_t dataSize) {

    i2c_Write(hmpu6050, reg, data, dataSize);
}

/* --- Public function implementation ---------------------------------------------------------- */
MPU6050_HandleTypeDef_t * MPU6050_IMU_Init() {

    /* Create an instance of the MPU6050_IMU device */
    MPU6050_HandleTypeDef_t * hmpu6050 = MPU6050_IMU_InstanceInit();

    /* Check if instance was successfully created */
    if (NULL != hmpu6050) {
        /* Instance was successfully created */

        /* Initialize I2C communication */
        if (i2c_Init(hmpu6050)) {

            /* Initialization was successful */
            /* Configure device */
            // MPU6050_IMU_Config(hmpu6050);

            instancesNumber++;

            return hmpu6050;
        } else {

            /* Initialization was unsuccessful */
#ifdef USE_FREERTOS
            /* Free up dynamic allocated memory */
            vPortFree(hmpu6050->buffer);
            vPortFree(hmpu6050);
#else
            /* Free up dynamic allocated memory */
            free(hmpu6050->buffer);
            free(hmpu6050);
#endif

            return NULL;
        }
    } else {

        /* Instance couldn't be created */
        return NULL;
    }
}

void MPU6050_IMU_Reset(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Reset device */
    uint8_t regValue = 1 << 7;

    /* Write '1' to PWR_MGMT_1 register to DEVICE_RESET bit (7) */
    MPU6050_IMU_WriteRegister(hmpu6050, MPU_6050_REG_PWR_MGMT_1, &regValue, sizeof(regValue));
}

uint16_t MPU6050_IMU_ReadGyroscope(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Read gyroscope */
    uint16_t gyroValue;
    uint8_t gyroValue_L;
    uint8_t gyroValue_H;

    MPU6050_IMU_ReadRegister(hmpu6050, MPU_6050_REG_GYRO_XOUT_L, &gyroValue_L, sizeof(gyroValue_L));
    MPU6050_IMU_ReadRegister(hmpu6050, MPU_6050_REG_GYRO_XOUT_H, &gyroValue_H, sizeof(gyroValue_H));

    gyroValue = gyroValue_H << 8 | gyroValue_L;

    return gyroValue;
}

/* --- End of file ----------------------------------------------------------------------------- */
