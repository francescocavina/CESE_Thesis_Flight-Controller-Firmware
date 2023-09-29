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
 * @file:    MPU6050_driver_UAI.c
 * @date:    28/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.2.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_UAI.h"
#include "MPU6050_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MPU6050_MAX_NUMBER_INSTANCES (2) // Maximum number of possible IMUs connected to the i2c bus

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static MPU6050_HandleTypeDef_t * MPU6050_InstanceInit(I2C_HandleTypeDef * hi2c);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_Config();

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_ReadRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static void MPU6050_WriteRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static MPU6050_HandleTypeDef_t * MPU6050_InstanceInit(I2C_HandleTypeDef * hi2c) {

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
            hmpu6050->address = MPU6050_AUX_VAL_I2C_ADDR1;
        } else if (instancesNumber == 1) {
            hmpu6050->instance = 2;
            hmpu6050->address = MPU6050_AUX_VAL_I2C_ADDR2;
        }
        hmpu6050->hi2c = hi2c;
        hmpu6050->buffer = buffer;
    }

    /* Return created instance */
    return hmpu6050;
}

static void MPU6050_Config(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure MPU6050 device */
    uint8_t regValue;

    /* Wake up device */
    regValue = 0;
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regValue);

    /* Set clock source */
    regValue = MPU_6050_BIT_PWR_MGMT_1_CLKSEL_1;
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regValue);

    /* Set sample rate divider */
    regValue = MPU_6050_BIT_SMPLRT_DIV;
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_SMPLRT_DIV, &regValue);

    /* Configure gyroscope full scale range */
    regValue = MPU_6050_BIT_GYRO_CONFIG_FS_SEL_3;
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_CONFIG, &regValue);

    /* Configure accelerometer full scale range */
    regValue = MPU_6050_BIT_ACCEL_CONFIG_FS_SEL_3;
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_CONFIG, &regValue);

    //    /* Disable I2C Master Mode */
    //    regValue = MPU_6050_VAL_USER_CTRL_MST_DIS;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_USER_CTRL, &regValue,
    //                              sizeof(regValue));
    //
    //    /* Enable Bypass */
    //    regValue = MPU_6050_REG_INT_PIN_CFG_I2C_BP_EN;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_INT_PIN_CFG,
    //                              &regValue, sizeof(regValue));
    //
    //    /* Configure HMC5883L itself */
    //    regValue = 0b00011000; // Fill Slave0 DO
    //    MPU6050_WriteRegister(hmpu6050->hi2c, HMC5883L_AUX_VAL_I2C_ADDR << 1, HMC5883L_REG_CONFIG_A,
    //                              &regValue, sizeof(regValue));
    //    regValue = 0b00100000; // Fill Slave0 DO
    //    MPU6050_WriteRegister(hmpu6050->hi2c, HMC5883L_AUX_VAL_I2C_ADDR << 1, HMC5883L_REG_CONFIG_B,
    //                              &regValue, sizeof(regValue));
    //    regValue = 0x00; // Mode: Continuous
    //    MPU6050_WriteRegister(hmpu6050->hi2c, HMC5883L_AUX_VAL_I2C_ADDR << 1, HMC5883L_REG_MODE,
    //                              &regValue, sizeof(regValue));
    //
    //    /* Disable Bypass */
    //    regValue = MPU_6050_REG_INT_PIN_CFG_I2C_BP_DIS;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_INT_PIN_CFG,
    //                              &regValue, sizeof(regValue));
    //
    //    /* Enable I2C Master Mode */
    //    regValue = MPU_6050_VAL_USER_CTRL_MST_EN;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_USER_CTRL, &regValue,
    //                              sizeof(regValue));
    //
    //    /* Set I2C Master Clock */
    //    regValue = MPU_6050_VAL_I2C_MST_CTRL_CLK_13;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_MST_CTRL,
    //                              &regValue, sizeof(regValue));
    //
    //    /* Configure HMC5883L in MPU-6050 */
    //    regValue = HMC5883L_AUX_VAL_I2C_ADDR | 0x80;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_ADDR,
    //                              &regValue, sizeof(regValue));
    //    regValue = 0x03;
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_REG,
    //                              &regValue, sizeof(regValue));
    //    regValue = 0x80 | 0x06; // Number of data bytes
    //    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_CTRL,
    //                              &regValue, sizeof(regValue));
}

static void MPU6050_ReadRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize) {

    I2C_Read(hi2c, address, reg, data, dataSize);
}

static void MPU6050_WriteRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data) {

    I2C_Write(hi2c, address, reg, data);
}

/* --- Public function implementation ---------------------------------------------------------- */
MPU6050_HandleTypeDef_t * MPU6050_Init(I2C_HandleTypeDef * hi2c) {

    /* Check if driver was already once or twice initialized */
    if (MPU6050_MAX_NUMBER_INSTANCES == instancesNumber) {
        return NULL;
    }

    /* Create an instance of the MPU6050_IMU device */
    MPU6050_HandleTypeDef_t * hmpu6050 = MPU6050_InstanceInit(hi2c);

    /* Check if instance was successfully created */
    if (NULL != hmpu6050) {
        /* Instance was successfully created */

        /* Initialize I2C communication */
        if (I2C_Init(hmpu6050)) {

            /* Initialization was successful */
            /* Configure device */
            MPU6050_Config(hmpu6050);

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

void MPU6050_Reset(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Reset device */
    uint8_t regValue = MPU_6050_BIT_PWR_MGMT_1_DEVICE_RESET;

    /* Write '1' to PWR_MGMT_1 register to DEVICE_RESET bit (7) */
    MPU6050_WriteRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regValue);
}

void MPU6050_ReadGyroscope(MPU6050_HandleTypeDef_t * hmpu6050, gyroscopeValues_t * gyroscopeValues) {

    gyroscopeValues->gyroscopeX = 0;
    gyroscopeValues->gyroscopeY = 0;
    gyroscopeValues->gyroscopeZ = 0;

    uint8_t gyroscopeRawData[2];
    int16_t scaleFactor = MPU_6050_AUX_VAL_GYRO_SF_2000;

    /* Read gyroscope in axis X */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_XOUT_H, gyroscopeRawData, sizeof(uint16_t));
    gyroscopeValues->gyroscopeX = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

    /* Read gyroscope in axis Y */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_YOUT_H, gyroscopeRawData, sizeof(uint16_t));
    gyroscopeValues->gyroscopeY = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

    /* Read gyroscope in axis Z */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_ZOUT_H, gyroscopeRawData, sizeof(uint16_t));
    gyroscopeValues->gyroscopeZ = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;
}

void MPU6050_ReadAccelerometer(MPU6050_HandleTypeDef_t * hmpu6050, accelerometerValues_t * accelerometerValues) {

    accelerometerValues->accelerometerX = 0;
    accelerometerValues->accelerometerY = 0;
    accelerometerValues->accelerometerZ = 0;

    uint8_t accelerometerRawData[2];
    int16_t scaleFactor = MPU_6050_AUX_VAL_ACCEL_SF_16;

    /* Read accelerometer in axis X */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_XOUT_H, accelerometerRawData, sizeof(uint16_t));
    accelerometerValues->accelerometerX = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

    /* Read accelerometer in axis Y */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_YOUT_H, accelerometerRawData, sizeof(uint16_t));
    accelerometerValues->accelerometerY = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

    /* Read accelerometer in axis Z */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_ZOUT_H, accelerometerRawData, sizeof(uint16_t));
    accelerometerValues->accelerometerZ = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;
}

int16_t MPU6050_ReadTemperatureSensor(MPU6050_HandleTypeDef_t * hmpu6050) {

    uint8_t temperatureSensorRawData[2];
    int16_t scaleFactor = MPU_6050_AUX_VAL_TEMP_SF;
    int16_t offset = MPU_6050_AUX_VAL_TEMP_OFS;

    /* Read temperature sensor */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_TEMP_OUT_H, temperatureSensorRawData, sizeof(uint16_t));

    return ((int16_t)(temperatureSensorRawData[0] << 8 | temperatureSensorRawData[1]) / scaleFactor) + offset;
}

// void MPU6050_ReadMagnetometer(MPU6050_HandleTypeDef_t * hmpu6050,
//                                   magnetometerValues_t * magnetometerValues) {
//
//     magnetometerValues->magnetometerX = 0;
//     magnetometerValues->magnetometerY = 0;
//     magnetometerValues->magnetometerZ = 0;
//
//     uint8_t magnetometerRawData[2];
//     int16_t scaleFactor = 1;
//
//     /* Read accelerometer in axis X */
//     MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_14,
//                              magnetometerRawData, sizeof(uint16_t));
//     magnetometerValues->magnetometerX =
//         (int16_t)(magnetometerRawData[0] << 8 | magnetometerRawData[1]) / scaleFactor;
//
//     /* Read accelerometer in axis Y */
//     MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_16,
//                              magnetometerRawData, sizeof(uint16_t));
//     magnetometerValues->magnetometerY =
//         (int16_t)(magnetometerRawData[0] << 8 | magnetometerRawData[1]) / scaleFactor;
//
//     /* Read accelerometer in axis Z */
//     MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_18,
//                              magnetometerRawData, sizeof(uint16_t));
//     magnetometerValues->magnetometerZ =
//         (int16_t)(magnetometerRawData[0] << 8 | magnetometerRawData[1]) / scaleFactor;
// }

/* --- End of file ----------------------------------------------------------------------------- */
