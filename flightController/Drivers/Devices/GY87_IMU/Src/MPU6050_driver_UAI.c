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
 * @date:    22/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.4.0
 *
 * @brief:   TODO
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_UAI.h"
#include "MPU6050_driver_register_map.h"
#include "QMC5883L_driver_register_map.h"
#include "BMP180_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define USE_FREERTOS                 // Remove comment when using FreeRTOS
#define MPU6050_USE_LOGGING          // Remove comment to allow driver info logging

#define MPU6050_MAX_NUMBER_INSTANCES (2) // Maximum number of possible IMUs connected to the i2c bus
#define MPU6050_SET_BIT              (1)
#define MPU6050_CLEAR_BIT            (0)
#define QMC5883L_SET_BIT             (1)
#define QMC5883L_CLEAR_BIT           (0)
#define BMP180_SET_BIT               (1)
#define BMP180_CLEAR_BIT             (0)

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define RADIANS_TO_DEGREES_CONST (180 / M_PI)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Initializes an instance of the IMU device. Only two devices can be initialized.
 * @param  TODO
 * @retval TODO
 */
static MPU6050_HandleTypeDef_t * MPU6050_InstanceInit(I2C_HandleTypeDef * hi2c);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_WakeUpDevice(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_SetClockSource(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_SetSampleDivider(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_SetGyroscopeRange(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_SetAccelerometerRange(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_EnableI2CMasterMode(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_DisableI2CMasterMode(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_EnableBypassMode(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_DisableBypassMode(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_SetMasterClock(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static bool_t MPU6050_TestConnection_QMC5883L(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static bool_t MPU6050_TestConnection_BMP180(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void QMC5883L_Configure(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void BMP180_Configure(MPU6050_HandleTypeDef_t * hmpu6050);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
static void MPU6050_Config();

/*
 * @brief  Reads IMU register.
 * @param  TODO
 * @retval None
 */
static void MPU6050_ReadRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize);

/*
 * @brief  Writes IMU register. It is a destructive operation
 * @param  TODO
 * @retval None
 */
static void MPU6050_WriteRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data);

/*
 * @brief  Writers IMU register. It is a non-destructive operation.
 * @param  TODO
 * @retval None
 */
static void MPU6050_WriteRegisterBitmasked(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t set);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static MPU6050_HandleTypeDef_t * MPU6050_InstanceInit(I2C_HandleTypeDef * hi2c) {

#ifdef USE_FREERTOS
    /* Allocate dynamic memory for the MPU6050_HandleTypeDef_t structure */
    MPU6050_HandleTypeDef_t * hmpu6050 = pvPortMalloc(sizeof(MPU6050_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = pvPortMalloc(sizeof(1));
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

static void MPU6050_WakeUpDevice(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Wake up device */
    uint8_t regData;

    regData = MPU_6050_BIT_PWR_MGMT_1_SLEEP;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_CLEAR_BIT);
}

static void MPU6050_SetClockSource(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Set clock source */
    uint8_t regData;

    regData = MPU_6050_BIT_PWR_MGMT_1_CLKSEL_1;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetSampleDivider(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Set sample rate divider */
    uint8_t regData;

    regData = MPU_6050_BIT_SMPLRT_DIV;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_SMPLRT_DIV, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetGyroscopeRange(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Set gyroscope range */
    uint8_t regData;

    regData = MPU_6050_BIT_GYRO_CONFIG_FS_SEL_3; // Full range
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetAccelerometerRange(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Set accelerometer range */
    uint8_t regData;

    regData = MPU_6050_BIT_ACCEL_CONFIG_FS_SEL_3; // Full range
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_EnableI2CMasterMode(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Enable I2C Master mode */
    uint8_t regData;

    regData = 0b00100010; // TODO
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_USER_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_DisableI2CMasterMode(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Disable I2C Master mode */
    uint8_t regData;

    regData = MPU_6050_BIT_USER_CTRL_MST_EN;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_USER_CTRL, &regData, MPU6050_CLEAR_BIT);
}

static void MPU6050_EnableBypassMode(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Enable Bypass mode */
    uint8_t regData;

    regData = MPU_6050_BIT_INT_PIN_CFG_I2C_BP_EN;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_INT_PIN_CFG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_DisableBypassMode(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Disable Bypass mode */
    uint8_t regData;

    regData = MPU_6050_BIT_INT_PIN_CFG_I2C_BP_EN;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_INT_PIN_CFG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetMasterClock(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Set Master Clock */
    uint8_t regData;

    regData = 0b00001101; // 400 kHz TODO
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_MST_CTRL, &regData, MPU6050_SET_BIT);
}

static bool_t MPU6050_TestConnection_QMC5883L(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Test QMC5883L magnetometer connection */
    uint8_t regData;

    MPU6050_ReadRegister(hmpu6050->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CHIP_ID, &regData, sizeof(regData));

    if (QMC5883L_BIT_CHIP_ID != regData) {
        return false;
    } else {
        return true;
    }
}

static bool_t MPU6050_TestConnection_BMP180(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Test BMP180 barometer connection */
    uint8_t regData;

    MPU6050_ReadRegister(hmpu6050->hi2c, BMP180_AUX_VAL_I2C_ADDR << 1, 0xD0, &regData, sizeof(regData));

    if (0x55 != regData) { // TODO
        return false;
    } else {
        return true;
    }
}

static void QMC5883L_Configure(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure QMC5883L magnetometer */
    uint8_t regData;

    /* Reset QMC5883L magnetometer */
    regData = 0b00000001;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_RESET, &regData, QMC5883L_SET_BIT);

    /* Configure QMC5883L magnetometer: Control Register 1 */
    regData = 0b00011101;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CONFIG1, &regData, QMC5883L_SET_BIT);

    /* Configure QMC5883L magnetometer: Control Register 2 */
    regData = 0b00000000;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CONFIG2, &regData, QMC5883L_SET_BIT);
}

static void BMP180_Configure(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure BMP180 barometer */
    uint8_t regData;

    regData = 0b01000000;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, BMP180_AUX_VAL_I2C_ADDR << 1, 0xF4, &regData, BMP180_SET_BIT);
}

static void MPU6050_Configure_QMC5883l(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure slave QMC5883L magnetometer */
    uint8_t regData;

    /* Set slave QMC5883L magnetometer device address */
    regData = QMC5883L_AUX_VAL_I2C_ADDR | 0x80;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_ADDR, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer registers addresses to read */
    regData = QMC5883L_REG_X_LSB;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_REG, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer number of registers to read*/
    regData = 0x80 | 0x06;
    MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_I2C_SLV0_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_Config(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Configure MPU6050 device */

    /* Wake up device */
    MPU6050_WakeUpDevice(hmpu6050);

    /* Set clock source */
    MPU6050_SetClockSource(hmpu6050);

    /* Set sample rate divider */
    MPU6050_SetSampleDivider(hmpu6050);

    /* Set gyroscope range */
    MPU6050_SetGyroscopeRange(hmpu6050);

    /* Set accelerometer range */
    MPU6050_SetAccelerometerRange(hmpu6050);

    //    /* Disable I2C Master mode */
    //    MPU6050_DisableI2CMasterMode(hmpu6050);
    //
    //    /* Enable Bypass mode */
    //    MPU6050_EnableBypassMode(hmpu6050);
    //
    //    /* Test QMC5883L magnetometer connection */
    //    if (!MPU6050_TestConnection_QMC5883L(hmpu6050)) {
    // #ifdef MPU6050_USE_LOGGING
    //        LOG((uint8_t *)"QMC5883L magnetometer not detected.\r\n\n", LOG_ERROR);
    // #endif
    //        // return -1; TODO
    //    } else {
    // #ifdef MPU6050_USE_LOGGING
    //        LOG((uint8_t *)"QMC5883L magnetometer detected.\r\n\n", LOG_INFORMATION);
    // #endif
    //    }
    //
    //    /* Configure QMC5883L magnetometer */
    //    QMC5883L_Configure(hmpu6050);
    //
    //    /* Test BMP180 barometer connection */
    //    if (!MPU6050_TestConnection_BMP180(hmpu6050)) {
    // #ifdef MPU6050_USE_LOGGING
    //        LOG((uint8_t *)"BMP180 barometer not detected.\r\n\n", LOG_ERROR);
    // #endif
    //        // return -1; TODO
    //    } else {
    // #ifdef MPU6050_USE_LOGGING
    //        LOG((uint8_t *)"BMP180 barometer detected.\r\n\n", LOG_INFORMATION);
    // #endif
    //    }
    //
    //    /* Configure BMP180 barometer */
    //    BMP180_Configure(hmpu6050);
    //
    //    /* Disable Bypass */
    //    MPU6050_DisableBypassMode(hmpu6050);
    //
    //    /* Enable I2C Master mode */
    //    MPU6050_EnableI2CMasterMode(hmpu6050);
    //
    //    /* Set Master clock */
    //    MPU6050_SetMasterClock(hmpu6050);
    //
    //    /* Configure slave QMC5883L magnetometer */
    //    MPU6050_Configure_QMC5883l(hmpu6050);
}

static void MPU6050_ReadRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize) {

    /* Read register */
    I2C_Read(hi2c, address, reg, data, dataSize);
}

static void MPU6050_WriteRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data) {

    /* Write register - ¡Destructive operation! */
    I2C_Write(hi2c, address, reg, data);
}

static void MPU6050_WriteRegisterBitmasked(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t set) {

    /* Declare variable for original data read from register */
    uint8_t originalData;

    /* Declare variable for new data to write into register */
    uint8_t newData;

    MPU6050_ReadRegister(hi2c, address, reg, &originalData, sizeof(originalData));

    /* Apply mask to data to write */
    if (set) {

        newData = originalData | *data;
    } else {

        newData = originalData & (~*data);
    }

    MPU6050_WriteRegister(hi2c, address, reg, &newData);
}

/* --- Public function implementation ---------------------------------------------------------- */
MPU6050_HandleTypeDef_t * MPU6050_Init(I2C_HandleTypeDef * hi2c) {

    /* Check parameter */
    if (NULL == hi2c) {
        return NULL;
    }

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
#ifdef MPU6050_USE_LOGGING
            LOG((uint8_t *)"MPU6050 IMU detected.\r\n\n", LOG_INFORMATION);
#endif

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

#ifdef MPU6050_USE_LOGGING
            LOG((uint8_t *)"MPU6050 IMU not detected.\r\n\n", LOG_ERROR);
#endif
            return NULL;
        }
    } else {

        /* Instance couldn't be created */
#ifdef MPU6050_USE_LOGGING
        LOG((uint8_t *)"MPU6050 IMU couldn't be initialized.\r\n\n", LOG_ERROR);
#endif
        return NULL;
    }
}

void MPU6050_Reset(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Declare variable for data to write into register */
    uint8_t regData;

    /* Check parameter */
    if (NULL != hmpu6050) {

        /* Set data to write into register */
        regData = MPU_6050_BIT_PWR_MGMT_1_DEVICE_RESET;

        /* Reset device */
        MPU6050_WriteRegisterBitmasked(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_SET_BIT);
    }
}

void MPU6050_ReadGyroscope(MPU6050_HandleTypeDef_t * hmpu6050, gyroscopeValues_t * gyroscopeValues) {

    /* Declare variable for raw data */
    uint8_t gyroscopeRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_GYRO_SF_2000;

    /* Check parameters */
    if (NULL != hmpu6050 && NULL != gyroscopeValues) {

        /* Read gyroscope in axis X */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_XOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeX = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

        /* Read gyroscope in axis Y */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_YOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeY = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

        /* Read gyroscope in axis Z */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_GYRO_ZOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeZ = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

    } else {
        /* Wrong parameters */
        gyroscopeValues->gyroscopeX = 0;
        gyroscopeValues->gyroscopeY = 0;
        gyroscopeValues->gyroscopeZ = 0;
    }
}

void MPU6050_ReadAccelerometer(MPU6050_HandleTypeDef_t * hmpu6050, accelerometerValues_t * accelerometerValues) {

    /* Declare variable for raw data */
    uint8_t accelerometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_ACCEL_SF_16;

    /* Check parameters */
    if (NULL != hmpu6050 && NULL != accelerometerValues) {

        /* Read accelerometer in axis X */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_XOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerX = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

        /* Read accelerometer in axis Y */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_YOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerY = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

        /* Read accelerometer in axis Z */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_ACCEL_ZOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerZ = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

    } else {
        /* Wrong parameters */
        accelerometerValues->accelerometerX = 0;
        accelerometerValues->accelerometerY = 0;
        accelerometerValues->accelerometerZ = 0;
    }
}

int16_t MPU6050_ReadTemperatureSensor(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Declare variable for raw data */
    uint8_t temperatureSensorRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_TEMP_SF;

    /* Define variable to offset raw data */
    int16_t offset = MPU_6050_AUX_VAL_TEMP_OFS;

    /* Check parameter */
    if (NULL == hmpu6050) {
        return 0;
    }

    /* Read temperature sensor */
    MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_TEMP_OUT_H, temperatureSensorRawData, sizeof(uint16_t));

    return ((int16_t)(temperatureSensorRawData[0] << 8 | temperatureSensorRawData[1]) / scaleFactor) + offset;
}

void MPU6050_ReadMagnetometer(MPU6050_HandleTypeDef_t * hmpu6050, magnetometerValues_t * magnetometerValues) {

    /* Declare variable for raw data */
    uint8_t magnetometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = 1;

    /* Check parameters */
    if (NULL != hmpu6050 && NULL != magnetometerValues) {

        /* Read magnetometer in axis X */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_00, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerX = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

        /* Read magnetometer in axis Y */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_02, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerY = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

        /* Read magnetometer in axis Z */
        MPU6050_ReadRegister(hmpu6050->hi2c, hmpu6050->address, MPU_6050_REG_EXT_SENS_DATA_04, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerZ = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

    } else {
        /* Wrong parameters */
        magnetometerValues->magnetometerX = 0;
        magnetometerValues->magnetometerY = 0;
        magnetometerValues->magnetometerZ = 0;
    }
}

int16_t MPU6050_ReadMagnetometerHeading(MPU6050_HandleTypeDef_t * hmpu6050) {

    /* Allocate dynamic memory for the magnetometerValues_t structure */
#ifdef USE_FREERTOS
    magnetometerValues_t * magnetometerValues = pvPortMalloc(sizeof(magnetometerValues_t));
#else
    magnetometerValues_t * magnetometerValues = malloc(sizeof(magnetometerValues_t));
#endif

    /* Declare variable for compass heading */
    float heading;

    /* Check parameters */
    if (NULL != hmpu6050) {

        MPU6050_ReadMagnetometer(hmpu6050, magnetometerValues);

        /* Calculate heading */
        heading = atan2(magnetometerValues->magnetometerY, magnetometerValues->magnetometerX) * RADIANS_TO_DEGREES_CONST;

        /* Check if heading is within 0 and 360 degrees */
        if (heading < 0) {
            heading += 360;
        }

    } else {

        heading = -1;
    }

    /* Free up dynamic allocated memory */
#ifdef USE_FREERTOS
    vPortFree(magnetometerValues);
#else
    free(magnetometerValues);
#endif

    return heading;
}

/* --- End of file ----------------------------------------------------------------------------- */
