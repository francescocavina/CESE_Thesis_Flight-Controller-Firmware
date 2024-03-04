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
 * @date:    03/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.4.0
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
 * @details: In order to be able to use the IMU module, it must be
 *           initialized first. Only two devices can be initialized and therefore
 *           used.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_UAI.h"
#include "MPU6050_driver_register_map.h"
#include "QMC5883L_driver_register_map.h"
#include "BMP180_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define USE_FREERTOS // Remove comment when using FreeRTOS
// #define GY87_USE_LOGGING          // Remove comment to allow driver info logging

#define GY87_MAX_NUMBER_INSTANCES (2) // Maximum number of possible IMUs connected to the i2c bus
#define MPU6050_SET_BIT           (1)
#define MPU6050_CLEAR_BIT         (0)
#define QMC5883L_SET_BIT          (1)
#define QMC5883L_CLEAR_BIT        (0)
#define BMP180_SET_BIT            (1)
#define BMP180_CLEAR_BIT          (0)

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define QMC5883L_RADIANS_TO_DEGREES_CONST (180 / M_PI)
#define QMC5883L_MAGNETIC_DECLINATION     (0)      // Magnetic declination [degrees] for Córdoba City 02/15/2024
#define QMC5883L_CALIBRATION_OFFSET       (26)     // Calibration offset [degrees]
#define BMP180_ATMOSFERIC_PRESSURE        (101325) // Atmosferic pressure [Pascals]
#define BMP180_OVERSAMPLING               (2)      // High resolution accuracy

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;
static BMP180_CallibrationData_t BMP180_CallibrationData;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Initializes an instance of the IMU device. Only two devices can be initialized.
 * @param  hi2c: Pointer to a I2C_HandleTypeDef structure that contains the configuration
 *               information for the I2C communication.
 * @retval Pointer to a GY87_HandleTypeDef_t structure that contains the configuration
 *         information for the GY87 device.
 */
static GY87_HandleTypeDef_t * GY87_InstanceInit(I2C_HandleTypeDef * hi2c);

/*
 * @brief  Wakes up MPU6050 device.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_WakeUpDevice(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Sets clock source.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_SetClockSource(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Sets sample divider.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_SetSampleDivider(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Sets gyroscope range.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_SetGyroscopeRange(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Sets accelerometer range.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_SetAccelerometerRange(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Enbales I2C master mode.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_EnableI2CMasterMode(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Disables I2C master mode.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_DisableI2CMasterMode(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Enables bypass mode.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_EnableBypassMode(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Disables bypass mode.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_DisableBypassMode(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Sets master's clock.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_SetMasterClock(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Configures slave QMC5883L magnetometer in MPU6050 device.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_Configure_QMC5883l(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Configures slave BMP180 barometer in MPU6050 device.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_Configure_BMP180(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Reads IMU register.
 * @param  hi2c:     Pointer to a I2C_HandleTypeDef structure that contains the configuration
 *                   information for the I2C communication.
 *         address:  Device address.
 *         reg:		 Device register to read.
 *         data:	 Pointer to variable where the read information will be located.
 *         dataSize: Size of the data that will be read.
 * @retval None
 */
static void MPU6050_ReadRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t dataSize);

/*
 * @brief  Writes IMU register. It is a destructive operation
 * @param  hi2c:    Pointer to a I2C_HandleTypeDef structure that contains the configuration
 *                  information for the I2C communication.
 *         address: Device address.
 *         reg:		Device register to write.
 *         data:    Pointer to variable that holds the data to be written into the specified register.
 * @retval None
 */
static void MPU6050_WriteRegister(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data);

/*
 * @brief  Writers IMU register. It is a non-destructive operation.
 * @param  hi2c:    Pointer to a I2C_HandleTypeDef structure that contains the configuration
 *                  information for the I2C communication.
 *         address: Device address.
 *         reg:		Device register to write.
 *         data:    Pointer to variable that holds the data to be written into the specified register.
 *         set:     MPU6050_CLEAR_BIT: Clear specified bits.
 *                  MPU6050_SET_BIT: Write specified bits.
 * @retval None
 */
static void MPU6050_WriteRegisterBitmasked(I2C_HandleTypeDef * hi2c, uint8_t address, uint8_t reg, uint8_t * data, uint8_t set);

/*
 * @brief  Configures GY87 device at initialization.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval true:  GY87 could be configured.
 *         false: GY87 couldn't be configured.
 */
static bool_t GY87_Configure(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Tests if QMC5883L magnetometer was detected.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval true:  QMC5883L magnetometer was detected.
 *         false: QMC5883L magnetometer was not detected.
 */
static bool_t QMC5883L_TestConnection(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Tests if BMP180 barometer was detected.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval true:  BMP180 barometer was detected.
 *         false: BMP180 barometer was not detected.
 */
static bool_t BMP180_TestConnection(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Configures QMC5883L magnetometer directly as if MPU6050 was not
 *         interfacing it (MPU6050 must be in bypass mode).
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void QMC5883L_Configure(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Configures BMP180 barometer directly as if MPU6050 was not
 *         interfacing it (MPU6050 must be in bypass mode).
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void BMP180_Configure(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  TODO
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void BMP180_ReadCallibrationData(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  TODO
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval TODO
 */
static uint32_t BMP180_ReadUncompensatedPressure(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  TODO
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval TODO
 */
static uint32_t BMP180_ReadUncompensatedTemperature(GY87_HandleTypeDef_t * hgy87);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static GY87_HandleTypeDef_t * GY87_InstanceInit(I2C_HandleTypeDef * hi2c) {

#ifdef USE_FREERTOS
    /* Allocate dynamic memory for the GY87_HandleTypeDef_t structure */
    GY87_HandleTypeDef_t * hgy87 = pvPortMalloc(sizeof(GY87_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = pvPortMalloc(sizeof(1));
#else
    /* Allocate dynamic memory for the GY87_HandleTypeDef_t structure */
    GY87_HandleTypeDef_t * hgy87 = malloc(sizeof(GY87_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = malloc(sizeof(1));
#endif

    /* Check if dynamic memory allocation was successful */
    if (NULL == hgy87 || NULL == buffer) {
        /* Dynamic memory allocation was not successful */
#ifdef USE_FREERTOS
        /* Free up dynamic allocated memory */
        vPortFree(hgy87->buffer);
        vPortFree(hgy87);
#else
        /* Free up dynamic allocated memory */
        hgy87->buffer = 0;
        free(hgy87->buffer);
        free(hgy87);
#endif
        return NULL;
    } else {
        /* Dynamic memory allocation was successful */

        /* Initialize GY87_HandleTypeDef_t structure */
        if (instancesNumber == 0) {
            hgy87->instance = 1;
            hgy87->address = MPU6050_AUX_VAL_I2C_ADDR1;
        } else if (instancesNumber == 1) {
            hgy87->instance = 2;
            hgy87->address = MPU6050_AUX_VAL_I2C_ADDR2;
        }
        hgy87->hi2c = hi2c;
        hgy87->buffer = buffer;
    }

    /* Return created instance */
    return hgy87;
}

static void MPU6050_WakeUpDevice(GY87_HandleTypeDef_t * hgy87) {

    /* Wake up device */
    uint8_t regData;

    regData = MPU_6050_BIT_PWR_MGMT_1_SLEEP;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_CLEAR_BIT);
}

static void MPU6050_SetClockSource(GY87_HandleTypeDef_t * hgy87) {

    /* Set clock source */
    uint8_t regData;

    regData = MPU_6050_BIT_PWR_MGMT_1_CLKSEL_1;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetSampleDivider(GY87_HandleTypeDef_t * hgy87) {

    /* Set sample rate divider */
    uint8_t regData;

    regData = MPU_6050_BIT_SMPLRT_DIV;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_SMPLRT_DIV, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetGyroscopeRange(GY87_HandleTypeDef_t * hgy87) {

    /* Set gyroscope range */
    uint8_t regData;

    regData = MPU_6050_BIT_GYRO_CONFIG_FS_SEL_0; // Full range
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetAccelerometerRange(GY87_HandleTypeDef_t * hgy87) {

    /* Set accelerometer range */
    uint8_t regData;

    regData = MPU_6050_BIT_ACCEL_CONFIG_FS_SEL_0; // Full range
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_EnableI2CMasterMode(GY87_HandleTypeDef_t * hgy87) {

    /* Enable I2C Master mode */
    uint8_t regData;

    regData = 0b00100010; // TODO
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_USER_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_DisableI2CMasterMode(GY87_HandleTypeDef_t * hgy87) {

    /* Disable I2C Master mode */
    uint8_t regData;

    regData = MPU_6050_BIT_USER_CTRL_MST_EN;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_USER_CTRL, &regData, MPU6050_CLEAR_BIT);
}

static void MPU6050_EnableBypassMode(GY87_HandleTypeDef_t * hgy87) {

    /* Enable Bypass mode */
    uint8_t regData;

    regData = MPU_6050_BIT_INT_PIN_CFG_I2C_BP_EN;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_INT_PIN_CFG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_DisableBypassMode(GY87_HandleTypeDef_t * hgy87) {

    /* Disable Bypass mode */
    uint8_t regData;

    regData = MPU_6050_BIT_INT_PIN_CFG_I2C_BP_EN;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_INT_PIN_CFG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetMasterClock(GY87_HandleTypeDef_t * hgy87) {

    /* Set Master Clock */
    uint8_t regData;

    regData = 0b00001101; // 400 kHz TODO
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_MST_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_Configure_QMC5883l(GY87_HandleTypeDef_t * hgy87) {

    /* Configure slave QMC5883L magnetometer in MPU6050 */
    uint8_t regData;

    /* Set slave QMC5883L magnetometer device address */
    regData = QMC5883L_AUX_VAL_I2C_ADDR | 0x80;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_ADDR, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer registers addresses to read */
    regData = QMC5883L_REG_X_LSB;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_REG, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer number of registers to read*/
    regData = 0x80 | 0x06;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_Configure_BMP180(GY87_HandleTypeDef_t * hgy87) {

    /* Configure slave BMP180 barometer in MPU6050 */
    uint8_t regData;

    /* Set slave BMP180 barometer device address */
    regData = BMP180_AUX_VAL_I2C_ADDR | 0x80; // TODO
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV1_ADDR, &regData, MPU6050_SET_BIT);
    //
    //    /* Set slave BMP180 barometer registers addresses to read */
    //    regData = ; // TODO
    //    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV1_REG, &regData, MPU6050_SET_BIT);
    //
    //    /* Set slave BMP180 barometer number of registers to read*/
    //    regData = 0x80 | 0x06; // TODO
    //    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV1_CTRL, &regData, MPU6050_SET_BIT);
}

static bool_t GY87_Configure(GY87_HandleTypeDef_t * hgy87) {

    /* Configure MPU6050 device */

    /* Wake up device */
    MPU6050_WakeUpDevice(hgy87);

    /* Set clock source */
    MPU6050_SetClockSource(hgy87);

    /* Set sample rate divider */
    MPU6050_SetSampleDivider(hgy87);

    /* Set gyroscope range */
    MPU6050_SetGyroscopeRange(hgy87);

    /* Set accelerometer range */
    MPU6050_SetAccelerometerRange(hgy87);

    /* Disable I2C Master mode */
    MPU6050_DisableI2CMasterMode(hgy87);

    /* Enable Bypass mode */
    MPU6050_EnableBypassMode(hgy87);

    /* Test QMC5883L magnetometer connection */
    if (!QMC5883L_TestConnection(hgy87)) {
#ifdef GY87_USE_LOGGING
        LOG((uint8_t *)"QMC5883L magnetometer not detected.\r\n\n", LOG_ERROR);
#endif
        return false;
    } else {
#ifdef GY87_USE_LOGGING
        LOG((uint8_t *)"QMC5883L magnetometer detected.\r\n\n", LOG_INFORMATION);
#endif
    }

    /* Configure QMC5883L magnetometer */
    QMC5883L_Configure(hgy87);

    /* Test BMP180 barometer connection */
    if (!BMP180_TestConnection(hgy87)) {
#ifdef GY87_USE_LOGGING
        LOG((uint8_t *)"BMP180 barometer not detected.\r\n\n", LOG_ERROR);
#endif
        return false;
    } else {
#ifdef GY87_USE_LOGGING
        LOG((uint8_t *)"BMP180 barometer detected.\r\n\n", LOG_INFORMATION);
#endif
    }

    /* Configure BMP180 barometer */
    BMP180_Configure(hgy87);

    /* Disable Bypass */
    MPU6050_DisableBypassMode(hgy87);

    /* Enable I2C Master mode */
    MPU6050_EnableI2CMasterMode(hgy87);

    /* Set Master clock */
    MPU6050_SetMasterClock(hgy87);

    /* Configure slave QMC5883L magnetometer in MPU6050 */
    MPU6050_Configure_QMC5883l(hgy87);

    //    /* Configure slave BMP180 barometer in MPU6050 */
    //    MPU6050_Configure_BMP180(hgy87);

    return true;
}

static bool_t QMC5883L_TestConnection(GY87_HandleTypeDef_t * hgy87) {

    /* Test QMC5883L magnetometer connection */
    uint8_t regData;

    MPU6050_ReadRegister(hgy87->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CHIP_ID, &regData, sizeof(regData));

    if (QMC5883L_BIT_CHIP_ID != regData) {
        return false;
    } else {
        return true;
    }
}

static bool_t BMP180_TestConnection(GY87_HandleTypeDef_t * hgy87) {

    /* Test BMP180 barometer connection */
    uint8_t regData;

    MPU6050_ReadRegister(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR << 1, 0xD0, &regData, sizeof(regData));

    if (0x55 != regData) { // TODO
        return false;
    } else {
        return true;
    }
}

static void QMC5883L_Configure(GY87_HandleTypeDef_t * hgy87) {

    /* Configure QMC5883L magnetometer */
    uint8_t regData;

    /* Reset QMC5883L magnetometer */
    regData = 0b00000001;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_RESET, &regData, QMC5883L_SET_BIT);

    /* Configure QMC5883L magnetometer: Control Register 1 */
    regData = 0b00011101;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CONFIG1, &regData, QMC5883L_SET_BIT);

    /* Configure QMC5883L magnetometer: Control Register 2 */
    regData = 0b00000000;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, QMC5883L_AUX_VAL_I2C_ADDR << 1, QMC5883L_REG_CONFIG2, &regData, QMC5883L_SET_BIT);
}

static void BMP180_Configure(GY87_HandleTypeDef_t * hgy87) {

    /* Configure BMP180 barometer */
    /* Read calibration data */
    BMP180_ReadCallibrationData(hgy87);
}

static void BMP180_ReadCallibrationData(GY87_HandleTypeDef_t * hgy87) {

    uint8_t callibrationData[22] = {0};
    uint16_t startRegisterAddress = 0xAA;

    // HAL_I2C_Mem_Read(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, startRegisterAddress, 1, callibrationData, 22, HAL_MAX_DELAY);
    /* Read calibration data */
    MPU6050_ReadRegister(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, startRegisterAddress, callibrationData, sizeof(callibrationData));

    BMP180_CallibrationData.AC1 = ((callibrationData[0] << 8) | callibrationData[1]);
    BMP180_CallibrationData.AC2 = ((callibrationData[2] << 8) | callibrationData[3]);
    BMP180_CallibrationData.AC3 = ((callibrationData[4] << 8) | callibrationData[5]);
    BMP180_CallibrationData.AC4 = ((callibrationData[6] << 8) | callibrationData[7]);
    BMP180_CallibrationData.AC5 = ((callibrationData[8] << 8) | callibrationData[9]);
    BMP180_CallibrationData.AC6 = ((callibrationData[10] << 8) | callibrationData[11]);
    BMP180_CallibrationData.B1 = ((callibrationData[12] << 8) | callibrationData[13]);
    BMP180_CallibrationData.B2 = ((callibrationData[14] << 8) | callibrationData[15]);
    BMP180_CallibrationData.MB = ((callibrationData[16] << 8) | callibrationData[17]);
    BMP180_CallibrationData.MC = ((callibrationData[18] << 8) | callibrationData[19]);
    BMP180_CallibrationData.MD = ((callibrationData[20] << 8) | callibrationData[21]);
}

static uint32_t BMP180_ReadUncompensatedPressure(GY87_HandleTypeDef_t * hgy87) {

    uint8_t datatowrite = 0x34 + (BMP180_OVERSAMPLING << 6);
    uint8_t Press_RAW[3] = {0};

    HAL_I2C_Mem_Write(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, 0xF4, 1, &datatowrite, 1, 1000);
    // MPU6050_WriteRegisterBitmasked(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR << 1, 0xF4, &regData, BMP180_SET_BIT);

    switch (BMP180_OVERSAMPLING) {
    case (0):
        HAL_Delay(5);
        break;
    case (1):
        HAL_Delay(8);
        break;
    case (2):
        HAL_Delay(14);
        break;
    case (3):
        HAL_Delay(26);
        break;
    }

    HAL_I2C_Mem_Read(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, 0xF6, 1, Press_RAW, 3, 1000);

    return (((Press_RAW[0] << 16) + (Press_RAW[1] << 8) + Press_RAW[2]) >> (8 - BMP180_OVERSAMPLING));
}

static uint32_t BMP180_ReadUncompensatedTemperature(GY87_HandleTypeDef_t * hgy87) {

    uint8_t datatowrite = 0x2E;
    uint8_t Temp_RAW[2] = {0};

    HAL_I2C_Mem_Write(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, 0xF4, 1, &datatowrite, 1, 1000);

    HAL_Delay(5); // wait 4.5 ms

    HAL_I2C_Mem_Read(hgy87->hi2c, BMP180_AUX_VAL_I2C_ADDR, 0xF6, 1, Temp_RAW, 2, 1000);

    return ((Temp_RAW[0] << 8) + Temp_RAW[1]);
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
GY87_HandleTypeDef_t * GY87_Init(I2C_HandleTypeDef * hi2c) {

    /* Check parameter */
    if (NULL == hi2c) {
        return NULL;
    }

    /* Check if driver was already once or twice initialized */
    if (GY87_MAX_NUMBER_INSTANCES == instancesNumber) {
        return NULL;
    }

    /* Create an instance of the MPU6050_IMU device */
    GY87_HandleTypeDef_t * hgy87 = GY87_InstanceInit(hi2c);

    /* Check if instance was successfully created */
    if (NULL != hgy87) {
        /* Instance was successfully created */

        /* Initialize I2C communication */
        if (I2C_Init(hgy87)) {

            /* Initialization was successful */
#ifdef GY87_USE_LOGGING
            LOG((uint8_t *)"MPU6050 IMU detected.\r\n\n", LOG_INFORMATION);
#endif

            /* Configure device */
            GY87_Configure(hgy87);

            instancesNumber++;

            return hgy87;
        } else {

            /* Initialization was unsuccessful */
#ifdef USE_FREERTOS
            /* Free up dynamic allocated memory */
            vPortFree(hgy87->buffer);
            vPortFree(hgy87);
#else
            /* Free up dynamic allocated memory */
            free(hgy87->buffer);
            free(hgy87);
#endif

#ifdef GY87_USE_LOGGING
            LOG((uint8_t *)"GY87 IMU not detected.\r\n\n", LOG_ERROR);
#endif
            return NULL;
        }
    } else {

        /* Instance couldn't be created */
#ifdef GY87_USE_LOGGING
        LOG((uint8_t *)"GY87 IMU couldn't be initialized.\r\n\n", LOG_ERROR);
#endif
        return NULL;
    }
}

void GY87_Reset(GY87_HandleTypeDef_t * hgy87) {

    /* Declare variable for data to write into register */
    uint8_t regData;

    /* Check parameter */
    if (NULL != hgy87) {

        /* Set data to write into register */
        regData = MPU_6050_BIT_PWR_MGMT_1_DEVICE_RESET;

        /* Reset device */
        MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_PWR_MGMT_1, &regData, MPU6050_SET_BIT);
    }
}

void GY87_ReadGyroscope(GY87_HandleTypeDef_t * hgy87, GY87_gyroscopeValues_t * gyroscopeValues) {

    /* Declare variable for raw data */
    uint8_t gyroscopeRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_GYRO_SF_0250;

    /* Check parameters */
    if (NULL != hgy87 && NULL != gyroscopeValues) {

        /* Read gyroscope in axis X */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_XOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeX = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

        /* Read gyroscope in axis Y */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_YOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeY = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

        /* Read gyroscope in axis Z */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_ZOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->gyroscopeZ = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]) / scaleFactor;

    } else {

        /* Wrong parameters */
        gyroscopeValues->gyroscopeX = 0;
        gyroscopeValues->gyroscopeY = 0;
        gyroscopeValues->gyroscopeZ = 0;
    }
}

void GY87_ReadAccelerometer(GY87_HandleTypeDef_t * hgy87, GY87_accelerometerValues_t * accelerometerValues) {

    /* Declare variable for raw data */
    uint8_t accelerometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_ACCEL_SF_02;

    /* Check parameters */
    if (NULL != hgy87 && NULL != accelerometerValues) {

        /* Read accelerometer in axis X */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_XOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerX = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

        /* Read accelerometer in axis Y */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_YOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerY = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

        /* Read accelerometer in axis Z */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_ZOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->accelerometerZ = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]) / scaleFactor;

    } else {
        /* Wrong parameters */
        accelerometerValues->accelerometerX = 0;
        accelerometerValues->accelerometerY = 0;
        accelerometerValues->accelerometerZ = 0;
    }
}

int16_t GY87_ReadTemperatureSensor(GY87_HandleTypeDef_t * hgy87) {

    /* Declare variable for raw data */
    uint8_t temperatureSensorRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_TEMP_SF;

    /* Define variable to offset raw data */
    int16_t offset = MPU_6050_AUX_VAL_TEMP_OFS;

    /* Check parameter */
    if (NULL == hgy87) {
        return 0;
    }

    /* Read temperature sensor */
    MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_TEMP_OUT_H, temperatureSensorRawData, sizeof(uint16_t));

    return ((int16_t)(temperatureSensorRawData[0] << 8 | temperatureSensorRawData[1]) / scaleFactor) + offset;
}

void GY87_ReadMagnetometer(GY87_HandleTypeDef_t * hgy87, GY87_magnetometerValues_t * magnetometerValues) {

    /* Declare variable for raw data */
    uint8_t magnetometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = 1;

    /* Check parameters */
    if (NULL != hgy87 && NULL != magnetometerValues) {

        /* Read magnetometer in axis X */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_00, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerX = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

        /* Read magnetometer in axis Y */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_02, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerY = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

        /* Read magnetometer in axis Z */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_04, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->magnetometerZ = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]) / scaleFactor;

    } else {
        /* Wrong parameters */
        magnetometerValues->magnetometerX = 0;
        magnetometerValues->magnetometerY = 0;
        magnetometerValues->magnetometerZ = 0;
    }
}

float GY87_ReadMagnetometerHeading(GY87_HandleTypeDef_t * hgy87) {

    /* Declare structure to read the magnetometer values */
    GY87_magnetometerValues_t magnetometerValues;

    /* Declare variable for compass heading */
    float heading;

    /* Check parameters */
    if (NULL != hgy87) {

        GY87_ReadMagnetometer(hgy87, &magnetometerValues);

        /* Calculate heading */
        heading = atan2(-magnetometerValues.magnetometerY, -magnetometerValues.magnetometerX) * QMC5883L_RADIANS_TO_DEGREES_CONST + QMC5883L_MAGNETIC_DECLINATION + QMC5883L_CALIBRATION_OFFSET;

        /* Check if heading is within 0 and 360 degrees */
        if (heading < 0) {
            heading += 360;
        }

    } else {

        heading = -1;
    }

    return heading;
}

float GY87_ReadBarometerPressure(GY87_HandleTypeDef_t * hgy87) {

    int32_t pressure;

    int32_t X1;
    int32_t X2;
    int32_t X3;
    int32_t B3;
    uint32_t B4;
    int32_t B5;
    int32_t B6;
    uint32_t B7;

    int32_t UP;
    int32_t UT;

    /* Calculate temperature */
    UT = BMP180_ReadUncompensatedTemperature(hgy87);

    UP = BMP180_ReadUncompensatedPressure(hgy87);
    X1 = ((UT - BMP180_CallibrationData.AC6) * (BMP180_CallibrationData.AC5 / (pow(2, 15))));
    X2 = ((BMP180_CallibrationData.MC * (pow(2, 11))) / (X1 + BMP180_CallibrationData.MD));
    B5 = X1 + X2;
    B6 = B5 - 4000;
    X1 = (BMP180_CallibrationData.B2 * (B6 * B6 / (pow(2, 12)))) / (pow(2, 11));
    X2 = BMP180_CallibrationData.AC2 * B6 / (pow(2, 11));
    X3 = X1 + X2;
    B3 = (((BMP180_CallibrationData.AC1 * 4 + X3) << BMP180_OVERSAMPLING) + 2) / 4;
    X1 = BMP180_CallibrationData.AC3 * B6 / pow(2, 13);
    X2 = (BMP180_CallibrationData.B1 * (B6 * B6 / (pow(2, 12)))) / (pow(2, 16));
    X3 = ((X1 + X2) + 2) / pow(2, 2);
    B4 = BMP180_CallibrationData.AC4 * (unsigned long)(X3 + 32768) / (pow(2, 15));
    B7 = ((unsigned long)UP - B3) * (50000 >> BMP180_OVERSAMPLING);

    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    } else {
        pressure = (B7 / B4) * 2;
    }

    X1 = (pressure / (pow(2, 8))) * (pressure / (pow(2, 8)));
    X1 = (X1 * 3038) / (pow(2, 16));
    X2 = (-7357 * pressure) / (pow(2, 16));

    pressure = pressure + (X1 + X2 + 3791) / (pow(2, 4));

    return pressure;
}

float GY87_ReadBarometerAltitude(GY87_HandleTypeDef_t * hgy87) {

    float pressure;
    float altitude;

    pressure = GY87_ReadBarometerPressure(hgy87);

    altitude = 44330 * (1 - (pow(((float)pressure / (float)BMP180_ATMOSFERIC_PRESSURE), 0.19029495718)));

    return altitude;
}

/* --- End of file ----------------------------------------------------------------------------- */
