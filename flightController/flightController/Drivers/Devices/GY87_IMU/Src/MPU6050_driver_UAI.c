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
 * @date:    10/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.5.0
 *
 * @brief:   This is a driver for the GY87 IMU module.
 *           It is divided in three parts: One high level abstraction layer
 *           (MPU6050_driver_UAI.c and MPU6050_driver_UAI.h) for interface with the user
 *           application, one low level abstraction layer (MPU6050_driver_HWI.c and
 *           MPU6050_driver_HWI.h) for interface with the hardware (also known as port)
 *           and register maps (MPU6050_driver_register_map.h and QMC5883L_driver_register_map.h).
 *           In case of need to port this driver to another platform, please only modify the low
 *           layer abstraction layer files where the labels indicate it.
 *
 * @details: In order to be able to use the IMU module, it must be
 *           initialized first. Only two devices can be initialized and therefore
 *           used.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_UAI.h"
#include "MPU6050_driver_register_map.h"
#include "QMC5883L_driver_register_map.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define USE_FREERTOS // Remove comment when using FreeRTOS
// #define GY87_USE_LOGGING            // Remove comment to allow driver info logging

#define GY87_MAX_NUMBER_INSTANCES   (2)    // Maximum number of possible IMUs connected to the i2c bus
#define GY87_CALIBRATION_ITERATIONS (1000) // No. of readings to get a calibration value
#define MPU6050_SET_BIT             (1)
#define MPU6050_CLEAR_BIT           (0)
#define QMC5883L_SET_BIT            (1)
#define QMC5883L_CLEAR_BIT          (0)

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define RADIANS_TO_DEGREES_CONST      (180 / M_PI)
#define QMC5883L_MAGNETIC_DECLINATION (0)  // Magnetic declination [degrees] for Córdoba City 02/15/2024
#define QMC5883L_CALIBRATION_OFFSET   (26) // Calibration offset [degrees]

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;
/* Gyroscope calibration values */
static float gyroscopeCalibrationRoll = 0;
static float gyroscopeCalibrationPitch = 0;
static float gyroscopeCalibrationYaw = 0;
/* Accelerometer calibration values */
static float accelerometerCalibrationX = 0;
static float accelerometerCalibrationY = 0;
static float accelerometerCalibrationZ = 0;

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
 * @brief  Enables the digital low pass filter.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void MPU6050_EnableDLPF(GY87_HandleTypeDef_t * hgy87);

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
 * @brief  Configures QMC5883L magnetometer directly as if MPU6050 was not
 *         interfacing it (MPU6050 must be in bypass mode).
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
static void QMC5883L_Configure(GY87_HandleTypeDef_t * hgy87);

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
        if (buffer)
            vPortFree(buffer);
        if (hgy87)
            vPortFree(hgy87);
#else
        /* Free up dynamic allocated memory */
        if (buffer)
            free(buffer);
        if (hgy87)
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

static void MPU6050_EnableDLPF(GY87_HandleTypeDef_t * hgy87) {

    /* Enable digital low pass filter */
    uint8_t regData;

    regData = MPU_6050_BIT_CONFIG_DLPF_CFG_3;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetGyroscopeRange(GY87_HandleTypeDef_t * hgy87) {

    /* Set gyroscope range */
    uint8_t regData;

    regData = MPU_6050_BIT_GYRO_CONFIG_FS_SEL_1; // Full range
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_SetAccelerometerRange(GY87_HandleTypeDef_t * hgy87) {

    /* Set accelerometer range */
    uint8_t regData;

    regData = MPU_6050_BIT_ACCEL_CONFIG_FS_SEL_2; // Full range
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_CONFIG, &regData, MPU6050_SET_BIT);
}

static void MPU6050_EnableI2CMasterMode(GY87_HandleTypeDef_t * hgy87) {

    /* Enable I2C Master mode */
    uint8_t regData;

    regData = MPU_6050_BIT_USER_CTRL_MST_EN;
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

    regData = MPU_6050_BIT_I2C_MST_CTRL_CLK_13;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_MST_CTRL, &regData, MPU6050_SET_BIT);
}

static void MPU6050_Configure_QMC5883l(GY87_HandleTypeDef_t * hgy87) {

    /* Configure slave QMC5883L magnetometer in MPU6050 */
    uint8_t regData;

    /* Set slave QMC5883L magnetometer device address */
    regData = 0x80 | QMC5883L_AUX_VAL_I2C_ADDR;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_ADDR, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer registers addresses to read */
    regData = QMC5883L_REG_X_LSB;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_REG, &regData, MPU6050_SET_BIT);

    /* Set slave QMC5883L magnetometer number of registers to read*/
    regData = 0x80 | 0x06;
    MPU6050_WriteRegisterBitmasked(hgy87->hi2c, hgy87->address, MPU_6050_REG_I2C_SLV0_CTRL, &regData, MPU6050_SET_BIT);
}

static bool_t GY87_Configure(GY87_HandleTypeDef_t * hgy87) {

    /* Configure MPU6050 device */

    /* Wake up device */
    MPU6050_WakeUpDevice(hgy87);

    /* Set clock source */
    MPU6050_SetClockSource(hgy87);

    /* Set sample rate divider */
    MPU6050_SetSampleDivider(hgy87);

    /* Enable digital low pass filter */
    MPU6050_EnableDLPF(hgy87);

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

    /* Disable Bypass */
    MPU6050_DisableBypassMode(hgy87);

    /* Enable I2C Master mode */
    MPU6050_EnableI2CMasterMode(hgy87);

    /* Set Master clock */
    MPU6050_SetMasterClock(hgy87);

    /* Configure slave QMC5883L magnetometer in MPU6050 */
    MPU6050_Configure_QMC5883l(hgy87);

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
            if (hgy87->buffer)
                vPortFree(hgy87->buffer);
            if (hgy87)
                vPortFree(hgy87);
#else
            /* Free up dynamic allocated memory */
            if (hgy87->buffer)
                free(hgy87->buffer);
            if (hgy87)
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

GY87_gyroscopeCalibrationValues_t GY87_CalibrateGyroscope(GY87_HandleTypeDef_t * hgy87, bool_t fixedCalibration_en) {

    /* Declare structure to read the gyroscope values */
    GY87_gyroscopeValues_t gyroscopeValues;
    /* Declare variable to write the gyroscope calibration values */
    GY87_gyroscopeCalibrationValues_t gyroscopeCalibrationValues;

    /* Declare variables to accumulate measurements */
    float ratesRoll = 0;
    float ratesPitch = 0;
    float ratesYaw = 0;

    /* Check parameter and calculate calibration value */
    if (NULL != hgy87) {
        if (true == fixedCalibration_en) {
            gyroscopeCalibrationValues.fixedCalibration_en = true;
        } else {
            /* Calibrate gyroscope measurements */
            for (int i = 0; i < GY87_CALIBRATION_ITERATIONS; i++) {

                /* Read gyroscope values */
                GY87_ReadGyroscope(hgy87, &gyroscopeValues);

                /* Accumulate measurements */
                ratesRoll += gyroscopeValues.rotationRateRoll;
                ratesPitch += gyroscopeValues.rotationRatePitch;
                ratesYaw += gyroscopeValues.rotationRateYaw;
            }

            gyroscopeCalibrationRoll = ratesRoll / GY87_CALIBRATION_ITERATIONS;
            gyroscopeCalibrationPitch = ratesPitch / GY87_CALIBRATION_ITERATIONS;
            gyroscopeCalibrationYaw = ratesYaw / GY87_CALIBRATION_ITERATIONS;
            gyroscopeCalibrationValues.fixedCalibration_en = false;

#ifdef GY87_USE_LOGGING
            uint8_t loggingStr[120] = {0};
            sprintf((char *)loggingStr, (const char *)"Gyroscope calibration done. CALVAL_ROLL = %.2f, CALVAL_PITCH = %.2f, CALVAL_YAW = %.2f\r\n\n", gyroscopeCalibrationRoll, gyroscopeCalibrationPitch, gyroscopeCalibrationYaw);
            LOG(loggingStr, LOG_INFORMATION);
#endif
        }

        gyroscopeCalibrationValues.calibrationValueRateRoll = gyroscopeCalibrationRoll;
        gyroscopeCalibrationValues.calibrationValueRatePitch = gyroscopeCalibrationPitch;
        gyroscopeCalibrationValues.calibrationValueRateYaw = gyroscopeCalibrationYaw;

    } else {
        gyroscopeCalibrationValues.calibrationValueRateRoll = -1;
        gyroscopeCalibrationValues.calibrationValueRatePitch = -1;
        gyroscopeCalibrationValues.fixedCalibration_en = true;
    }

    return gyroscopeCalibrationValues;
}

void GY87_ReadGyroscope(GY87_HandleTypeDef_t * hgy87, GY87_gyroscopeValues_t * gyroscopeValues) {

    /* Declare variable for raw data */
    uint8_t gyroscopeRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_GYRO_SF_0500;

    /* Check parameters */
    if (NULL != hgy87 && NULL != gyroscopeValues) {

        /* Read gyroscope raw value for X axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_XOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->rawValueX = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]);
        /* Calculate gyroscope rotation rate along X axis (roll) */
        gyroscopeValues->rotationRateRoll = -((float)gyroscopeValues->rawValueX / scaleFactor) - gyroscopeCalibrationRoll;

        /* Read gyroscope raw value for Y axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_YOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->rawValueY = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]);
        /* Calculate gyroscope rotation rate along Y axis (pitch) */
        gyroscopeValues->rotationRatePitch = -((float)gyroscopeValues->rawValueY / scaleFactor) - gyroscopeCalibrationPitch;

        /* Read gyroscope raw value for Z axis  */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_GYRO_ZOUT_H, gyroscopeRawData, sizeof(uint16_t));
        gyroscopeValues->rawValueZ = (int16_t)(gyroscopeRawData[0] << 8 | gyroscopeRawData[1]);
        /* Calculate gyroscope rotation rate along Z axis (yaw)  */
        gyroscopeValues->rotationRateYaw = ((float)gyroscopeValues->rawValueZ / scaleFactor) - gyroscopeCalibrationYaw;

    } else {

        /* Wrong parameters */
        gyroscopeValues->rawValueX = 0;
        gyroscopeValues->rawValueY = 0;
        gyroscopeValues->rawValueZ = 0;
        gyroscopeValues->rotationRateRoll = 0;
        gyroscopeValues->rotationRatePitch = 0;
        gyroscopeValues->rotationRateYaw = 0;
    }
}

GY87_accelerometerCalibrationValues_t GY87_CalibrateAccelerometer(GY87_HandleTypeDef_t * hgy87, bool_t fixedCalibration_en) {

    /* Declare structure to read the accelerometer values */
    GY87_accelerometerValues_t accelerometerValues;
    /* Declare variable to write the gyroscope calibration values */
    GY87_accelerometerCalibrationValues_t accelerometerCalibrationValues;

    /* Declare variables to accumulate measurements */
    float linearAccelerationsX = 0;
    float linearAccelerationsY = 0;
    float linearAccelerationsZ = 0;

    /* Check parameter and calculate calibration value */
    if (NULL != hgy87) {
        if (true == fixedCalibration_en) {
            accelerometerCalibrationValues.fixedCalibration_en = true;
        } else {
            /* Calibrate gyroscope measurements */
            for (int i = 0; i < GY87_CALIBRATION_ITERATIONS; i++) {

                /* Read gyroscope values */
                GY87_ReadAccelerometer(hgy87, &accelerometerValues);

                /* Accumulate measurements */
                linearAccelerationsX += accelerometerValues.linearAccelerationX;
                linearAccelerationsY += accelerometerValues.linearAccelerationY;
                linearAccelerationsZ += (accelerometerValues.linearAccelerationZ - 1);
            }

            accelerometerCalibrationX = linearAccelerationsX / GY87_CALIBRATION_ITERATIONS;
            accelerometerCalibrationY = linearAccelerationsY / GY87_CALIBRATION_ITERATIONS;
            accelerometerCalibrationZ = linearAccelerationsZ / GY87_CALIBRATION_ITERATIONS;
            accelerometerCalibrationValues.fixedCalibration_en = false;

#ifdef GY87_USE_LOGGING
            uint8_t loggingStr[120] = {0};
            sprintf((char *)loggingStr, (const char *)"Accelerometer calibration done. CALVAL_X = %.2f, CALVAL_Y = %.2f, CALVAL_Z = %.2f\r\n\n", accelerometerCalibrationX, accelerometerCalibrationY, accelerometerCalibrationZ);
            LOG((uint8_t *)loggingStr, LOG_INFORMATION);
#endif
        }

        accelerometerCalibrationValues.calibrationValuelinearAccelerationX = accelerometerCalibrationX;
        accelerometerCalibrationValues.calibrationValuelinearAccelerationY = accelerometerCalibrationY;
        accelerometerCalibrationValues.calibrationValuelinearAccelerationZ = accelerometerCalibrationZ;

    } else {
        accelerometerCalibrationValues.calibrationValuelinearAccelerationX = -1;
        accelerometerCalibrationValues.calibrationValuelinearAccelerationY = -1;
        accelerometerCalibrationValues.calibrationValuelinearAccelerationZ = -1;
        accelerometerCalibrationValues.fixedCalibration_en = false;
    }

    return accelerometerCalibrationValues;
}

void GY87_ReadAccelerometer(GY87_HandleTypeDef_t * hgy87, GY87_accelerometerValues_t * accelerometerValues) {

    /* Declare variable for raw data */
    uint8_t accelerometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = MPU_6050_AUX_VAL_ACCEL_FS_08;

    float accX, accY, accZ;
    float denomRoll, denomPitch;

    /* Check parameters */
    if (NULL != hgy87 && NULL != accelerometerValues) {

        /* Read accelerometer raw value for X axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_XOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->rawValueX = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]);
        /* Calculate accelerometer linear acceleration along X axis */
        accX = accelerometerValues->linearAccelerationX = -((float)accelerometerValues->rawValueX / scaleFactor) - accelerometerCalibrationX;

        /* Read accelerometer raw value for Y axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_YOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->rawValueY = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]);
        /* Calculate accelerometer linear acceleration along Y axis */
        accY = accelerometerValues->linearAccelerationY = -((float)accelerometerValues->rawValueY / scaleFactor) - accelerometerCalibrationY;

        /* Read accelerometer raw value for Z axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_ACCEL_ZOUT_H, accelerometerRawData, sizeof(uint16_t));
        accelerometerValues->rawValueZ = (int16_t)(accelerometerRawData[0] << 8 | accelerometerRawData[1]);
        /* Calculate accelerometer linear acceleration along Z axis */
        accZ = accelerometerValues->linearAccelerationZ = ((float)accelerometerValues->rawValueZ / scaleFactor) - accelerometerCalibrationZ;

        /* Calculate roll and pitch angles using an approximation with linear accelerations */
        denomRoll = sqrt(accX * accX + accZ * accZ);
        denomPitch = sqrt(accY * accY + accZ * accZ);

        /* Prevent division by zero */
        if (denomRoll > 0.0001f) {
            accelerometerValues->angleRoll = atan(accY / denomRoll) * RADIANS_TO_DEGREES_CONST;
        } else {
            accelerometerValues->angleRoll = (accY >= 0) ? 90.0f : -90.0f;
        }

        if (denomPitch > 0.0001f) {
            accelerometerValues->anglePitch = -atan(accX / denomPitch) * RADIANS_TO_DEGREES_CONST;
        } else {
            accelerometerValues->anglePitch = (accX >= 0) ? -90.0f : 90.0f;
        }

    } else {
        /* Wrong parameters */
        accelerometerValues->rawValueX = 0;
        accelerometerValues->rawValueY = 0;
        accelerometerValues->rawValueZ = 0;
        accelerometerValues->linearAccelerationX = 0;
        accelerometerValues->linearAccelerationY = 0;
        accelerometerValues->linearAccelerationZ = 0;
        accelerometerValues->angleRoll = 0;
        accelerometerValues->anglePitch = 0;
    }
}

float GY87_ReadTemperatureSensor(GY87_HandleTypeDef_t * hgy87) {

    /* Declare variable for raw data */
    uint8_t temperatureSensorRawData[2];
    int16_t rawTemperature;
    float temperature;

    /* Check parameter */
    if (NULL == hgy87) {
        return 0;
    }

    /* Read temperature sensor */
    MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_TEMP_OUT_H, temperatureSensorRawData, sizeof(uint16_t));

    rawTemperature = (int16_t)((((int16_t)temperatureSensorRawData[0]) << 8) | ((int16_t)temperatureSensorRawData[1]));
    temperature = ((float)rawTemperature / (float)MPU_6050_AUX_VAL_TEMP_SF) + (float)MPU_6050_AUX_VAL_TEMP_OFS;

    /* Sanity check - typical environmental temperature range */
    if (temperature < -5.0f || temperature > 60.0f) {
        return 0;
    } else {
        return temperature;
    }
}

void GY87_ReadMagnetometer(GY87_HandleTypeDef_t * hgy87, GY87_magnetometerValues_t * magnetometerValues) {

    /* Declare variable for raw data */
    uint8_t magnetometerRawData[2];

    /* Define variable for scale factoring raw data */
    int16_t scaleFactor = 4096;

    /* Check parameters */
    if (NULL != hgy87 && NULL != magnetometerValues) {

        /* Read magnetometer raw value for X axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_00, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->rawValueX = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]);
        /* Calculate magnetometer magnetic field along X axis */
        magnetometerValues->magneticFieldX = ((float)magnetometerValues->rawValueX / scaleFactor);

        /* Read magnetometer raw value for Y axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_02, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->rawValueY = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]);
        /* Calculate magnetometer magnetic field along Y axis */
        magnetometerValues->magneticFieldY = ((float)magnetometerValues->rawValueY / scaleFactor);

        /* Read magnetometer raw value for Z axis */
        MPU6050_ReadRegister(hgy87->hi2c, hgy87->address, MPU_6050_REG_EXT_SENS_DATA_04, magnetometerRawData, sizeof(uint16_t));
        magnetometerValues->rawValueZ = (int16_t)(magnetometerRawData[1] << 8 | magnetometerRawData[0]);
        /* Calculate magnetometer magnetic field along Z axis */
        magnetometerValues->magneticFieldZ = ((float)magnetometerValues->rawValueZ / scaleFactor);

    } else {
        /* Wrong parameters */
        magnetometerValues->magneticFieldX = 0;
        magnetometerValues->magneticFieldY = 0;
        magnetometerValues->magneticFieldZ = 0;
    }
}

float GY87_ReadMagnetometerHeading(GY87_HandleTypeDef_t * hgy87) {

    /* Declare structure to read the magnetometer values */
    GY87_magnetometerValues_t magnetometerValues;

    /* Declare variable for compass heading */
    float heading;

    /* Check parameter and calculate heading */
    if (NULL != hgy87) {

        GY87_ReadMagnetometer(hgy87, &magnetometerValues);

        /* Calculate heading */
        heading = atan2(-magnetometerValues.magneticFieldY, -magnetometerValues.magneticFieldX) * RADIANS_TO_DEGREES_CONST + QMC5883L_MAGNETIC_DECLINATION + QMC5883L_CALIBRATION_OFFSET;

        /* Check if heading is within 0 and 360 degrees */
        if (heading < 0) {
            heading += 360;
        }

    } else {

        heading = -1;
    }

    return heading;
}

/* --- End of file ----------------------------------------------------------------------------- */
