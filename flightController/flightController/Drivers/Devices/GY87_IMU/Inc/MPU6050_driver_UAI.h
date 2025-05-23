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
 * @file:    MPU6050_driver_UAI.h
 * @date:    10/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.5.0
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
 * @details: In order to be able to use the IMU module, it must be initialized first.
 * 			 Only two devices can be initialized and therefore used.
 */

#ifndef INC_MPU6050_DRIVER_UAI_H
#define INC_MPU6050_DRIVER_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU6050_driver_HWI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
typedef struct gyroscopeCalibrationValues {
    bool_t calibrationDone;
    bool_t fixedCalibration_en;
    float  calibrationRateRoll;
    float  calibrationRatePitch;
    float  calibrationRateYaw;
    float  stdRateRoll;
    float  stdRatePitch;
    float  stdRateYaw;

} GY87_gyroscopeCalibrationValues_t;

typedef struct gyroscopeValues {
    int16_t rawValueX;
    int16_t rawValueY;
    int16_t rawValueZ;
    float   rotationRateRoll;
    float   rotationRatePitch;
    float   rotationRateYaw;
} GY87_gyroscopeValues_t;

typedef struct accelerometerCalibrationValues {
    bool_t calibrationDone;
    bool_t fixedCalibration_en;
    float  calibrationLinearAccelerationX;
    float  calibrationLinearAccelerationY;
    float  calibrationLinearAccelerationZ;
    float  stdLinearAccelerationX;
    float  stdLinearAccelerationY;
    float  stdLinearAccelerationZ;
    float  stdAngleRoll;
    float  stdAnglePitch;
} GY87_accelerometerCalibrationValues_t;

typedef struct accelerometerValues {
    int16_t rawValueX;
    int16_t rawValueY;
    int16_t rawValueZ;
    float   linearAccelerationX;
    float   linearAccelerationY;
    float   linearAccelerationZ;
    float   angleRoll;
    float   anglePitch;
} GY87_accelerometerValues_t;

typedef struct magnetometerValues {
    int16_t rawValueX;
    int16_t rawValueY;
    int16_t rawValueZ;
    float   magneticFieldX;
    float   magneticFieldY;
    float   magneticFieldZ;
} GY87_magnetometerValues_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Initializes MPU6050 device.
 * @param  hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *               the configuration information for the I2C communication.
 * @retval Pointer to a GY87_HandleTypeDef_t structure that contains
 *         the configuration information for the GY87 device.
 */
GY87_HandleTypeDef_t *GY87_Init(I2C_HandleTypeDef *hi2c);

/*
 * @brief  Resets the device.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
void GY87_Reset(GY87_HandleTypeDef_t *hgy87);

/*
 * @brief  Calibrates gyroscope measurement.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 *         gyroscopeCalibrationValues: Pointer to a GY87_gyroscopeCalibrationValues_t structure that stores the gyroscope calibration values.
 *         fixedCalibration_en:        Enables fixed calibration.
 *         calibrationIterations:      Number of iterations for calibration.
 * @retval true:  Calibration was successful.
 *         false: Calibrations was not successful.
 */
void GY87_CalibrateGyroscope(GY87_HandleTypeDef_t *hgy87, GY87_gyroscopeCalibrationValues_t *gyroscopeCalibrationValues, bool_t fixedCalibration_en, uint16_t calibrationIterations);

/*
 * @brief  Reads gyroscope values.
 * @param  hgy87:           Pointer to a GY87_HandleTypeDef_t structure that contains
 *                          the configuration information for the GY87 device.
 *         gyroscopeValues: Pointer to a GY87_gyroscopeValues_t structure that stores the gyroscope values.
 * @retval None
 */
void GY87_ReadGyroscope(GY87_HandleTypeDef_t *hgy87, GY87_gyroscopeValues_t *gyroscopeValues, GY87_gyroscopeCalibrationValues_t *gyroscopeCalibrationValues);

/*
 * @brief  Calibrates accelerometer measurement.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 *         accelerometerCalibrationValues: Pointer to a GY87_accelerometerCalibrationValues_t structure that stores the accelerometer calibration values.
 *         fixedCalibration_en:            Enables fixed calibration.
 *         calibrationIterations:          Number of iterations for calibration.
 * @retval true:  Calibration was successful.
 *         false: Calibrations was not successful.
 */
void GY87_CalibrateAccelerometer(GY87_HandleTypeDef_t *hgy87, GY87_accelerometerCalibrationValues_t *accelerometerCalibrationValues, bool_t fixedCalibration_en, uint16_t calibrationIterations);

/*
 * @brief  Reads accelerometer values.
 * @param  hgy87:               Pointer to a GY87_HandleTypeDef_t structure that contains
 *                              the configuration information for the GY87 device.
 *         accelerometerValues: Pointer to a GY87_accelerometerValues_t structure that stores the accelerometer values.
 * @retval None
 */
void GY87_ReadAccelerometer(GY87_HandleTypeDef_t *hgy87, GY87_accelerometerValues_t *accelerometerValues, GY87_accelerometerCalibrationValues_t *accelerometerCalibrationValues);

/*
 * @brief  Reads temperature sensor value.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Temperature in degrees Celsius [°C].
 */
float GY87_ReadTemperatureSensor(GY87_HandleTypeDef_t *hgy87);

/*
 * @brief  Reads magnetometer values.
 * @param  hgy87:              Pointer to a GY87_HandleTypeDef_t structure that contains
 *                             the configuration information for the GY87 device.
 *         magnetometerValues: Pointer to a GY87_magnetometerValues_t structure that stores the magnetometer values.
 * @retval None
 */
void GY87_ReadMagnetometer(GY87_HandleTypeDef_t *hgy87, GY87_magnetometerValues_t *magnetometerValues);

/*
 * @brief  Reads magnetometer heading.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Magnetometer heading in degrees [°].
 */
float GY87_ReadMagnetometerHeading(GY87_HandleTypeDef_t *hgy87);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_DRIVER_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
