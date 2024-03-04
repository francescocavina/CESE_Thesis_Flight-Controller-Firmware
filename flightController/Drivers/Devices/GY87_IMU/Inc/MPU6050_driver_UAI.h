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
typedef struct gyroscopeValues {
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;
} GY87_gyroscopeValues_t;

typedef struct accelerometerValues {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
} GY87_accelerometerValues_t;

typedef struct magnetometerValues {
    int16_t magnetometerX;
    int16_t magnetometerY;
    int16_t magnetometerZ;
} GY87_magnetometerValues_t;

typedef struct BMP180_CallibrationData {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} BMP180_CallibrationData_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Initializes MPU6050 device.
 * @param  hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *               the configuration information for the I2C communication.
 * @retval Pointer to a GY87_HandleTypeDef_t structure that contains
 *         the configuration information for the GY87 device.
 */
GY87_HandleTypeDef_t * GY87_Init(I2C_HandleTypeDef * hi2c);

/*
 * @brief  Resets the device.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval None
 */
void GY87_Reset(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Reads gyroscope values.
 * @param  hgy87:           Pointer to a GY87_HandleTypeDef_t structure that contains
 *                          the configuration information for the GY87 device.
 *         gyroscopeValues: Pointer to a GY87_gyroscopeValues_t structure that stores the gyroscope values.
 * @retval None
 */
void GY87_ReadGyroscope(GY87_HandleTypeDef_t * hgy87, GY87_gyroscopeValues_t * gyroscopeValues);

/*
 * @brief  Reads accelerometer values.
 * @param  hgy87:               Pointer to a GY87_HandleTypeDef_t structure that contains
 *                              the configuration information for the GY87 device.
 *         accelerometerValues: Pointer to a GY87_accelerometerValues_t structure that stores the accelerometer values.
 * @retval None
 */
void GY87_ReadAccelerometer(GY87_HandleTypeDef_t * hgy87, GY87_accelerometerValues_t * accelerometerValues);

/*
 * @brief  Reads temperature sensor value.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Temperature in degrees Celsius [°C].
 */
int16_t GY87_ReadTemperatureSensor(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Reads magnetometer values.
 * @param  hgy87:              Pointer to a GY87_HandleTypeDef_t structure that contains
 *                             the configuration information for the GY87 device.
 *         magnetometerValues: Pointer to a GY87_magnetometerValues_t structure that stores the magnetometer values.
 * @retval None
 */
void GY87_ReadMagnetometer(GY87_HandleTypeDef_t * hgy87, GY87_magnetometerValues_t * magnetometerValues);

/*
 * @brief  Reads magnetometer heading.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Magnetometer heading in degrees [°].
 */
float GY87_ReadMagnetometerHeading(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Reads barometer pressure value.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Barometer pressure in Pascals [Pa].
 */
float GY87_ReadBarometerPressure(GY87_HandleTypeDef_t * hgy87);

/*
 * @brief  Reads barometer altitude value.
 * @param  hgy87: Pointer to a GY87_HandleTypeDef_t structure that contains
 *                the configuration information for the GY87 device.
 * @retval Altitude in meters [m].
 */
float GY87_ReadBarometerAltitude(GY87_HandleTypeDef_t * hgy87);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_DRIVER_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
