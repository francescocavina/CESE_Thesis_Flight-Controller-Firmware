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
 * @file:    main_app.c
 * @date:    23/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is the main application.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "main_app.h"

#include "FSA8S_driver_UAI.h"
#include "MPU6050_driver_UAI.h"
#include "LoggingSystem_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
void flightController_App(void) {

    /* Radio Control Demo */
    //    IBUS_HandleTypeDef_t * rc_controller;
    //    rc_controller = FSA8S_Init(&huart2);
    //    uint16_t channelValue;
    //    uint8_t channel = CHANNEL_1;
    //    uint8_t str0[40];
    //
    //    while (1) {
    //        channelValue = FSA8S_ReadChannel(rc_controller, channel);
    //        sprintf((char *)str0, (const char *)"Channel %d: %d\r\n", channel, channelValue);
    //        CDC_Transmit_FS(str0, strlen((const char *)str0));
    //    }

    /* IMU Demo */
    MPU6050_HandleTypeDef_t * hmpu6050;
    HAL_Delay(1000);
    hmpu6050 = MPU6050_Init(&hi2c1);
    uint8_t str1[50];
    uint8_t str2[50];
    uint8_t str3[50];
    uint8_t str4[50];
    gyroscopeValues_t * gyroscopeValues;
    accelerometerValues_t * accelerometerValues;
    uint16_t tempVal;
    magnetometerValues_t * magnetometerValues;

    while (1) {
        HAL_Delay(1000);

        //        MPU6050_ReadGyroscope(hmpu6050, gyroscopeValues);
        //        sprintf((char *)str1, (const char *)"Value Gyro X: %d\r\n", gyroscopeValues->gyroscopeX);
        //        LOG(str1, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //        sprintf((char *)str1, (const char *)"Value Gyro Y: %d\r\n", gyroscopeValues->gyroscopeY);
        //        LOG(str1, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //        sprintf((char *)str1, (const char *)"Value Gyro Z: %d\r\n\n", gyroscopeValues->gyroscopeZ);
        //        LOG(str1, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //
        //        MPU6050_ReadAccelerometer(hmpu6050, accelerometerValues);
        //        sprintf((char *)str2, (const char *)"Value Accel X: %d\r\n", accelerometerValues->accelerometerX);
        //        LOG(str2, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //        sprintf((char *)str2, (const char *)"Value Accel Y: %d\r\n", accelerometerValues->accelerometerY);
        //        LOG(str2, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //        sprintf((char *)str2, (const char *)"Value Accel Z: %d\r\n\n", accelerometerValues->accelerometerZ);
        //        LOG(str2, LOG_DEBUGGING);
        //        HAL_Delay(10);
        //
        //        tempVal = MPU6050_ReadTemperatureSensor(hmpu6050);
        //        sprintf((char *)str3, (const char *)"Value Temperature: %d\r\n\n", tempVal);
        //        LOG(str3, LOG_DEBUGGING);
        //        HAL_Delay(10);

        MPU6050_ReadMagnetometer(hmpu6050, magnetometerValues);
        sprintf((char *)str4, (const char *)"Value Mag X: %d\r\n", magnetometerValues->magnetometerX);
        LOG(str4, LOG_DEBUGGING);
        HAL_Delay(10);
        sprintf((char *)str4, (const char *)"Value Mag Y: %d\r\n", magnetometerValues->magnetometerY);
        LOG(str4, LOG_DEBUGGING);
        HAL_Delay(10);
        sprintf((char *)str4, (const char *)"Value Mag Z: %d\r\n\n", magnetometerValues->magnetometerZ);
        LOG(str4, LOG_DEBUGGING);
        HAL_Delay(10);

        int16_t heading = MPU6050_ReadMagnetometerHeading(hmpu6050);
        sprintf((char *)str4, (const char *)"Heading Value: %d\r\n\n", heading);
        LOG(str4, LOG_DEBUGGING);
    }

    //    /* Logging System Demo */
    //    uint8_t str1[40];
    //
    //    while(1) {
    //    	strcpy((char *) str1, "Initializing Flight Controller\r\n");
    //    	LOG((uint8_t *) "Initializing Flight Controller\r\n", LOG_ERROR);
    //    	HAL_Delay(1000);
    //    }
}

/* --- End of file ----------------------------------------------------------------------------- */
