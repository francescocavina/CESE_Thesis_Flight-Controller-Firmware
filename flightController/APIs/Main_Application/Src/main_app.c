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
// #include "MPU6050_driver_UAI.h"
#include "LoggingSystem_UAI.h"
// #include "ESC_UAI.h"
// #include "PowerOnOff_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
#define HEARTBEAT_DELAY    (200)
#define DEFAULT_TASK_DELAY (50)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */
// extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
// extern TIM_HandleTypeDef htim3;

/* Tasks Handle */
static TaskHandle_t FlightController_Init_Handle = NULL;
static TaskHandle_t FlightController_HeartbeatLight_Handle = NULL;

/* --- Private function implementation --------------------------------------------------------- */
void FlightController_HeartbeatLight() {

    uint8_t ledState = GPIO_PIN_RESET;
    const TickType_t xDelay = pdMS_TO_TICKS(HEARTBEAT_DELAY);

    while (1) {

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);

        if (ledState == GPIO_PIN_RESET) {

            ledState = GPIO_PIN_SET;
        } else {

            ledState = GPIO_PIN_RESET;
        }

        vTaskDelay(xDelay);
    }
}

void FlightController_Init() {

    /* Radio Control Demo */
    static IBUS_HandleTypeDef_t * rc_controller;
    uint8_t channel = CHANNEL_1;
    static uint16_t channelValue;
    uint8_t str[20];

    rc_controller = FSA8S_Init(&huart2);

    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        // LOG((uint8_t *)"Initializing Flight Controller...\r\n\n", LOG_INFORMATION);

        channelValue = FSA8S_ReadChannel(rc_controller, channel);
        sprintf((char *)str, (const char *)"Channel %d: %d\r\n", channel, channelValue);
        CDC_Transmit_FS(str, strlen((const char *)str));

        vTaskDelay(xDelay);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FreeRTOS_CreateTasks() {

    BaseType_t ret;

    /* Task 1: FlightController_HeartbeatLight */
    ret = xTaskCreate(FlightController_HeartbeatLight, "FlightController_HeartbeatLight", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_HeartbeatLight_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    /* Task 2: FlightController_Init */
    ret = xTaskCreate(FlightController_Init, "FlightController_Init", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_Init_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Init_Handle == NULL) {
        vTaskDelete(FlightController_Init_Handle);
    }
}

void flightController_App(void) {

    //    /* Radio Control Demo */
    //    static IBUS_HandleTypeDef_t * rc_controller;
    //    uint8_t channel = CHANNEL_1;
    //    static uint16_t channelValue;
    //    uint8_t str[20];
    //
    //    rc_controller = FSA8S_Init(&huart2);
    //
    //    while (1) {
    //        channelValue = FSA8S_ReadChannel(rc_controller, channel);
    //        sprintf((char *)str, (const char *)"Channel %d: %d\r\n", channel, channelValue);
    //        CDC_Transmit_FS(str, strlen((const char *)str));
    //    }

    while (1)
        ;

    //    /* IMU Demo */
    //    MPU6050_HandleTypeDef_t * hmpu6050;
    //    HAL_Delay(1000);
    //    hmpu6050 = MPU6050_Init(&hi2c1);
    //    uint8_t str1[50];
    //    uint8_t str2[50];
    //    uint8_t str3[50];
    //    uint8_t str4[50];
    //    gyroscopeValues_t * gyroscopeValues;
    //    accelerometerValues_t * accelerometerValues;
    //    uint16_t tempVal;
    //    magnetometerValues_t * magnetometerValues;

    //    while (1) {
    //        HAL_Delay(1000);

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

    //        MPU6050_ReadMagnetometer(hmpu6050, magnetometerValues);
    //        sprintf((char *)str4, (const char *)"Value Mag X: %d\r\n", magnetometerValues->magnetometerX);
    //        LOG(str4, LOG_DEBUGGING);
    //        HAL_Delay(10);
    //        sprintf((char *)str4, (const char *)"Value Mag Y: %d\r\n", magnetometerValues->magnetometerY);
    //        LOG(str4, LOG_DEBUGGING);
    //        HAL_Delay(10);
    //        sprintf((char *)str4, (const char *)"Value Mag Z: %d\r\n\n", magnetometerValues->magnetometerZ);
    //        LOG(str4, LOG_DEBUGGING);
    //        HAL_Delay(10);
    //
    //        int16_t heading = MPU6050_ReadMagnetometerHeading(hmpu6050);
    //        sprintf((char *)str4, (const char *)"Heading Value: %d\r\n\n", heading);
    //        LOG(str4, LOG_DEBUGGING);
    //    }

    //    /* Logging System Demo */
    //    uint8_t str1[40];
    //
    //    while(1) {
    //    	strcpy((char *) str1, "Initializing Flight Controller\r\n");
    //    	LOG((uint8_t *) "Initializing Flight Controller\r\n", LOG_ERROR);
    //    	HAL_Delay(1000);
    //    }

    //    ESC_HandleTypeDef_t * hesc;
    //
    //    // Delay before start
    //    HAL_Delay(1000);
    //
    //    LOG((uint8_t *)"Initializing Flight Controller...\r\n\n", LOG_INFORMATION);
    //    HAL_Delay(50);
    //
    //    hesc = ESC_Init(&htim3);
    //
    //    LOG((uint8_t *)"Flight Controller initialized.\r\n\n", LOG_INFORMATION);
    //    HAL_Delay(50);
    //
    //    HAL_Delay(3000);
    //    ESC_SetSpeed(hesc, hesc->channel1, 100);
    //    ESC_SetSpeed(hesc, hesc->channel2, 100);
    //    ESC_SetSpeed(hesc, hesc->channel3, 100);
    //    ESC_SetSpeed(hesc, hesc->channel4, 100);

    //    while (1) {
    //    }
}

/* --- End of file ----------------------------------------------------------------------------- */
