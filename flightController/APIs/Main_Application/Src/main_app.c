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
#include "user_settings.h"

#include "LoggingSystem_UAI.h"
#include "FSA8S_driver_UAI.h"
#include "MPU6050_driver_UAI.h"
#include "ESC_UAI.h"
// #include "PowerOnOff_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
// #define MAIN_APP_USE_LOGGING						// Remove comment to allow driver info logging
#define DEFAULT_TASK_DELAY (50)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Creates system start-up tasks like the FlightController_StartUp,
 *         FlightController_OnOffButton and Timer1.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateStartUpTasks(void);

/*
 * @brief  Creates other system tasks.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateTasks(void);

/*
 * @brief  Creates system software timers.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateTimers(void);

/*
 * @brief  Task: Creates other tasks and timers, and initializes the different flight controller
 *         drivers.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_StartUp(void * ptr);

/*
 * @brief  Task: Blinks an on-board-LED.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_HeartbeatLight(void * ptr);

/*
 * @brief  Task: Produces blinking sequences with the 4 flight lights.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_FlightLights(void * ptr);

/*
 * @brief  Task: Reads incoming data from the radio control receiver.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_Read_FSA8S(void * ptr);

/*
 * @brief  Task: Reads incoming data from the IMU.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_Read_GY87(void * ptr);

/*
 * @brief  Task: Set the different motors speeds in the ESCs.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_Write_ESCs(void * ptr);

/*
 * @brief  Task: Reads the on-board on/off button and turns on/off the flight controller.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_OnOffButton(void * ptr);

/*
 * @brief  Task: Reads the flight controller battery level and gives a signal whenever the level
 *               is below a threshold set by the user.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_BatteryLevel(void * ptr);

/* --- Private function callback declarations ---------------------------------------------------*/
/*
 * @brief  Reads the on-board on/off button and turns on/off the flight controller depending on
 *         the time expiration of the system timer.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer1_Callback(TimerHandle_t xTimer);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* Tasks Handles */
static TaskHandle_t FlightController_StartUp_Handle = NULL;
static TaskHandle_t FlightController_HeartbeatLight_Handle = NULL;
static TaskHandle_t FlightController_Read_FSA8S_Handle = NULL;
static TaskHandle_t FlightController_Read_GY87_Handle = NULL;
static TaskHandle_t FlightController_FlightLights_Handle = NULL;
static TaskHandle_t FlightController_Write_ESCs_Handle = NULL;
static TaskHandle_t FlightController_OnOffButton_Handle = NULL;
static TaskHandle_t FlightController_BatteryLevel_Handle = NULL;

/* Timers Handles */
static TimerHandle_t Timer1_Handle = NULL;
bool_t Timer1_running = false;
bool_t FlightController_running = false;

/* Drivers Handle */
static IBUS_HandleTypeDef_t * rc_controller;
static MPU6050_HandleTypeDef_t * hmpu6050;
static ESC_HandleTypeDef_t * hesc;

/* --- Private function implementation --------------------------------------------------------- */
void FreeRTOS_CreateStartUpTasks(void) {

    BaseType_t ret;

    /* Task: FlightController_Startup */
    ret = xTaskCreate(FlightController_StartUp, "FlightController_StartUp", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_StartUp_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_StartUp_Handle == NULL) {
        vTaskDelete(FlightController_StartUp_Handle);
    }

    /* Task: FlightController_OnOffButton */
    ret = xTaskCreate(FlightController_OnOffButton, "FlightController_OnOffButton", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_OnOffButton_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_OnOffButton_Handle == NULL) {
        vTaskDelete(FlightController_OnOffButton_Handle);
    }

    /* Timer: OnOff_Button */
    Timer1_Handle = xTimerCreate("OnOff_Button", 100, pdTRUE, (void *)0, Timer1_Callback);
}

void FreeRTOS_CreateTasks(void) {

    BaseType_t ret;

    /* Task 1: FlightController_HeartbeatLight */
    ret = xTaskCreate(FlightController_HeartbeatLight, "FlightController_HeartbeatLight", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_HeartbeatLight_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_HeartbeatLight_Handle == NULL) {
        vTaskDelete(FlightController_HeartbeatLight_Handle);
    }

    /* Task 2: FlightController_FlightLights */
    ret = xTaskCreate(FlightController_FlightLights, "FlightController_FlightLights", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_FlightLights_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_FlightLights_Handle == NULL) {
        vTaskDelete(FlightController_FlightLights_Handle);
    }

    /* Task 3: FlightController_Read_FSA8S */
    ret = xTaskCreate(FlightController_Read_FSA8S, "FlightController_Read_FSA8S", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_Read_FSA8S_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_FSA8S_Handle == NULL) {
        vTaskDelete(FlightController_Read_FSA8S_Handle);
    }

    /* Task 4: FlightController_Read_GY87 */
    ret = xTaskCreate(FlightController_Read_GY87, "FlightController_Read_GY87", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_Read_GY87_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_GY87_Handle == NULL) {
        vTaskDelete(FlightController_Read_GY87_Handle);
    }

    /* Task 5: FlightController_Write_ESCs */
    ret = xTaskCreate(FlightController_Write_ESCs, "FlightController_Write_ESCs", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_Write_ESCs_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Write_ESCs_Handle == NULL) {
        vTaskDelete(FlightController_Write_ESCs_Handle);
    }

    /* Task 6: FlightController_BatteryLevel */
    ret = xTaskCreate(FlightController_BatteryLevel, "FlightController_BatteryLevel", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 1UL), &FlightController_BatteryLevel_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_BatteryLevel_Handle == NULL) {
        vTaskDelete(FlightController_BatteryLevel_Handle);
    }
}

void FreeRTOS_CreateTimers(void) {
}

void FlightController_StartUp(void * ptr) {

    while (1) {
        /* Check if flight controller is already running */
        if (FlightController_running) {

            /* Create system tasks */
            FreeRTOS_CreateTasks();

            /* Create system timers */
            FreeRTOS_CreateTimers();

            /* Initialize drivers */
            rc_controller = FSA8S_Init(&huart2);
            hmpu6050 = MPU6050_Init(&hi2c1);
            hesc = ESC_Init(&htim3);

            /* Delete this task, as initialization must happen only once */
            vTaskDelete(FlightController_StartUp_Handle);
        }
    }
}

void FlightController_HeartbeatLight(void * ptr) {

    uint8_t ledState = GPIO_PIN_RESET;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(HEARTBEAT_PERIOD / 2);

    while (1) {

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);

        /* Change pin state */
        if (ledState == GPIO_PIN_RESET) {

            ledState = GPIO_PIN_SET;
        } else {

            ledState = GPIO_PIN_RESET;
        }

        /* Set time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_FlightLights(void * ptr) {

    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        /* Demo */
        //    	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        //    	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        //    	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        //    	HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

        vTaskDelay(xDelay);
    }
}

void FlightController_Read_FSA8S(void * ptr) {

    uint8_t channel = CHANNEL_9;
    uint16_t channelValue;
    uint8_t str[20];

    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * 10);

    while (1) {

        /* Demo */
        //    	channelValue = FSA8S_ReadChannel(rc_controller, channel);
        //    	sprintf((char *)str, (const char *)"Channel %d: %d\r\n", channel, channelValue);
        //    	CDC_Transmit_FS(str, strlen((const char *)str));

        vTaskDelay(xDelay);
    }
}

void FlightController_Read_GY87(void * ptr) {

    uint8_t str1[50];
    uint8_t str2[50];
    uint8_t str3[50];
    uint8_t str4[50];
    gyroscopeValues_t * gyroscopeValues;
    accelerometerValues_t * accelerometerValues;
    uint16_t tempVal;
    magnetometerValues_t * magnetometerValues;

    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

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
        tempVal = MPU6050_ReadTemperatureSensor(hmpu6050);
        sprintf((char *)str3, (const char *)"Value Temperature: %d\r\n\n", tempVal);
        LOG(str3, LOG_DEBUGGING);
        //
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

        vTaskDelay(xDelay);
    }
}

void FlightController_Write_ESCs(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        //        ESC_SetSpeed(hesc, hesc->channel3, channel_test/10);

        vTaskDelay(xDelay);
    }
}

void FlightController_OnOffButton(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        /* Check On/Off Button status */
        if (!HAL_GPIO_ReadPin(PW_ON_OFF_DRIVER_INPUT_GPIO_Port, PW_ON_OFF_DRIVER_INPUT_Pin)) {
            /* User is trying to turn it on or off */
            if (!Timer1_running) {

                xTimerStart(Timer1_Handle, 0);
                Timer1_running = true;
            }
        }

        /* Set time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_BatteryLevel(void * ptr) {

#ifdef MAIN_APP_USE_LOGGING
    uint8_t loggingStr[50];
#endif

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * 10);
    uint16_t adcValue;
    double batteryLevel = 3.3;

    while (1) {

        // Start ADC Conversion
        HAL_ADC_Start(&hadc1);

        // Poll ADC peripheral
        HAL_ADC_PollForConversion(&hadc1, 1);

        // Read ADC value
        adcValue = HAL_ADC_GetValue(&hadc1);

        // Convert ADC value to real value
        batteryLevel = (adcValue * 3.3) / 4096;

        // Correct real value, as when battery full, ADC input is not 3.3V
        batteryLevel = batteryLevel * 1.046046;

        // Map real value to battery levels
        batteryLevel = batteryLevel * 3.363636;

#ifdef MAIN_APP_USE_LOGGING
        sprintf((char *)loggingStr, "Battery Level: %.2f[V]\r\n\n", batteryLevel);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Set time delay */
        vTaskDelay(xDelay);
    }
}

/* --- Private callback function implementation ------------------------------------------------ */
void Timer1_Callback(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (PW_ON_OFF_DRIVER_TIME / xTimerPeriod)) {
        /* Check if On/Off Button is still pressed after 3 seconds */
        if (!HAL_GPIO_ReadPin(PW_ON_OFF_DRIVER_INPUT_GPIO_Port, PW_ON_OFF_DRIVER_INPUT_Pin)) {

            if (!FlightController_running) {
                /* Flight controller was off */
                /* User turned it on */
                /* Turn on flight controller */
                HAL_GPIO_WritePin(PW_ON_OFF_DRIVER_OUTPUT_GPIO_Port, PW_ON_OFF_DRIVER_OUTPUT_Pin, 1);

                FlightController_running = true;
            } else {
                /* Flight controller was on */
                /* User turned it off */
                /* Suspend HeartbeatLight task and turn LED on */
                vTaskSuspend(FlightController_HeartbeatLight_Handle);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

                FlightController_running = false;

                /* Turn off flight controller */
                HAL_GPIO_WritePin(PW_ON_OFF_DRIVER_OUTPUT_GPIO_Port, PW_ON_OFF_DRIVER_OUTPUT_Pin, 0);
            }
        }

        /* Stop Timer1 */
        xTimerStop(xTimer, 0);
        vTimerSetTimerID(xTimer, (void *)0);

        /* Reset running flag */
        Timer1_running = false;
    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    /* Welcome message */
    LOG((uint8_t *)"Initializing Flight Controller...\r\n\n", LOG_INFORMATION);

    /* Create start-up tasks and timers */
    FreeRTOS_CreateStartUpTasks();
}

/* --- End of file ----------------------------------------------------------------------------- */
