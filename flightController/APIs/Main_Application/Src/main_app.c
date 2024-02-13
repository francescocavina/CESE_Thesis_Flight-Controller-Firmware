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
#define USE_FREERTOS                  // Remove comment when using FreeRTOS
#define LOGGING_TASK_DELAY_MULTIPLIER (20)
// #define MAIN_APP_USE_LOGGING_FSA8S							// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_GYROSCOPE // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_TEMPERATURE					// Remove comment to allow driver info logging
#define MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL	// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING									// Remove comment to allow driver info logging
#define DEFAULT_TASK_DELAY (20)
#define FSA8S_CHANNELS     (10) // Number of remote control channels to read

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
static bool_t Timer1_running = false;
static bool_t FlightController_running = false;

/* Drivers Handle */
static IBUS_HandleTypeDef_t * rc_controller = NULL;
static MPU6050_HandleTypeDef_t * hmpu6050 = NULL;
static ESC_HandleTypeDef_t * hesc = NULL;

/* Remote Control Channel Values */
static uint16_t FSA8S_channelValues[FSA8S_CHANNELS] = {0};

/* IMU Sensors Values */
static int16_t GY87_temperature = 0;
static gyroscopeValues_t * GY87_gyroscopeValues;
static accelerometerValues_t * GY87_accelerometerValues;
static magnetometerValues_t * GY87_magnetometerValues;

/* Flight Controller Battery Level */
static float FlightController_batteryLevel;

/* --- Private function implementation --------------------------------------------------------- */
void FreeRTOS_CreateStartUpTasks(void) {

    BaseType_t ret;

    /* Task: FlightController_Startup */
    ret = xTaskCreate(FlightController_StartUp, "FlightController_StartUp", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 2UL), &FlightController_StartUp_Handle);

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
    ret = xTaskCreate(FlightController_Read_FSA8S, "FlightController_Read_FSA8S", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Read_FSA8S_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_FSA8S_Handle == NULL) {
        vTaskDelete(FlightController_Read_FSA8S_Handle);
    }

    /* Task 4: FlightController_Read_GY87 */
    ret = xTaskCreate(FlightController_Read_GY87, "FlightController_Read_GY87", (6 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Read_GY87_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_GY87_Handle == NULL) {
        vTaskDelete(FlightController_Read_GY87_Handle);
    }

    /* Task 5: FlightController_Write_ESCs */
    ret = xTaskCreate(FlightController_Write_ESCs, "FlightController_Write_ESCs", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Write_ESCs_Handle);

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

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        /* Turn on-board LED on */
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

        /* Check if flight controller is already running */
        /* Create tasks and timers, and initialize drivers (only once) */
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

        /* Set task time delay */
        vTaskDelay(xDelay);
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

        /* Set task time delay */
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

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_Read_FSA8S(void * ptr) {

    FSA8S_CHANNEL_t channels[FSA8S_CHANNELS] = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10};

#ifdef MAIN_APP_USE_LOGGING_FSA8S
    uint8_t loggingStr[30] = {0};
#endif

    /* Change delay from time in [ms] to ticks */
#ifdef MAIN_APP_USE_LOGGING_FSA8S
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    while (1) {

        for (uint8_t i = 0; i < FSA8S_CHANNELS; i++) {
            /* Read channels */
            FSA8S_channelValues[i] = FSA8S_ReadChannel(rc_controller, channels[i]);

            /* Log channel values */
#ifdef MAIN_APP_USE_LOGGING_FSA8S
            if (9 < (i + 1)) {
                sprintf((char *)loggingStr, "FSA8S Channel %d: %04d\r\n\n", channels[i], FSA8S_channelValues[i]);
            } else {
                sprintf((char *)loggingStr, "FSA8S Channel %d:  %04d\r\n", channels[i], FSA8S_channelValues[i]);
            }
            LOG(loggingStr, LOG_INFORMATION);
#endif
        }

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_Read_GY87(void * ptr) {

#if defined MAIN_APP_USE_LOGGING_GY87_GYROSCOPE || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER || defined MAIN_APP_USE_LOGGING_GY87_TEMPERATURE || defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER
    uint8_t loggingStr[40];
#endif

    /* Change delay from time in [ms] to ticks */
#if defined MAIN_APP_USE_LOGGING_GY87_GYROSCOPE || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER || defined MAIN_APP_USE_LOGGING_GY87_TEMPERATURE || defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    /* Allocate dynamic memory for the MPU6050 gyroscope values */
    GY87_gyroscopeValues = pvPortMalloc(sizeof(GY87_gyroscopeValues));

    if (NULL == GY87_gyroscopeValues) {

        /* Free up dynamic allocated memory */
        vPortFree(GY87_gyroscopeValues);
    }

    /* Allocate dynamic memory for the MPU6050 accelerometer values */
    GY87_accelerometerValues = pvPortMalloc(sizeof(GY87_accelerometerValues));

    if (NULL == GY87_accelerometerValues) {

        /* Free up dynamic allocated memory */
        vPortFree(GY87_accelerometerValues);
    }

    /* Allocate dynamic memory for the MPU6050 magnetometer values */
    GY87_magnetometerValues = pvPortMalloc(sizeof(GY87_magnetometerValues));

    if (NULL == GY87_magnetometerValues) {

        /* Free up dynamic allocated memory */
        vPortFree(GY87_magnetometerValues);
    }

    while (1) {

        /* Read GY87 gyroscope values */
        MPU6050_ReadGyroscope(hmpu6050, GY87_gyroscopeValues);

        /* Log GY87 gyroscope values */
#ifdef MAIN_APP_USE_LOGGING_GY87_GYROSCOPE
        sprintf((char *)loggingStr, (const char *)"GY87 Gyroscope X: %d\r\n", GY87_gyroscopeValues->gyroscopeX);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Gyroscope Y: %d\r\n", GY87_gyroscopeValues->gyroscopeY);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Gyroscope Z: %d\r\n\n", GY87_gyroscopeValues->gyroscopeZ);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Read GY87 accelerometer values */
        MPU6050_ReadAccelerometer(hmpu6050, GY87_accelerometerValues);

        /* Log GY87 accelerometer values */
#ifdef MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER
        sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer X: %d\r\n", GY87_accelerometerValues->accelerometerX);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer Y: %d\r\n", GY87_accelerometerValues->accelerometerY);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer Z: %d\r\n\n", GY87_accelerometerValues->accelerometerZ);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Read GY87 temperature value */
        GY87_temperature = MPU6050_ReadTemperatureSensor(hmpu6050);

        /*  Log GY87 temperature value */
#ifdef MAIN_APP_USE_LOGGING_GY87_TEMPERATURE
        sprintf((char *)loggingStr, (const char *)"GY87 Temperature: %d°C\r\n\n", GY87_temperature);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Read GY87 magnetometer values */
        MPU6050_ReadMagnetometer(hmpu6050, GY87_magnetometerValues);

        /* Log GY87 magnetometer values */
#ifdef MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER
        sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer X: %d\r\n", GY87_magnetometerValues->magnetometerX);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer Y: %d\r\n", GY87_magnetometerValues->magnetometerY);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer Z: %d\r\n\n", GY87_magnetometerValues->magnetometerZ);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_Write_ESCs(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        //    	ESC_SetSpeed(hesc, hesc->channel3, channel_test/10);

        /* Set task time delay */
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

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_BatteryLevel(void * ptr) {

    uint16_t adcValue;

#ifdef MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL
    uint8_t loggingStr[20];
#endif

    /* Change delay from time in [ms] to ticks */
#ifdef MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    while (1) {

        /* Start ADC Conversion */
        HAL_ADC_Start(&hadc1);

        /* Poll ADC peripheral */
        HAL_ADC_PollForConversion(&hadc1, 1);

        /* Read ADC value */
        adcValue = HAL_ADC_GetValue(&hadc1);

        /* Convert ADC value to real value */
        FlightController_batteryLevel = (adcValue * 3.3) / 4096;

        /* Correct real value, as when battery full, ADC input is not 3.3V */
        FlightController_batteryLevel = FlightController_batteryLevel * 1.046046;

        /* Map real value to battery levels */
        FlightController_batteryLevel = FlightController_batteryLevel * 3.363636;

        /* Log battery level */
#ifdef MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL
        sprintf((char *)loggingStr, (const char *)"Battery Level: %.2f[V]\r\n\n", FlightController_batteryLevel);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Set task time delay */
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
                /* Suspend HeartbeatLight task and turn on-board LED on */
                vTaskSuspend(FlightController_HeartbeatLight_Handle);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

                FlightController_running = false;

                /* Turn off flight controller */
                HAL_GPIO_WritePin(PW_ON_OFF_DRIVER_OUTPUT_GPIO_Port, PW_ON_OFF_DRIVER_OUTPUT_Pin, 0);

                /* Next line will execute only if USB power is connected */
                /* Reset micro-controller */
                HAL_NVIC_SystemReset();
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
