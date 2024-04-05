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
 * @date:    27/02/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:   This is the main application that uses FreeRTOS.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "main_app.h"
#include "user_settings.h"

#include "LoggingSystem_UAI.h"
#include "FSA8S_driver_UAI.h"
#include "MPU6050_driver_UAI.h"
#include "ESC_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
#define USE_FREERTOS                  // Remove comment when using FreeRTOS
#define LOGGING_TASK_DELAY_MULTIPLIER (20)
// #define MAIN_APP_USE_LOGGING_CONTROL_SYSTEM					 	// Remove comment to allow driver info logging
#define MAIN_APP_USE_LOGGING_FSA8S // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_GYROSCOPE 					// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES		// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_TEMPERATURE					// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER 				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE 			// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL // Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_ESC								// Remove comment to allow driver info logging
#define DEFAULT_TASK_DELAY (20)
#define FSA8S_CHANNELS     (10) // Number of remote control channels to read

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Flight Controller State */
static bool_t FlightController_running = false;

/* Tasks Handles */
static TaskHandle_t FlightController_StartUp_Handle = NULL;
static TaskHandle_t FlightController_ControlSystem_Handle = NULL;
static TaskHandle_t FlightController_Read_FSA8S_Handle = NULL;
static TaskHandle_t FlightController_Read_GY87_Handle = NULL;
static TaskHandle_t FlightController_Write_ESCs_Handle = NULL;
static TaskHandle_t FlightController_OnOffButton_Handle = NULL;
static TaskHandle_t FlightController_BatteryLevel_Handle = NULL;
static TaskHandle_t FlightController_BatteryAlarm_Handle = NULL;
static TaskHandle_t FlightController_HeartbeatLight_Handle = NULL;
static TaskHandle_t FlightController_FlightLights_Handle = NULL;

/* Timers Handles */
static TimerHandle_t Timer1_Handle = NULL;
static TimerHandle_t Timer2_Handle = NULL;
static TimerHandle_t Timer3_Handle = NULL;

/* Timers Variables */
static bool_t Timer1_running = false;
static bool_t Timer2_flag = false;
static bool_t Timer3_flag = false;
static uint16_t Timer2_AutoReloadTime = 200;
static uint16_t Timer3_AutoReloadTime = 200;

/* Drivers Handle */
static IBUS_HandleTypeDef_t * rc_controller = NULL;
static GY87_HandleTypeDef_t * hgy87 = NULL;
static ESC_HandleTypeDef_t * hesc = NULL;

/* Remote Control Channel Values */
static uint16_t FSA8S_channelValues[FSA8S_CHANNELS] = {0};

/* IMU Sensors Values */
static int16_t GY87_temperature = 0;
static GY87_gyroscopeValues_t * GY87_gyroscopeValues = NULL;
static GY87_accelerometerValues_t * GY87_accelerometerValues = NULL;
static GY87_magnetometerValues_t * GY87_magnetometerValues = NULL;
static float GY87_magnetometerHeadingValue = 0;
static float GY87_barometerTemperatureValue = 0;
static float GY87_barometerPressureValue = 0;
static float GY87_barometerAltitudeValue = 0;

/* ESCs Speed */
static uint16_t ESC_speeds[4] = {0};

/* Flight Controller Battery Level */
static float FlightController_batteryLevelValue = 11.1;

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
 * @brief  Task: Controls the whole system as a closed-loop system, taking as inputs the data
 *               received by the radio controller receiver and the IMU module, and controlling
 *               accordingly the electronic speed controllers.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_ControlSystem(void * ptr);

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

/*
 * @brief  Task: Activates an alarm whenever the battery level is below an user-defined threshold.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_BatteryAlarm(void * ptr);

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

/* --- Private function callback declarations ---------------------------------------------------*/
/*
 * @brief  Reads the on-board on/off button and turns on/off the flight controller depending on
 *         the time expiration of the system timer.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer1_Callback(TimerHandle_t xTimer);

/*
 * @brief  Sets a flag whenever the timer has expired. It is used for the Battery Level Alarm.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer2_Callback(TimerHandle_t xTimer);

/*
 * @brief  Sets a flag whenever the timer has expired. It is used for the Flight Lights.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer3_Callback(TimerHandle_t xTimer);

/* --- Public variable definitions ------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* --- Private variable definitions ------------------------------------------------------------ */

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

    /* Timer1: OnOff_Button */
    Timer1_Handle = xTimerCreate("OnOff_Button", 100, pdTRUE, (void *)0, Timer1_Callback);
}

void FreeRTOS_CreateTasks(void) {

    BaseType_t ret;

    /* Task 1: FlightController_ControlSystem */
    ret = xTaskCreate(FlightController_ControlSystem, "FlightController_ControlSystem", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_ControlSystem_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_ControlSystem_Handle == NULL) {
        vTaskDelete(FlightController_ControlSystem_Handle);
    }

    /* Task 2: FlightController_Read_FSA8S */
    ret = xTaskCreate(FlightController_Read_FSA8S, "FlightController_Read_FSA8S", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Read_FSA8S_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_FSA8S_Handle == NULL) {
        vTaskDelete(FlightController_Read_FSA8S_Handle);
    }

    /* Task 3: FlightController_Read_GY87 */
    ret = xTaskCreate(FlightController_Read_GY87, "FlightController_Read_GY87", (6 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Read_GY87_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Read_GY87_Handle == NULL) {
        vTaskDelete(FlightController_Read_GY87_Handle);
    }

    /* Task 4: FlightController_Write_ESCs */
    ret = xTaskCreate(FlightController_Write_ESCs, "FlightController_Write_ESCs", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_Write_ESCs_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Write_ESCs_Handle == NULL) {
        vTaskDelete(FlightController_Write_ESCs_Handle);
    }

    /* Task 5: FlightController_BatteryLevel */
    ret = xTaskCreate(FlightController_BatteryLevel, "FlightController_BatteryLevel", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_BatteryLevel_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_BatteryLevel_Handle == NULL) {
        vTaskDelete(FlightController_BatteryLevel_Handle);
    }

    /* Task 6: FlightController_BatteryAlarm */
    ret = xTaskCreate(FlightController_BatteryAlarm, "FlightController_BatteryAlarm", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_BatteryAlarm_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_BatteryAlarm_Handle == NULL) {
        vTaskDelete(FlightController_BatteryAlarm_Handle);
    }

    /* Task 7: FlightController_HeartbeatLight */
    ret = xTaskCreate(FlightController_HeartbeatLight, "FlightController_HeartbeatLight", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_HeartbeatLight_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_HeartbeatLight_Handle == NULL) {
        vTaskDelete(FlightController_HeartbeatLight_Handle);
    }

    /* Task 8: FlightController_FlightLights */
    ret = xTaskCreate(FlightController_FlightLights, "FlightController_FlightLights", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3UL), &FlightController_FlightLights_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_FlightLights_Handle == NULL) {
        vTaskDelete(FlightController_FlightLights_Handle);
    }
}

void FreeRTOS_CreateTimers(void) {

    /* Timer2: BatteryLevelAlarm */
    Timer2_Handle = xTimerCreate("BatteryLevelAlarm", 200, pdTRUE, (void *)0, Timer2_Callback);
    if (NULL != Timer2_Handle) {
        /* Start timer */
        xTimerStart(Timer2_Handle, 0);
    }

    /* Timer3: FlightLights */
    Timer3_Handle = xTimerCreate("FlightLights", 100, pdTRUE, (void *)0, Timer3_Callback);
    if (NULL != Timer3_Handle) {
        /* Start timer */
        xTimerStart(Timer3_Handle, 0);
    }
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
            hgy87 = GY87_Init(&hi2c1);
            hesc = ESC_Init(&htim3);

            /* Delete this task, as initialization must happen only once */
            vTaskDelete(FlightController_StartUp_Handle);
        }

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_ControlSystem(void * ptr) {

    //	float rateRoll, ratePitch, rateYaw;
    //	float angleRoll, anglePitch;
    //	float gyroscopeCalibrationRoll, gyroscopeCalibrationPitch, gyroscopeCalibrationYaw;
    //	float linearAccelerationX, linearAcceleretionY, linearAccelerationZ;
    //
    //	float kalmanAngleRoll = 0;
    //	float kalmanUncertaintyAngleRoll = 2 * 2;
    //	float kalmanAnglePitch = 0;
    //	float kalmanUncertaintyAnglePitch = 2 * 2;
    //
    //	float kalman1DOutput[] = {0, 0};
    //
    //	/* 1D Kalman Filter */
    //	  kalmanState = kalmanState + 0.004 * kalmanInput;
    //	  kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    //	  float KalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + 3 * 3);
    //	  kalmanState = KalmanState + KalmanGain * (kalmanMeasurement - kalmanState);
    //	  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;
    //	  kalman1DOutput[0] = kalmanState;
    //	  kalman1DOutput[1] = kalmanUncertainty;

#ifdef MAIN_APP_USE_LOGGING_CONTROL_SYSTEM
    uint8_t loggingStr[40];
#endif

    /* Change delay from time in [ms] to ticks */
#ifdef MAIN_APP_USE_LOGGING_CONTROL_SYSTEM
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    while (1) {

        /* Set ESCs speeds */
        ESC_speeds[0] = FSA8S_channelValues[2] / 10;
        ESC_speeds[1] = FSA8S_channelValues[2] / 10;
        ESC_speeds[2] = FSA8S_channelValues[2] / 10;
        ESC_speeds[3] = FSA8S_channelValues[2] / 10;

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

#if defined MAIN_APP_USE_LOGGING_GY87_GYROSCOPE || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES || defined MAIN_APP_USE_LOGGING_GY87_TEMPERATURE ||                                         \
    defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER || defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING || defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE || defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE ||                       \
    defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE
    uint8_t loggingStr[120];
#endif

    /* Change delay from time in [ms] to ticks */
#if defined MAIN_APP_USE_LOGGING_GY87_GYROSCOPE || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER || defined MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES || defined MAIN_APP_USE_LOGGING_GY87_TEMPERATURE ||                                         \
    defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER || defined MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING || defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE || defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE ||                       \
    defined MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    bool_t gyroscopeCalibrationIsDone = false;
    bool_t accelerometerCalibrationIsDone = false;

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

        /* Check if gyroscope and accelerometer calibrations were done */
        if (gyroscopeCalibrationIsDone && accelerometerCalibrationIsDone) {

            /* Read GY87 gyroscope values */
            GY87_ReadGyroscope(hgy87, GY87_gyroscopeValues);

            /* Log GY87 gyroscope values */
#ifdef MAIN_APP_USE_LOGGING_GY87_GYROSCOPE
            sprintf((char *)loggingStr, (const char *)"GY87 Gyroscope ROLL: %.2f [°/s] PITCH: %.2f [°/s] YAW: %.2f [°/s]\r\n", GY87_gyroscopeValues->rotationRateRoll, GY87_gyroscopeValues->rotationRatePitch, GY87_gyroscopeValues->rotationRateYaw);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 accelerometer values */
            GY87_ReadAccelerometer(hgy87, GY87_accelerometerValues);

            /* Log GY87 accelerometer values */
#ifdef MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER
            sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer X: %.2f [g] Y: %.2f[g] Z: %.2f [g]\r\n", GY87_accelerometerValues->linearAccelerationX, GY87_accelerometerValues->linearAccelerationY,
                    GY87_accelerometerValues->linearAccelerationZ);
            LOG(loggingStr, LOG_INFORMATION);
#endif
#ifdef MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES
            sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer ROLL: %.2f [°] PITCH: %.2f [°]\r\n", GY87_accelerometerValues->angleRoll, GY87_accelerometerValues->anglePitch);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 temperature value */
            GY87_temperature = GY87_ReadTemperatureSensor(hgy87);

            /*  Log GY87 temperature value */
#ifdef MAIN_APP_USE_LOGGING_GY87_TEMPERATURE
            sprintf((char *)loggingStr, (const char *)"GY87 Temperature: %d [°C]\r\n", GY87_temperature);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 magnetometer values */
            GY87_ReadMagnetometer(hgy87, GY87_magnetometerValues);

            /* Log GY87 magnetometer values */
#ifdef MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER
            sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer X: %.3f [G] Y: %.3f [G] Z: %.3f [G]\r\n", GY87_magnetometerValues->magneticFieldX, GY87_magnetometerValues->magneticFieldY, GY87_magnetometerValues->magneticFieldZ);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 magnetometer heading */
            GY87_magnetometerHeadingValue = GY87_ReadMagnetometerHeading(hgy87);

            /* Log GY87 magnetometer heading value */
#ifdef MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING
            sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer Heading: %.2f [°]\r\n", GY87_magnetometerHeadingValue);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 barometer temperature value */
            GY87_barometerTemperatureValue = GY87_ReadBarometerTemperature(hgy87);

            /* Log GY87 barometer pressure value */
#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE
            sprintf((char *)loggingStr, (const char *)"GY87 Barometer Temperature: %.2f [°C]\r\n", GY87_barometerTemperatureValue);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 barometer pressure value */
            GY87_barometerPressureValue = GY87_ReadBarometerPressure(hgy87);

            /* Log GY87 barometer pressure value */
#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE
            sprintf((char *)loggingStr, (const char *)"GY87 Barometer Pressure: %.2f [Pa]\r\n", GY87_barometerPressureValue);
            LOG(loggingStr, LOG_INFORMATION);
#endif

            /* Read GY87 barometer altitude value */
            GY87_barometerAltitudeValue = GY87_ReadBarometerAltitude(hgy87);

            /* Log GY87 barometer altitude value */
#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE
            sprintf((char *)loggingStr, (const char *)"GY87 Barometer Altitude: %.2f [m]\r\n", GY87_barometerAltitudeValue);
            LOG(loggingStr, LOG_INFORMATION);
#endif

        } else {

            /* Calibrate gyroscope measurements */
            if (false == gyroscopeCalibrationIsDone) {

                gyroscopeCalibrationIsDone = GY87_CalibrateGyroscope(hgy87);
            }

            /* Calibrate accelerometer measurements */
            if (false == accelerometerCalibrationIsDone) {

                accelerometerCalibrationIsDone = GY87_CalibrateAccelerometer(hgy87);
            }
        }

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_Write_ESCs(void * ptr) {

#ifdef MAIN_APP_USE_LOGGING_ESC
    uint8_t loggingStr[40];
#endif

    /* Change delay from time in [ms] to ticks */
#ifdef MAIN_APP_USE_LOGGING_ESC
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);
#else
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
#endif

    while (1) {

        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[0]);
        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[1]);
        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[2]);
        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[3]);

#ifdef MAIN_APP_USE_LOGGING_ESC
        sprintf((char *)loggingStr, (const char *)"PWM Channel 1 Speed: %d\r\n", ESC_speeds[0]);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"PWM Channel 2 Speed: %d\r\n", ESC_speeds[1]);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"PWM Channel 3 Speed: %d\r\n", ESC_speeds[2]);
        LOG(loggingStr, LOG_INFORMATION);
        sprintf((char *)loggingStr, (const char *)"PWM Channel 4 Speed: %d\r\n\n", ESC_speeds[3]);
        LOG(loggingStr, LOG_INFORMATION);
#endif

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
        FlightController_batteryLevelValue = (adcValue * 3.3) / 4096;

        /* Correct real value, as when battery full, ADC input is not 3.3V */
        FlightController_batteryLevelValue = FlightController_batteryLevelValue * 1.046046;

        /* Map real value to battery levels */
        FlightController_batteryLevelValue = FlightController_batteryLevelValue * 3.363636;

        /* Log battery level */
#ifdef MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL
        sprintf((char *)loggingStr, (const char *)"Battery Level: %.2f [V]\r\n\n", FlightController_batteryLevel);
        LOG(loggingStr, LOG_INFORMATION);
#endif

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_BatteryAlarm(void * ptr) {

    uint8_t alarmSequence[] = {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t alarmSequenceSize = sizeof(alarmSequence);
    uint8_t alarmSequenceCursor = 0;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        if (FlightController_batteryLevelValue < BATTERY_ALARM_THRESHOLD) {

            if (Timer2_flag) {
                /* If timer expired */

                /* Parse alarm sequence */
                alarmSequenceCursor++;
                if (alarmSequenceSize <= alarmSequenceCursor) {
                    alarmSequenceCursor = 0;
                }

                /* Write to buzzer */
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, alarmSequence[alarmSequenceCursor]);

                /* Reset Timer2 flag */
                Timer2_flag = false;
            }

        } else {

            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
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

    /* Define flight lights sequences */
    uint8_t flightLightsSequenceA1[] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceA3[] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceA2[] = {0, 0, 1, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceA4[] = {0, 0, 1, 0, 0, 0, 0, 0};

    uint8_t flightLightsSequenceB1[] = {1, 0, 1, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceB3[] = {1, 0, 1, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceB2[] = {0, 0, 0, 0, 1, 0, 1, 0};
    uint8_t flightLightsSequenceB4[] = {0, 0, 0, 0, 1, 0, 1, 0};

    uint8_t flightLightsSequenceC1[] = {1, 0, 1, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceC3[] = {1, 0, 1, 0, 0, 0, 0, 0};
    uint8_t flightLightsSequenceC2[] = {0, 0, 0, 0, 1, 0, 0, 0};
    uint8_t flightLightsSequenceC4[] = {0, 0, 0, 0, 1, 0, 0, 0};

    uint8_t flightLightsSequence = 0;
    uint8_t flightLightsSequenceSize = 0;
    uint8_t flightLightsSequenceCursor = 0;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        /* Turn on/off flight lights (Switch D on radio controller) */
        if (500 <= FSA8S_channelValues[9]) {

            /* Set flight light sequence (Switch C on radio controller) */
            if (250 >= FSA8S_channelValues[8]) {

                flightLightsSequence = 0;

            } else if (250 < FSA8S_channelValues[8] && 750 >= FSA8S_channelValues[8]) {

                flightLightsSequence = 1;

            } else if (750 < FSA8S_channelValues[8]) {

                flightLightsSequence = 2;
            }

            /* Set flight light sequence speed (Potentiometer B on radio controller) */
            Timer3_AutoReloadTime = 200 + FSA8S_channelValues[7] / 5;

            /* Check if timer has expired */
            if (Timer3_flag) {

                /* Parse flight lights sequences */
                flightLightsSequenceCursor++;
                if (flightLightsSequenceSize <= flightLightsSequenceCursor) {
                    flightLightsSequenceCursor = 0;
                }

                /* Write to flight lights */
                if (flightLightsSequence == 0) {

                    flightLightsSequenceSize = sizeof(flightLightsSequenceA1);

                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, flightLightsSequenceA1[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, flightLightsSequenceA2[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, flightLightsSequenceA3[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, flightLightsSequenceA4[flightLightsSequenceCursor]);

                } else if (flightLightsSequence == 1) {

                    flightLightsSequenceSize = sizeof(flightLightsSequenceB1);

                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, flightLightsSequenceB1[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, flightLightsSequenceB2[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, flightLightsSequenceB3[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, flightLightsSequenceB4[flightLightsSequenceCursor]);

                } else if (flightLightsSequence == 2) {

                    flightLightsSequenceSize = sizeof(flightLightsSequenceC1);

                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, flightLightsSequenceC1[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, flightLightsSequenceC2[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, flightLightsSequenceC3[flightLightsSequenceCursor]);
                    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, flightLightsSequenceC4[flightLightsSequenceCursor]);
                }

                /* Reset Timer3 flag */
                Timer3_flag = false;
            }

        } else {

            /* Turn off flight lights */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
        }

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

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

        /* Reset running flag */
        Timer1_running = false;
    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer2_Callback(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (Timer2_AutoReloadTime / xTimerPeriod)) {

        /* Set Timer2 flag to true */
        Timer2_flag = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer3_Callback(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (Timer3_AutoReloadTime / xTimerPeriod)) {

        /* Set Timer3 flag to true */
        Timer3_flag = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    /* Welcome message */
    // LOG((uint8_t *)"Initializing Flight Controller...\r\n\n", LOG_INFORMATION);

    /* Create start-up tasks and timers */
    FreeRTOS_CreateStartUpTasks();
}

/* --- End of file ----------------------------------------------------------------------------- */
