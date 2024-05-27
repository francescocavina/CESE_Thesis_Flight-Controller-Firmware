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
/* FreeRTOS */
#define USE_FREERTOS                                  // Remove comment when using FreeRTOS
#define DEFAULT_TASK_DELAY                            (20)
#define TASK_FLIGHTCONTROLLER_STARTUP_PRIORITY        (2)
#define TASK_FLIGHTCONTROLLER_CONTROLSYSTEM_PRIORITY  (2)
#define TASK_FLIGHTCONTROLLER_DATA_LOGGING_PRIORITY   (2)
#define TASK_FLIGHTCONTROLLER_ONOFFBUTTON_PRIORITY    (2)
#define TASK_FLIGHTCONTROLLER_BATTERYLEVEL_PRIORITY   (2)
#define TASK_FLIGHTCONTROLLER_BATTERYALARM_PRIORITY   (2)
#define TASK_FLIGHTCONTROLLER_HEARTBEATLIGHT_PRIORITY (2)
#define TASK_FLIGHTCONTROLLER_FLIGHTLIGHTS_PRIORITY   (2)
/* Logging */
#define LOGGING_TASK_DELAY_MULTIPLIER (20)
// #define MAIN_APP_USE_LOGGING_STARTUP 							// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_CONTROL_SYSTEM_MODE0 			// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_CONTROL_SYSTEM_MODE1 			// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_FSA8S 							// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_GYROSCOPE 					// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES		// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_KALMAN_ANGLES				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_TEMPERATURE					// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER 				// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING 		// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE 		// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE 			// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE 			// Remove comment to allow driver info logging
// #define MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL 	// Remove comment to allow driver info logging
/* Drivers */
#define FSA8S_CHANNELS    (10) // Number of remote control channels to read
#define ESC_MAXIMUM_SPEED (100)
#define ESC_MINIMUM_SPEED (10)
/* Control System */
#define CONTROL_SYSTEM_MODE                   (2)
#define CONTROL_SYSTEM_LOOP_PERIOD_MS         (4) // Loop period in [ms]
#define CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE (25)
#define CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE (800)
#define CONTROL_SYSTEM_PID_OUTPUT_LIMIT       (20)
/* Battery Level */
#define BATTERY_LEVEL_CALIBRATION_OFFSET (0.76)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Flight Controller State */
static bool_t FlightController_isRunning = false;
static bool_t FlightController_isInitialized = false;

/* Tasks Handles */
static TaskHandle_t FlightController_StartUp_Handle = NULL;
static TaskHandle_t FlightController_ControlSystem_Handle = NULL;
static TaskHandle_t FlightController_Data_Logging_Handle = NULL;
static TaskHandle_t FlightController_OnOffButton_Handle = NULL;
static TaskHandle_t FlightController_BatteryLevel_Handle = NULL;
static TaskHandle_t FlightController_BatteryAlarm_Handle = NULL;
static TaskHandle_t FlightController_HeartbeatLight_Handle = NULL;
static TaskHandle_t FlightController_FlightLights_Handle = NULL;

/* Timers Handles */
static TimerHandle_t Timer1_Handle = NULL;
static TimerHandle_t Timer2_Handle = NULL;
static TimerHandle_t Timer3_Handle = NULL;
static TimerHandle_t Timer4_Handle = NULL;

/* Timers Variables */
static bool_t Timer1_running = false;
static bool_t Timer2_flag = false;
static bool_t Timer3_flag = false;
static bool_t Timer4_flag = false;
static uint16_t Timer1_AutoReloadTime = PW_ON_OFF_DRIVER_TIME;
static uint16_t Timer2_AutoReloadTime = 200;
static uint16_t Timer3_AutoReloadTime = 200;
static uint16_t Timer4_AutoReloadTime = CONTROL_SYSTEM_LOOP_PERIOD_MS;

/* Drivers Handles */
static IBUS_HandleTypeDef_t * rc_controller = NULL;
static GY87_HandleTypeDef_t * hgy87 = NULL;
static ESC_HandleTypeDef_t * hesc = NULL;

/* FS-A8S Radio Controller Variables */
static FSA8S_CHANNEL_t channels[FSA8S_CHANNELS] = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10};
static uint16_t FSA8S_channelValues[FSA8S_CHANNELS] = {0};

/* GY-87 IMU Variables */
static GY87_gyroscopeValues_t GY87_gyroscopeValues;
static GY87_accelerometerValues_t GY87_accelerometerValues;
static GY87_magnetometerValues_t GY87_magnetometerValues;
static float GY87_magnetometerHeadingValue = 0;
static bool_t gyroscopeCalibrationIsDone = false;
static bool_t accelerometerCalibrationIsDone = false;

/* Control System Mode 0 */
#if 0 == CONTROL_SYSTEM_MODE
/* Throttle stick check */
static bool_t throttleStick_startedDown = false;
/* References */
static float inputValue_throttle = 0;
/* Motors inputs */
static float motorSpeed1 = 0;
static float motorSpeed2 = 0;
static float motorSpeed3 = 0;
static float motorSpeed4 = 0;
/* ESCs */
static bool_t ESC_isEnabled = false;
static float ESC_speeds[5] = {0};
#endif
/* Control System Mode 1 */
#if 1 == CONTROL_SYSTEM_MODE
/* Throttle stick check */
static bool_t throttleStick_startedDown = false;
/* References */
static float inputValue_throttle = 0;
static float inputValue_rollRate = 0;
static float inputValue_pitchRate = 0;
static float inputValue_yawRate = 0;
/* Desired references */
static float desiredValue_rollRate = 0;
static float desiredValue_pitchRate = 0;
static float desiredValue_yawRate = 0;
/* Errors */
static float errorValue_rollRate = 0;
static float errorValue_pitchRate = 0;
static float errorValue_yawRate = 0;
/* Previously stored errors */
static float previousErrorValue_rollRate = 0;
static float previousErrorValue_pitchRate = 0;
static float previousErrorValue_yawRate = 0;
/* Previously stored terms */
static float previousIterm_rollRate = 0;
static float previousIterm_pitchRate = 0;
static float previousIterm_yawRate = 0;
/* PID gains */
static float kP_rollRate = 0.3;
static float kP_pitchRate = 0.3;
static float kP_yawRate = 0.3;
static float kI_rollRate = 0.1;
static float kI_pitchRate = 0.1;
static float kI_yawRate = 0.1;
static float kD_rollRate = 0.0;
static float kD_pitchRate = 0.0;
static float kD_yawRate = 0.0;
/* PID terms */
// static float Pterm_rollRate  = 0;
// static float Pterm_pitchRate = 0;
// static float Pterm_yawRate   = 0;
// static float Iterm_rollRate  = 0;
// static float Iterm_pitchRate = 0;
// static float Iterm_yawRate   = 0;
// static float Dterm_rollRate  = 0;
// static float Dterm_pitchRate = 0;
// static float Dterm_yawRate   = 0;
/* PID outputs */
static float pidOutputValue_rollRate = 0;
static float pidOutputValue_pitchRate = 0;
static float pidOutputValue_yawRate = 0;
/* Motors inputs */
static float motorSpeed1 = 0;
static float motorSpeed2 = 0;
static float motorSpeed3 = 0;
static float motorSpeed4 = 0;
/* ESCs */
static bool_t ESC_isEnabled = false;
static float ESC_speeds[5] = {0};
#endif
/* Control System Mode 2 */
#if 2 == CONTROL_SYSTEM_MODE
/* Throttle stick check */
static bool_t throttleStick_startedDown = false;
/* References: General */
static float inputValue_throttle = 0;
/* References: Angles */
static float inputValue_rollAngle = 0;
static float inputValue_pitchAngle = 0;
/* Kalman prediction measurements */
static float kalmanPredictionValue_rollAngle = 0;
static float kalmanPredictionValue_pitchAngle = 0;
/* Kalman measurements uncertainties */
static float kalmanUncertaintyValue_rollAngle = 2 * 2;
static float kalmanUncertaintyValue_pitchAngle = 2 * 2;
/* Kalman calculatio output */
static float kalmanOutput[2] = {0};
/* Desired references: Angles */
static float desiredValue_rollAngle = 0;
static float desiredValue_pitchAngle = 0;
/* Errors: Angles */
static float errorValue_rollAngle = 0;
static float errorValue_pitchAngle = 0;
/* Previously stored errors: Angles */
static float previousErrorValue_rollAngle = 0;
static float previousErrorValue_pitchAngle = 0;
/* Previously stored terms: Angles */
static float previousIterm_rollAngle = 0;
static float previousIterm_pitchAngle = 0;
/* PID gains: Angles */
static float kP_rollAngle = 0.3;
static float kP_pitchAngle = 0.3;
static float kI_rollAngle = 0;
static float kI_pitchAngle = 0;
static float kD_rollAngle = 0;
static float kD_pitchAngle = 0;
/* PID terms */
// static float Pterm_rollAngle  = 0;
// static float Pterm_pitchAngle = 0;
// static float Iterm_rollAngle  = 0;
// static float Iterm_pitchAngle = 0;
// static float Dterm_rollAngle  = 0;
// static float Dterm_pitchAngle = 0;
/* PID outputs */
static float pidOutputValue_rollAngle = 0;
static float pidOutputValue_pitchAngle = 0;
/* References: Rates */
static float inputValue_rollRate = 0;
static float inputValue_pitchRate = 0;
static float inputValue_yawRate = 0;
/* Desired references: Rates */
static float desiredValue_rollRate = 0;
static float desiredValue_pitchRate = 0;
static float desiredValue_yawRate = 0;
/* Errors: Rates */
static float errorValue_rollRate = 0;
static float errorValue_pitchRate = 0;
static float errorValue_yawRate = 0;
/* Previously stored errors: Rates */
static float previousErrorValue_rollRate = 0;
static float previousErrorValue_pitchRate = 0;
static float previousErrorValue_yawRate = 0;
/* Previously stored terms: Rates  */
static float previousIterm_rollRate = 0;
static float previousIterm_pitchRate = 0;
static float previousIterm_yawRate = 0;
/* PID gains: Rates */
static float kP_rollRate = 0.1;
static float kP_pitchRate = 0.1;
static float kP_yawRate = 0.1;
static float kI_rollRate = 0.0;
static float kI_pitchRate = 0.0;
static float kI_yawRate = 0.0;
static float kD_rollRate = 0.0;
static float kD_pitchRate = 0.0;
static float kD_yawRate = 0.0;
/* PID terms */
// static float Pterm_rollRate  = 0;
// static float Pterm_pitchRate = 0;
// static float Pterm_yawRate   = 0;
// static float Iterm_rollRate  = 0;
// static float Iterm_pitchRate = 0;
// static float Iterm_yawRate   = 0;
// static float Dterm_rollRate  = 0;
// static float Dterm_pitchRate = 0;
// static float Dterm_yawRate   = 0;
/* PID outputs */
static float pidOutputValue_rollRate = 0;
static float pidOutputValue_pitchRate = 0;
static float pidOutputValue_yawRate = 0;
/* Motors inputs */
static float motorSpeed1 = 0;
static float motorSpeed2 = 0;
static float motorSpeed3 = 0;
static float motorSpeed4 = 0;
/* ESCs */
static bool_t ESC_isEnabled = false;
static float ESC_speeds[5] = {0};
#endif

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
 * @brief  Task: Logs flight controller data.
 * @param  Task pointer: not used.
 * @retval None
 */
void FlightController_Data_Logging(void * ptr);

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
 * @brief  Timer Callback: Reads the on-board on/off button and turns on/off the flight controller
 * 		                   depending on the time expiration of the system timer.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer1_Callback(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: Sets a flag whenever the timer has expired. It is used for the Battery
 *                         Level Alarm.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer2_Callback(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: Sets a flag whenever the timer has expired. It is used for the
 *                         Flight Lights.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer3_Callback(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: TODO
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer4_Callback(TimerHandle_t xTimer);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
void CSM1_CalculatePID(float * PID_Output, float * previousIterm, float * previousErrorValue, float errorValue, float kP, float kI, float kD);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
void CSM1_ResetPID(void);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
void CSM2_CalculateKalman(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement);

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
void CSM2_CalculatePID();

/*
 * @brief  TODO
 * @param  TODO
 * @retval None
 */
void CSM2_ResetPID(void);

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
    ret = xTaskCreate(FlightController_StartUp, "FlightController_StartUp", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_STARTUP_PRIORITY), &FlightController_StartUp_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_StartUp_Handle == NULL) {
        vTaskDelete(FlightController_StartUp_Handle);
    }

    /* Task: FlightController_OnOffButton */
    ret = xTaskCreate(FlightController_OnOffButton, "FlightController_OnOffButton", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_ONOFFBUTTON_PRIORITY), &FlightController_OnOffButton_Handle);

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
    ret = xTaskCreate(FlightController_ControlSystem, "FlightController_ControlSystem", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_CONTROLSYSTEM_PRIORITY), &FlightController_ControlSystem_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_ControlSystem_Handle == NULL) {
        vTaskDelete(FlightController_ControlSystem_Handle);
    }

    /* Task 2: FlightController_Data_Logging */
    ret = xTaskCreate(FlightController_Data_Logging, "FlightController_Data_Logging", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_DATA_LOGGING_PRIORITY), &FlightController_Data_Logging_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_Data_Logging_Handle == NULL) {
        vTaskDelete(FlightController_Data_Logging_Handle);
    }

    /* Task 3: FlightController_BatteryLevel */
    ret = xTaskCreate(FlightController_BatteryLevel, "FlightController_BatteryLevel", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_BATTERYLEVEL_PRIORITY), &FlightController_BatteryLevel_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_BatteryLevel_Handle == NULL) {
        vTaskDelete(FlightController_BatteryLevel_Handle);
    }

    /* Task 4: FlightController_BatteryAlarm */
    ret = xTaskCreate(FlightController_BatteryAlarm, "FlightController_BatteryAlarm", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_BATTERYALARM_PRIORITY), &FlightController_BatteryAlarm_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_BatteryAlarm_Handle == NULL) {
        vTaskDelete(FlightController_BatteryAlarm_Handle);
    }

    /* Task 5: FlightController_HeartbeatLight */
    ret = xTaskCreate(FlightController_HeartbeatLight, "FlightController_HeartbeatLight", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_HEARTBEATLIGHT_PRIORITY), &FlightController_HeartbeatLight_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_HeartbeatLight_Handle == NULL) {
        vTaskDelete(FlightController_HeartbeatLight_Handle);
    }

    /* Task 6: FlightController_FlightLights */
    ret = xTaskCreate(FlightController_FlightLights, "FlightController_FlightLights", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_FLIGHTLIGHTS_PRIORITY), &FlightController_FlightLights_Handle);

    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    if (FlightController_FlightLights_Handle == NULL) {
        vTaskDelete(FlightController_FlightLights_Handle);
    }
}

void FreeRTOS_CreateTimers(void) {

    /* Timer2: BatteryLevelAlarm */
    Timer2_Handle = xTimerCreate("BatteryLevelAlarm", pdMS_TO_TICKS(200), pdTRUE, (void *)0, Timer2_Callback);
    if (NULL != Timer2_Handle) {
        /* Start timer */
        xTimerStart(Timer2_Handle, 0);
    }

    /* Timer3: FlightLights */
    Timer3_Handle = xTimerCreate("FlightLights", pdMS_TO_TICKS(100), pdTRUE, (void *)0, Timer3_Callback);
    if (NULL != Timer3_Handle) {
        /* Start timer */
        xTimerStart(Timer3_Handle, 0);
    }

    /* Timer4: ControlSystem */
    Timer4_Handle = xTimerCreate("ControlSystem", pdMS_TO_TICKS(1), pdTRUE, (void *)0, Timer4_Callback);
    if (NULL != Timer4_Handle) {
        /* Start timer */
        xTimerStart(Timer4_Handle, 0);
    }
}

void FlightController_StartUp(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

    while (1) {

        /* Turn on-board LED off */
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

        /* Check if flight controller is already running */
        /* Create tasks and timers, and initialize drivers (only once) */
        if (FlightController_isRunning) {

            /* Startup message */
#ifdef MAIN_APP_USE_LOGGING_STARTUP
            LOG((uint8_t *)"Initializing Flight Controller...\r\n\n", LOG_INFORMATION);
#endif

            /* Create system tasks */
            FreeRTOS_CreateTasks();

            /* Create system timers */
            FreeRTOS_CreateTimers();

            /* Initialize drivers */
            rc_controller = FSA8S_Init(&huart2);
#ifdef MAIN_APP_USE_LOGGING_STARTUP
            LOG((uint8_t *)"FSA8S Radio Controller Initialized.\r\n\n", LOG_INFORMATION);
#endif

            hgy87 = GY87_Init(&hi2c1);
#ifdef MAIN_APP_USE_LOGGING_STARTUP
            LOG((uint8_t *)"GY-87 IMU Initialized.\r\n\n", LOG_INFORMATION);
#endif

            hesc = ESC_Init(&htim3);
#ifdef MAIN_APP_USE_LOGGING_STARTUP
            LOG((uint8_t *)"ESCs Initialized.\r\n\n", LOG_INFORMATION);
#endif

            /* Delete this task, as initialization must happen only once */
            vTaskDelete(FlightController_StartUp_Handle);
        }

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_ControlSystem(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(1);

    while (1) {

        /* Calibrate GY-87 gyroscope sensor */
        if (false == gyroscopeCalibrationIsDone) {
            gyroscopeCalibrationIsDone = GY87_CalibrateGyroscope(hgy87);
        }

        /* Calibrate GY-87 accelerometer sensor */
        if (false == accelerometerCalibrationIsDone) {
            accelerometerCalibrationIsDone = GY87_CalibrateAccelerometer(hgy87);

            if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
#ifdef MAIN_APP_USE_LOGGING_STARTUP
                vTaskDelay(pdMS_TO_TICKS(5));
                LOG((uint8_t *)"Flight Controller Initialized.\r\n\n", LOG_INFORMATION);
#endif
                FlightController_isInitialized = true;
            }
        }

        /* Read flight lights control */
        FSA8S_channelValues[7] = FSA8S_ReadChannel(rc_controller, CHANNEL_8);
        FSA8S_channelValues[8] = FSA8S_ReadChannel(rc_controller, CHANNEL_9);
        FSA8S_channelValues[9] = FSA8S_ReadChannel(rc_controller, CHANNEL_10);

        /* Control system processing */
        if (FlightController_isInitialized && 0 == CONTROL_SYSTEM_MODE) {

#if 0 == CONTROL_SYSTEM_MODE

        	/* Avoid uncontrolled motor start */
        	while (false == throttleStick_startedDown) {

        		/* Read throttle input from radio controller */
        		inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

        		if (15 > inputValue_throttle) {

        			throttleStick_startedDown = true;

        		} else {

        			throttleStick_startedDown = false;
        		}
        	}

        	/* Check if ESCs are enabled (Switch B on radio controller) */
        	if (500 <= FSA8S_ReadChannel(rc_controller, CHANNEL_6)) {
        		ESC_isEnabled = true;
        	} else {
        		ESC_isEnabled = false;
        	}

    		/* Turn off motors in case ESCs are disabled */
    		if(false == ESC_isEnabled) {

    			/* Save motors speed */
    			ESC_speeds[1] = 0;
    			ESC_speeds[2] = 0;
    			ESC_speeds[3] = 0;
    			ESC_speeds[4] = 0;

                /* Turn off motors */
                ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

    		} else {

                /* Check if timer has expired */
                if (Timer4_flag) {

            		/* Read input throttle from radio controller */
                	inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                    if (CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE > inputValue_throttle) {

                        /* Save motors speed */
                        ESC_speeds[1] = 0;
                        ESC_speeds[2] = 0;
                        ESC_speeds[3] = 0;
                        ESC_speeds[4] = 0;

                        /* Turn off motors */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

                    } else {

                		/* Adjust and limit throttle input */
                		if (CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE < inputValue_throttle) {
                			inputValue_throttle = CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE;
                		}

                        /* Calculate motors speed */
                        motorSpeed1 = inputValue_throttle / 10;
                        motorSpeed2 = inputValue_throttle / 10;
                        motorSpeed3 = inputValue_throttle / 10;
                        motorSpeed4 = inputValue_throttle / 10;

                        /* Adjust and limit motors maximum speed */
                        if (ESC_MAXIMUM_SPEED < motorSpeed1)
                            motorSpeed1 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed2)
                            motorSpeed2 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed3)
                            motorSpeed3 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed4)
                            motorSpeed4 = ESC_MAXIMUM_SPEED;

                        /* Adjust and limit motors minimum speed */
                        if (0 > motorSpeed1)
                            motorSpeed1 = 0;
                        if (0 > motorSpeed2)
                            motorSpeed2 = 0;
                        if (0 > motorSpeed3)
                            motorSpeed3 = 0;
                        if (0 > motorSpeed4)
                            motorSpeed4 = 0;

						/* Save motors speed */
                        ESC_speeds[1] = motorSpeed1;
                        ESC_speeds[2] = motorSpeed2;
                        ESC_speeds[3] = motorSpeed3;
                        ESC_speeds[4] = motorSpeed4;

                        /* Set motors speed */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);
					}

					/* Reset Timer4 flag */
					Timer4_flag = false;

    			}

    		}

#endif

        } else if (FlightController_isInitialized && 1 == CONTROL_SYSTEM_MODE) {

#if 1 == CONTROL_SYSTEM_MODE

            /* Avoid uncontrolled motor start */
            while (false == throttleStick_startedDown) {

                /* Read throttle input from radio controller */
                inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                if (15 > inputValue_throttle) {

                    throttleStick_startedDown = true;

                } else {

                    throttleStick_startedDown = false;
                }
            }

            /* Check if ESCs are enabled (Switch B on radio controller) */
            if (500 <= FSA8S_ReadChannel(rc_controller, CHANNEL_6)) {
                ESC_isEnabled = true;
            } else {
                ESC_isEnabled = false;
            }

            /* Turn off motors in case ESCs are disabled */
            if (false == ESC_isEnabled) {

                /* Save motors speed */
                ESC_speeds[1] = 0;
                ESC_speeds[2] = 0;
                ESC_speeds[3] = 0;
                ESC_speeds[4] = 0;

                /* Turn off motors */
                ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

                /* Reset PID variables */
                CSM1_ResetPID();

            } else {

                /* Check if timer has expired */
                if (Timer4_flag) {

                    /* Read input throttle from radio controller */
                    inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                    if (CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE > inputValue_throttle) {

                        /* Save motors speed */
                        ESC_speeds[1] = 0;
                        ESC_speeds[2] = 0;
                        ESC_speeds[3] = 0;
                        ESC_speeds[4] = 0;

                        /* Turn off motors */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

                        /* Reset PID variables */
                        CSM1_ResetPID();

                    } else {

                        /* Read GY-87 gyroscope sensor */
                        if (true == gyroscopeCalibrationIsDone) {
                            GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues);
                        }

                        /* Read GY-87 accelerometer sensor */
                        if (true == accelerometerCalibrationIsDone) {
                            GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues);
                        }

                        /* Read inputs from radio controller */
                        inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
                        inputValue_rollRate = FSA8S_ReadChannel(rc_controller, CHANNEL_1);
                        inputValue_pitchRate = FSA8S_ReadChannel(rc_controller, CHANNEL_2);
                        inputValue_yawRate = FSA8S_ReadChannel(rc_controller, CHANNEL_4);

                        /* Adjust and limit throttle input */
                        if (CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE < inputValue_throttle) {
                            inputValue_throttle = CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE;
                        }

                        /* Calculate desired rates by mapping radio controller values to rates */
                        desiredValue_rollRate = 0.15 * (inputValue_rollRate - 500);
                        desiredValue_pitchRate = 0.15 * (inputValue_pitchRate - 500);
                        desiredValue_yawRate = 0.15 * (inputValue_yawRate - 500);

                        /* Calculate rates errors */
                        errorValue_rollRate = desiredValue_rollRate - GY87_gyroscopeValues.rotationRateRoll;
                        errorValue_pitchRate = desiredValue_pitchRate - GY87_gyroscopeValues.rotationRatePitch;
                        errorValue_yawRate = desiredValue_yawRate - GY87_gyroscopeValues.rotationRateYaw;

                        /* Calculate PID for roll rate */
                        CSM1_CalculatePID(&pidOutputValue_rollRate, &previousIterm_rollRate, &previousErrorValue_rollRate, errorValue_rollRate, kP_rollRate, kI_rollRate, kD_rollRate);

                        /* Calculate PID for pitch rate */
                        CSM1_CalculatePID(&pidOutputValue_pitchRate, &previousIterm_pitchRate, &previousErrorValue_pitchRate, errorValue_pitchRate, kP_pitchRate, kI_pitchRate, kD_pitchRate);

                        /* Calculate PID for yaw rate */
                        CSM1_CalculatePID(&pidOutputValue_yawRate, &previousIterm_yawRate, &previousErrorValue_yawRate, errorValue_yawRate, kP_yawRate, kI_yawRate, kD_yawRate);

                        /* Calculate motors speed */
                        motorSpeed1 = ((inputValue_throttle / 10) - pidOutputValue_rollRate - pidOutputValue_pitchRate - pidOutputValue_yawRate);
                        motorSpeed2 = ((inputValue_throttle / 10) + pidOutputValue_rollRate + pidOutputValue_pitchRate - pidOutputValue_yawRate);
                        motorSpeed3 = ((inputValue_throttle / 10) + pidOutputValue_rollRate - pidOutputValue_pitchRate + pidOutputValue_yawRate);
                        motorSpeed4 = ((inputValue_throttle / 10) - pidOutputValue_rollRate + pidOutputValue_pitchRate + pidOutputValue_yawRate);

                        /* Adjust and limit motors maximum speed */
                        if (ESC_MAXIMUM_SPEED < motorSpeed1)
                            motorSpeed1 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed2)
                            motorSpeed2 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed3)
                            motorSpeed3 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed4)
                            motorSpeed4 = ESC_MAXIMUM_SPEED;

                        /* Adjust and limit motors minimum speed */
                        if (0 > motorSpeed1)
                            motorSpeed1 = 0;
                        if (0 > motorSpeed2)
                            motorSpeed2 = 0;
                        if (0 > motorSpeed3)
                            motorSpeed3 = 0;
                        if (0 > motorSpeed4)
                            motorSpeed4 = 0;

                        /* Save motors speed */
                        ESC_speeds[1] = motorSpeed1;
                        ESC_speeds[2] = motorSpeed2;
                        ESC_speeds[3] = motorSpeed3;
                        ESC_speeds[4] = motorSpeed4;

                        /* Set motors speed */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);
                    }

                    /* Reset Timer4 flag */
                    Timer4_flag = false;
                }
            }

#endif

        } else if (FlightController_isInitialized && 2 == CONTROL_SYSTEM_MODE) {

#if 2 == CONTROL_SYSTEM_MODE

            /* Avoid uncontrolled motor start */
            while (false == throttleStick_startedDown) {

                /* Read throttle input from radio controller */
                inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                if (15 > inputValue_throttle) {

                    throttleStick_startedDown = true;

                } else {

                    throttleStick_startedDown = false;
                }
            }

            /* Check if ESCs are enabled (Switch B on radio controller) */
            if (500 <= FSA8S_ReadChannel(rc_controller, CHANNEL_6)) {
                ESC_isEnabled = true;
            } else {
                ESC_isEnabled = false;
            }

            /* Turn off motors in case ESCs are disabled */
            if (false == ESC_isEnabled) {

                /* Save motors speed */
                ESC_speeds[1] = 0;
                ESC_speeds[2] = 0;
                ESC_speeds[3] = 0;
                ESC_speeds[4] = 0;

                /* Turn off motors */
                ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

                /* Reset PID variables */
                CSM2_ResetPID();

            } else {

                /* Check if timer has expired */
                if (Timer4_flag) {

                    /* Read input throttle from radio controller */
                    inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                    /* Check if throttle stick is low */
                    if (CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE > inputValue_throttle) {

                        /* Save motors speed */
                        ESC_speeds[1] = 0;
                        ESC_speeds[2] = 0;
                        ESC_speeds[3] = 0;
                        ESC_speeds[4] = 0;

                        /* Turn off motors */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

                        /* Reset PID variables */
                        CSM2_ResetPID();

                    } else {

                        /* Read GY-87 gyroscope sensor */
                        if (true == gyroscopeCalibrationIsDone) {
                            GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues);
                        }

                        /* Read GY-87 accelerometer sensor */
                        if (true == accelerometerCalibrationIsDone) {
                            GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues);
                        }

                        /* Calculate Kalman roll angle */
                        CSM2_CalculateKalman(kalmanPredictionValue_rollAngle, kalmanUncertaintyValue_rollAngle, GY87_gyroscopeValues.rotationRateRoll, GY87_accelerometerValues.angleRoll);
                        kalmanPredictionValue_rollAngle = kalmanOutput[0];
                        kalmanUncertaintyValue_rollAngle = kalmanOutput[1];

                        /* Calculate Kalman pitch angle */
                        CSM2_CalculateKalman(kalmanPredictionValue_pitchAngle, kalmanUncertaintyValue_pitchAngle, GY87_gyroscopeValues.rotationRatePitch, GY87_accelerometerValues.anglePitch);
                        kalmanPredictionValue_pitchAngle = kalmanOutput[0];
                        kalmanUncertaintyValue_pitchAngle = kalmanOutput[1];

                        /* Read inputs from radio controller */
                        inputValue_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
                        inputValue_rollAngle = FSA8S_ReadChannel(rc_controller, CHANNEL_1);
                        inputValue_pitchAngle = FSA8S_ReadChannel(rc_controller, CHANNEL_2);
                        inputValue_yawRate = FSA8S_ReadChannel(rc_controller, CHANNEL_4);

                        /* Adjust and limit throttle input */
                        if (CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE < inputValue_throttle) {
                            inputValue_throttle = CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE;
                        }

                        /* Calculate desired angles by mapping radio controller values to angles */
                        desiredValue_rollAngle = 0.10 * (inputValue_rollRate - 500);
                        desiredValue_pitchAngle = 0.10 * (inputValue_pitchRate - 500);

                        /* Calculate angles errors */
                        errorValue_rollAngle = desiredValue_rollAngle - kalmanPredictionValue_rollAngle;
                        errorValue_pitchAngle = desiredValue_pitchAngle - kalmanPredictionValue_pitchAngle;

                        /* Calculate PID for roll angle */
                        CSM1_CalculatePID(&pidOutputValue_rollAngle, &previousIterm_rollAngle, &previousErrorValue_rollAngle, errorValue_rollAngle, kP_rollAngle, kI_rollAngle, kD_rollAngle);

                        /* Calculate PID for pitch angle */
                        CSM1_CalculatePID(&pidOutputValue_pitchAngle, &previousIterm_pitchAngle, &previousErrorValue_pitchAngle, errorValue_pitchAngle, kP_pitchAngle, kI_pitchAngle, kD_pitchAngle);

                        /* Calculate desired rates */
                        desiredValue_rollRate = pidOutputValue_rollAngle;
                        desiredValue_pitchRate = pidOutputValue_pitchAngle;
                        desiredValue_yawRate = 0.15 * (FSA8S_channelValues[3] - 500);

                        /* Calculate rates errors */
                        errorValue_rollRate = desiredValue_rollRate - GY87_gyroscopeValues.rotationRateRoll;
                        errorValue_pitchRate = desiredValue_pitchRate - GY87_gyroscopeValues.rotationRatePitch;
                        errorValue_yawRate = desiredValue_yawRate - GY87_gyroscopeValues.rotationRateYaw;

                        /* Calculate PID for roll rate */
                        CSM1_CalculatePID(&pidOutputValue_rollRate, &previousIterm_rollRate, &previousErrorValue_rollRate, errorValue_rollRate, kP_rollRate, kI_rollRate, kD_rollRate);

                        /* Calculate PID for pitch rate */
                        CSM1_CalculatePID(&pidOutputValue_pitchRate, &previousIterm_pitchRate, &previousErrorValue_pitchRate, errorValue_pitchRate, kP_pitchRate, kI_pitchRate, kD_pitchRate);

                        /* Calculate PID for yaw rate */
                        CSM1_CalculatePID(&pidOutputValue_yawRate, &previousIterm_yawRate, &previousErrorValue_yawRate, errorValue_yawRate, kP_yawRate, kI_yawRate, kD_yawRate);

                        /* Calculate motors speed */
                        motorSpeed1 = ((inputValue_throttle / 10) - pidOutputValue_rollRate - pidOutputValue_pitchRate - pidOutputValue_yawRate);
                        motorSpeed2 = ((inputValue_throttle / 10) + pidOutputValue_rollRate + pidOutputValue_pitchRate - pidOutputValue_yawRate);
                        motorSpeed3 = ((inputValue_throttle / 10) + pidOutputValue_rollRate - pidOutputValue_pitchRate + pidOutputValue_yawRate);
                        motorSpeed4 = ((inputValue_throttle / 10) - pidOutputValue_rollRate + pidOutputValue_pitchRate + pidOutputValue_yawRate);

                        /* Adjust and limit motors maximum speed */
                        if (ESC_MAXIMUM_SPEED < motorSpeed1)
                            motorSpeed1 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed2)
                            motorSpeed2 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed3)
                            motorSpeed3 = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < motorSpeed4)
                            motorSpeed4 = ESC_MAXIMUM_SPEED;

                        /* Adjust and limit motors minimum speed */
                        if (0 > motorSpeed1)
                            motorSpeed1 = 0;
                        if (0 > motorSpeed2)
                            motorSpeed2 = 0;
                        if (0 > motorSpeed3)
                            motorSpeed3 = 0;
                        if (0 > motorSpeed4)
                            motorSpeed4 = 0;

                        /* Save motors speed */
                        ESC_speeds[1] = motorSpeed1;
                        ESC_speeds[2] = motorSpeed2;
                        ESC_speeds[3] = motorSpeed3;
                        ESC_speeds[4] = motorSpeed4;

                        /* Set motors speed */
                        ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
                        ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
                        ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
                        ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);
                    }

                    /* Reset Timer4 flag */
                    Timer4_flag = false;
                }
            }

#endif

        } else if (FlightController_isInitialized && 3 == CONTROL_SYSTEM_MODE) {
        }

        /* Set task time delay */
        vTaskDelay(xDelay);
    }
}

void FlightController_Data_Logging(void * ptr) {

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(DEFAULT_TASK_DELAY * LOGGING_TASK_DELAY_MULTIPLIER);

    uint8_t loggingStr[150] = {0};

#ifdef MAIN_APP_USE_LOGGING_GY87_TEMPERATURE
    int16_t GY87_temperature_log = 0;
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE
    float GY87_barometerPressureValue_log = 0;
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE
    float GY87_barometerAltitudeValue_log = 0;
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE
    float GY87_barometerTemperatureValue_log = 0;
#endif

    while (1) {

#ifdef MAIN_APP_USE_LOGGING_CONTROL_SYSTEM_MODE0
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            if (ESC_isEnabled) {
                sprintf((char *)loggingStr, "Input Throttle: %04d | ESCs Enabled  | ESC1: %03d%%, ESC2: %03d%%, ESC3: %03d%%, ESC4: %03d%%\r\n", (uint16_t)inputValue_throttle, (uint16_t)ESC_speeds[1], (uint16_t)ESC_speeds[2], (uint16_t)ESC_speeds[3],
                        (uint16_t)ESC_speeds[4]);
            } else {
                sprintf((char *)loggingStr, "Input Throttle: %04d | ESCs Disabled | ESC1: %03d%%, ESC2: %03d%%, ESC3: %03d%%, ESC4: %03d%%\r\n", (uint16_t)inputValue_throttle, (uint16_t)ESC_speeds[1], (uint16_t)ESC_speeds[2], (uint16_t)ESC_speeds[3],
                        (uint16_t)ESC_speeds[4]);
            }
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_CONTROL_SYSTEM_MODE1
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            if (ESC_isEnabled) {
                sprintf((char *)loggingStr, "Input Throttle: %04d | ESCs Enabled  | ESC1: %03d%%, ESC2: %03d%%, ESC3: %03d%%, ESC4: %03d%%\r\n", (uint16_t)inputValue_throttle, (uint16_t)ESC_speeds[1], (uint16_t)ESC_speeds[2], (uint16_t)ESC_speeds[3],
                        (uint16_t)ESC_speeds[4]);
            } else {
                sprintf((char *)loggingStr, "Input Throttle: %04d | ESCs Disabled | ESC1: %03d%%, ESC2: %03d%%, ESC3: %03d%%, ESC4: %03d%%\r\n", (uint16_t)inputValue_throttle, (uint16_t)ESC_speeds[1], (uint16_t)ESC_speeds[2], (uint16_t)ESC_speeds[3],
                        (uint16_t)ESC_speeds[4]);
            }
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_FSA8S
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log channel values */
            sprintf((char *)loggingStr, "FSA8S Ch1: %04d, Ch2: %04d, Ch3: %04d, Ch4: %04d, Ch5: %04d, Ch6: %04d, Ch7: %04d, Ch8: %04d, Ch9: %04d, Ch10: %04d\r\n", FSA8S_channelValues[0], FSA8S_channelValues[1], FSA8S_channelValues[2],
                    FSA8S_channelValues[3], FSA8S_channelValues[4], FSA8S_channelValues[5], FSA8S_channelValues[6], FSA8S_channelValues[7], FSA8S_channelValues[8], FSA8S_channelValues[9]);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_GYROSCOPE
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 gyroscope values */
            sprintf((char *)loggingStr, (const char *)"GY87 Gyroscope ROLL: %.2f [°/s], PITCH: %.2f [°/s], YAW: %.2f [°/s]\r\n", GY87_gyroscopeValues.rotationRateRoll, GY87_gyroscopeValues.rotationRatePitch, GY87_gyroscopeValues.rotationRateYaw);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 accelerometer values */
            sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer X: %.2f [g], Y: %.2f[g], Z: %.2f [g]\r\n", GY87_accelerometerValues.linearAccelerationX, GY87_accelerometerValues.linearAccelerationY,
                    GY87_accelerometerValues.linearAccelerationZ);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_ACCELEROMETER_ANGLES
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 accelerometer angles */
            sprintf((char *)loggingStr, (const char *)"GY87 Accelerometer ROLL: %.2f [°], PITCH: %.2f [°]\r\n", GY87_accelerometerValues.angleRoll, GY87_accelerometerValues.anglePitch);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_KALMAN_ANGLES
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 accelerometer angles */
            sprintf((char *)loggingStr, (const char *)"GY87 Kalman ROLL: %.2f [°], Kalman PITCH: %.2f [°]\r\n", kalmanPredictionValue_rollAngle, kalmanPredictionValue_pitchAngle);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_TEMPERATURE
        /* Read GY87 temperature value */
        GY87_temperature_log = GY87_ReadTemperatureSensor(hgy87);

        /*  Log GY87 temperature value */
        sprintf((char *)loggingStr, (const char *)"GY87 Temperature: %d [°C]\r\n", GY87_temperature_log);
        LOG(loggingStr, LOG_INFORMATION);
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 magnetometer values */
            sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer X: %.3f [G], Y: %.3f [G], Z: %.3f [G]\r\n", GY87_magnetometerValues.magneticFieldX, GY87_magnetometerValues.magneticFieldY, GY87_magnetometerValues.magneticFieldZ);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_MAGNETOMETER_HEADING
        /* Check if GY-87 calibrations are done */
        if (true == gyroscopeCalibrationIsDone && true == accelerometerCalibrationIsDone) {
            /* Log GY87 magnetometer heading value */
            sprintf((char *)loggingStr, (const char *)"GY87 Magnetometer Heading: %.2f [°]\r\n", GY87_magnetometerHeadingValue);
            LOG(loggingStr, LOG_INFORMATION);
        }
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_TEMPERATURE

        /* Read GY87 barometer temperature value */
        GY87_barometerTemperatureValue_log = GY87_ReadBarometerTemperature(hgy87);

        /* Log GY87 barometer pressure value */
        sprintf((char *)loggingStr, (const char *)"GY87 Barometer Temperature: %.2f [°C]\r\n", GY87_barometerTemperatureValue_log);
        LOG(loggingStr, LOG_INFORMATION);
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_PRESSURE
        /* Read GY87 barometer pressure value */
        GY87_barometerPressureValue_log = GY87_ReadBarometerPressure(hgy87);

        /* Log GY87 barometer pressure value */
        sprintf((char *)loggingStr, (const char *)"GY87 Barometer Pressure: %.2f [Pa]\r\n", GY87_barometerPressureValue_log);
        LOG(loggingStr, LOG_INFORMATION);
#endif

#ifdef MAIN_APP_USE_LOGGING_GY87_BAROMETER_ALTITUDE
        /* Read GY87 barometer altitude value */
        GY87_barometerAltitudeValue_log = GY87_ReadBarometerAltitude(hgy87);

        /* Log GY87 barometer altitude value */
        sprintf((char *)loggingStr, (const char *)"GY87 Barometer Altitude: %.2f [m]\r\n", GY87_barometerAltitudeValue_log);
        LOG(loggingStr, LOG_INFORMATION);
#endif

#ifdef MAIN_APP_USE_LOGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL
        sprintf((char *)loggingStr, (const char *)"Battery Level: %.2f [V]\r\n\n", FlightController_batteryLevelValue);
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

    /* Change delay from time in [ms] to ticks */
    const TickType_t xDelay = pdMS_TO_TICKS(1000);

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
        FlightController_batteryLevelValue = FlightController_batteryLevelValue * 3.363636 + BATTERY_LEVEL_CALIBRATION_OFFSET;

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

        if (FlightController_isInitialized) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);

            /* Change pin state */
            if (ledState == GPIO_PIN_RESET) {

                ledState = GPIO_PIN_SET;
            } else {

                ledState = GPIO_PIN_RESET;
            }
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
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

void CSM1_CalculatePID(float * PID_Output, float * previousIterm, float * previousErrorValue, float errorValue, float kP, float kI, float kD) {

    float Pterm;
    float Iterm;
    float Dterm;
    float pidOutputValue;

    Pterm = kP * errorValue;

    Iterm = *previousIterm + kI * (errorValue + *previousErrorValue) * 0.004 / 2;
    if (-CONTROL_SYSTEM_PID_OUTPUT_LIMIT > Iterm) {
        Iterm = -CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    } else if (CONTROL_SYSTEM_PID_OUTPUT_LIMIT < Iterm) {
        Iterm = CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    }

    Dterm = kD * (errorValue - *previousErrorValue) / 0.004;

    pidOutputValue = Pterm + Iterm + Dterm;
    if (-CONTROL_SYSTEM_PID_OUTPUT_LIMIT > pidOutputValue) {
        pidOutputValue = -CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    } else if (CONTROL_SYSTEM_PID_OUTPUT_LIMIT < pidOutputValue) {
        pidOutputValue = CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    }

    *PID_Output = pidOutputValue;
    *previousErrorValue = errorValue;
    *previousIterm = Iterm;
}

void CSM1_ResetPID(void) {

#if 1 == CONTROL_SYSTEM_MODE

    /* Reset previously stored PID errors and terms values */
    previousErrorValue_rollRate = 0;
    previousErrorValue_pitchRate = 0;
    previousErrorValue_yawRate = 0;
    previousIterm_rollRate = 0;
    previousIterm_pitchRate = 0;
    previousIterm_yawRate = 0;

#endif
}

void CSM2_CalculateKalman(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {

#if 2 == CONTROL_SYSTEM_MODE

    float kalmanGain;

    kalmanState = kalmanState + 0.004 * kalmanInput;
    kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    kalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + 3 * 3);
    kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);
    kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;

    kalmanOutput[0] = kalmanState;
    kalmanOutput[1] = kalmanUncertainty;

#endif
}

void CSM2_CalculatePID(void) {
}

void CSM2_ResetPID(void) {

#if 2 == CONTROL_SYSTEM_MODE

    /* Reset previously stored PID errors and terms values: Angles */
    previousErrorValue_rollAngle = 0;
    previousErrorValue_pitchAngle = 0;
    previousIterm_rollAngle = 0;
    previousIterm_pitchAngle = 0;

    /* Reset previously stored PID errors and terms values: Rates */
    previousErrorValue_rollRate = 0;
    previousErrorValue_pitchRate = 0;
    previousErrorValue_yawRate = 0;
    previousIterm_rollRate = 0;
    previousIterm_pitchRate = 0;
    previousIterm_yawRate = 0;

#endif
}

/* --- Private callback function implementation ------------------------------------------------ */
void Timer1_Callback(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer1_AutoReloadTime) / xTimerPeriod)) {
        /* Check if On/Off Button is still pressed after 3 seconds */
        if (!HAL_GPIO_ReadPin(PW_ON_OFF_DRIVER_INPUT_GPIO_Port, PW_ON_OFF_DRIVER_INPUT_Pin)) {

            if (!FlightController_isRunning) {
                /* Flight controller was off */
                /* User turned it on */
                /* Turn on flight controller */
                HAL_GPIO_WritePin(PW_ON_OFF_DRIVER_OUTPUT_GPIO_Port, PW_ON_OFF_DRIVER_OUTPUT_Pin, 1);

                FlightController_isRunning = true;
            } else {
                /* Flight controller was on */
                /* User turned it off */
                /* Suspend HeartbeatLight task and turn on-board LED on */
                vTaskSuspend(FlightController_HeartbeatLight_Handle);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

                FlightController_isRunning = false;

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

    if (ulCount >= (pdMS_TO_TICKS(Timer2_AutoReloadTime) / xTimerPeriod)) {

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

    if (ulCount >= (pdMS_TO_TICKS(Timer3_AutoReloadTime) / xTimerPeriod)) {

        /* Set Timer3 flag to true */
        Timer3_flag = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer4_Callback(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer4_AutoReloadTime) / xTimerPeriod)) {

        /* Set Timer3 flag to true */
        Timer4_flag = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    /* Create start-up tasks and timers */
    FreeRTOS_CreateStartUpTasks();
}

/* --- End of file ----------------------------------------------------------------------------- */
