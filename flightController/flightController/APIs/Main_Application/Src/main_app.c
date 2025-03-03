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
#include "controlSystem_settings.h"
#include "debug_signals_ids.h"

#include "LoggingSystem_UAI.h"
#include "FSA8S_driver_UAI.h"
#include "MPU6050_driver_UAI.h"
#include "ESC_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
/* FreeRTOS Settings */
#define USE_FREERTOS                   // Remove comment when using FreeRTOS
#define DEFAULT_TASK_DELAY             (100)
#define TASK_ONOFFBUTTON_PRIORITY      (3)
#define TASK_CONTROLSYSTEM_PRIORITY    (5)
#define TASK_USBCOMMUNICATION_PRIORITY (4)
#define TASK_DEBUGGING_PRIORITY        (4)
#define TASK_BATTERYLEVEL_PRIORITY     (1)
#define TASK_BATTERYALARM_PRIORITY     (1)
#define TASK_HEARTBEATLIGHT_PRIORITY   (2)
#define TASK_FLIGHTLIGHTS_PRIORITY     (2)
/* Logging and Debugging Settings */
#define MAIN_APP_LOGGING_DEBUGGING                               (1)
#define MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE                   (1024)
#define MAIN_APP_DEBUGGING_FSA8S_MAIN                            (1)
#define MAIN_APP_DEBUGGING_FSA8S_AUX                             (1)
#define MAIN_APP_DEBUGGING_GY87_GYROSCOPE_CALIBRATION_VALUES     (1)
#define MAIN_APP_DEBUGGING_GY87_GYROSCOPE_VALUES                 (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_CALIBRATION_VALUES (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_VALUES             (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_ANGLES             (1)
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES              (0) // Not necessary
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING             (0) // Not necessary
#define MAIN_APP_DEBUGGING_GY87_TEMPERATURE                      (1) // Not necessary
#define MAIN_APP_DEBUGGING_ESCS                                  (1)
#define MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL       (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES        (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_KALMAN_ANGLES           (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_ERRORS           (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_PID              (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_RATE          (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_ERRORS            (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_PID               (1)
#define MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK             (1)
/* Drivers Settings */
#define FSA8S_CHANNELS                     (10) // Number of remote control channels to read
#define ESC_MAXIMUM_SPEED                  (90)
#define ESC_MINIMUM_SPEED                  (10)
#define USB_COMMUNICATION_INFO_QUEUE_SIZE  (32)
#define USB_COMMUNICATION_DEBUG_QUEUE_SIZE (1)

/* Battery Level Settings */
#define BATTERY_LEVEL_CALIBRATION_OFFSET (0.76)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Flight Controller State */
static bool_t FlightController_isRunning = false;
static bool_t FlightController_isInitialized = false;

/* Timers Handles */
static TimerHandle_t Timer_Handle_OnOffButton = NULL;
static TimerHandle_t Timer_Handle_BatteryLevelAlarm = NULL;
static TimerHandle_t Timer_Handle_FlightLights = NULL;
static TimerHandle_t Timer_Handle_ControlSystem = NULL;

/* Queues Handles */
static QueueHandle_t Queue_Handle_USB_Communication_Info = NULL;
static QueueHandle_t Queue_Handle_USB_Communication_Debug = NULL;

/* Mutexes Handles */
static SemaphoreHandle_t Mutex_Handle_LogDebuggingString = NULL;
static SemaphoreHandle_t Mutex_Handle_LogInformationString = NULL;

/* Tasks Handles */
static TaskHandle_t Task_Handle_OnOffButton = NULL;
static TaskHandle_t Task_Handle_ControlSystem = NULL;
static TaskHandle_t Task_Handle_USB_Communication = NULL;
static TaskHandle_t Task_Handle_Debugging = NULL;
static TaskHandle_t Task_Handle_BatteryLevel = NULL;
static TaskHandle_t Task_Handle_BatteryAlarm = NULL;
static TaskHandle_t Task_Handle_HeartbeatLight = NULL;
static TaskHandle_t Task_Handle_FlightLights = NULL;

/* Timers Variables */
static bool_t Timer_Flag_OnOffButton = false;
static bool_t Timer_Flag_BatteryLevelAlarm = false;
static bool_t Timer_Flag_FlightLights = false;
static bool_t Timer_Flag_ControlSystem = false;
static uint16_t Timer_AutoReloadTime_OnOffButton = PW_ON_OFF_DRIVER_TIME;
static uint16_t Timer_AutoReloadTime_BatteryLevelAlarm = 200;
static uint16_t Timer_AutoReloadTime_FlightLights = 200;
static uint16_t Timer_AutoReloadTime_ControlSystem = CONTROL_SYSTEM_LOOP_PERIOD_MS;

/* Stack High Watermarks Variables */
static UBaseType_t Task_StackHighWatermark_OnOffButton = 0;
static UBaseType_t Task_StackHighWatermark_ControlSystem = 0;
static UBaseType_t Task_StackHighWatermark_USB_Communication = 0;
static UBaseType_t Task_StackHighWatermark_Debugging = 0;
static UBaseType_t Task_StackHighWatermark_BatteryLevel = 0;
static UBaseType_t Task_StackHighWatermark_BatteryAlarm = 0;
static UBaseType_t Task_StackHighWatermark_HeartbeatLight = 0;
static UBaseType_t Task_StackHighWatermark_FlightLights = 0;

/* Debugging Variables */
static uint8_t debuggingStr_systemTime[16] = {0};                          // Size checked
static uint8_t debuggingStr_FSA8S_main[50] = {0};                          // Size checked
static uint8_t debuggingStr_FSA8S_aux[50] = {0};                           // Size checked
static uint8_t debuggingStr_GY87_gyroscopeCalibrationValues[40] = {0};     // Size checked
static uint8_t debuggingStr_GY87_gyroscopeValues[40] = {0};                // Size checked
static uint8_t debuggingStr_GY87_accelerometerCalibrationValues[40] = {0}; // Size checked
static uint8_t debuggingStr_GY87_accelerometerValues[40] = {0};            // Size checked
static uint8_t debuggingStr_GY87_accelerometerAngles[40] = {0};            // Size checked
static uint8_t debuggingStr_GY87_magnetometerValues[40] = {0};             // Size checked
static uint8_t debuggingStr_GY87_magnetometerHeadingValue[16] = {0};       // Size checked
static uint8_t debuggingStr_GY87_temperature[16] = {0};                    // Size checked
static uint8_t debuggingStr_ESCs[40] = {0};                                // Size checked
static uint8_t debuggingStr_BatteryLevel[10] = {0};                        // Size checked
static uint8_t debuggingStr_ControlSystem_ReferenceAngles[40] = {0};
static uint8_t debuggingStr_ControlSystem_KalmanAngles[30] = {0}; // Size checked
static uint8_t debuggingStr_ControlSystem_AnglesErrors[40] = {0};
static uint8_t debuggingStr_ControlSystem_AnglesPID[40] = {0};
static uint8_t debuggingStr_ControlSystem_ReferenceRates[40] = {0};
static uint8_t debuggingStr_ControlSystem_RatesErrors[40] = {0};
static uint8_t debuggingStr_ControlSystem_RatesPID[40] = {0};
static uint8_t debuggingStr_TasksStackHighWatermark[80] = {0}; // Size checked

/* Drivers Handles */
static IBUS_HandleTypeDef_t * rc_controller = NULL;
static GY87_HandleTypeDef_t * hgy87 = NULL;
static ESC_HandleTypeDef_t * hesc = NULL;

/* FS-A8S Radio Controller Variables */
static FSA8S_CHANNEL_t channels[FSA8S_CHANNELS] = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10};
static uint16_t FSA8S_channelValues[FSA8S_CHANNELS] = {0};

/* GY87 IMU Variables */
static GY87_gyroscopeCalibrationValues_t GY87_gyroscopeCalibrationValues = {0.0, 0.0, 0.0, true};
static GY87_gyroscopeValues_t GY87_gyroscopeValues;
static GY87_accelerometerCalibrationValues_t GY87_accelerometerCalibrationValues = {0.0, 0.0, 0.0, true};
static GY87_accelerometerValues_t GY87_accelerometerValues;
static GY87_magnetometerValues_t GY87_magnetometerValues;
static float GY87_magnetometerHeadingValue = 0;
static float GY87_temperature = 0;
static bool_t gyroscopeCalibrationIsDone = false;
static bool_t accelerometerCalibrationIsDone = false;
/* Kalman Filter Variables */
static float Kalman_predictionValue_rollAngle = 0;
static float Kalman_predictionValue_pitchAngle = 0;
static float Kalman_uncertaintyValue_rollAngle = 2 * 2;
static float Kalman_uncertaintyValue_pitchAngle = 2 * 2;

/* Control System Mode 1 */
/* Throttle stick check */
static bool_t throttleStick_startedDown = false;
/* References */
static float reference_throttle = 0;
static float reference_rollValue = 0;
static float reference_pitchValue = 0;
static float reference_rollAngle = 0;
static float reference_pitchAngle = 0;
/* Errors: Angles */
static float error_rollAngle = 0;
static float error_pitchAngle = 0;
/* Previously stored errors: Angles */
static float previousErrorValue_rollAngle = 0;
static float previousErrorValue_pitchAngle = 0;
/* Previously stored terms: Angles */
static float previousIterm_rollAngle = 0;
static float previousIterm_pitchAngle = 0;
/* PID outputs: Angles */
static float pidOutput_rollAngle = 0;
static float pidOutput_pitchAngle = 0;
/* References: Rates */
static float reference_yawValue = 0;
/* Desired references: Rates */
static float desiredValue_rollRate = 0;
static float desiredValue_pitchRate = 0;
static float desiredValue_yawRate = 0;
/* Errors: Rates */
static float error_rollRate = 0;
static float error_pitchRate = 0;
static float error_yawRate = 0;
/* Previously stored errors: Rates */
static float previousErrorValue_rollRate = 0;
static float previousErrorValue_pitchRate = 0;
static float previousErrorValue_yawRate = 0;
/* Previously stored terms: Rates  */
static float previousIterm_rollRate = 0;
static float previousIterm_pitchRate = 0;
static float previousIterm_yawRate = 0;
/* PID outputs: Rates */
static float pidOutput_rollRate = 0;
static float pidOutput_pitchRate = 0;
static float pidOutput_yawRate = 0;
/* Motors inputs */
static float motorSpeed1 = 0;
static float motorSpeed2 = 0;
static float motorSpeed3 = 0;
static float motorSpeed4 = 0;
/* ESCs */
static bool_t ESC_isEnabled = false;
static float ESC_speeds[5] = {0};

/* Flight Controller Battery Level */
static float FlightController_batteryLevelValue = 11.1;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Creates system software timers.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateTimers(void);

/*
 * @brief  Creates system software queues.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateQueues(void);

/*
 * @brief  Creates system software mutexes.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateMutexes(void);

/*
 * @brief  Creates other system tasks.
 * @param  None
 * @retval None
 */
void FreeRTOS_CreateTasks(void);

/*
 * @brief  Task: Controls the whole system as a closed-loop system, taking as inputs the data
 *               received by the radio controller receiver and the IMU module, and controlling
 *               accordingly the electronic speed controllers.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_ControlSystem(void * ptr);

/*
 * @brief  Task: Sends data through USB port.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_USB_Communication(void * ptr);

/*
 * @brief  Task: Logs flight controller data.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_Debugging(void * ptr);

/*
 * @brief  Task: Reads the on-board on/off button and turns on/off the flight controller.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_OnOffButton(void * ptr);

/*
 * @brief  Task: Reads the flight controller battery level and gives a signal whenever the level
 *               is below a threshold set by the user.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_BatteryLevel(void * ptr);

/*
 * @brief  Task: Activates an alarm whenever the battery level is below an user-defined threshold.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_BatteryAlarm(void * ptr);

/*
 * @brief  Task: Blinks an on-board-LED.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_HeartbeatLight(void * ptr);

/*
 * @brief  Task: Produces blinking sequences with the 4 flight lights.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_FlightLights(void * ptr);

/* --- Private function callback declarations ---------------------------------------------------*/
/*
 * @brief  Timer Callback: Reads the on-board on/off button and turns on/off the flight controller
 * 		                   depending on the time expiration of the system timer.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer_Callback_OnOffButton(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: Sets a flag whenever the timer has expired. It is used for the Battery
 *                         Level Alarm.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer_Callback_BatteryLevelAlarm(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: Sets a flag whenever the timer has expired. It is used for the
 *                         Flight Lights.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer_Callback_FlightLights(TimerHandle_t xTimer);

/*
 * @brief  Timer Callback: Sets a flag whenever the timer has expired. It is used for the
 *                         Control System Loop.
 * @param  TimerHandle_t structure that contains the configuration information for a FreeRTOS
 *         timer.
 * @retval None
 */
void Timer_Callback_ControlSystem(TimerHandle_t xTimer);

/*
 * @brief  Calculates an angle using a Kalman filter.
 * @param  TODO
 * @retval None
 */
void Kalman_CalculateAngle(float * kalmanState, float * kalmanUncertainty, float kalmanInput, float kalmanMeasurement);

/*
 * @brief  Calculates the PID controller output.
 * @param  PID_Output:         Pointer to a variable that hold the PID controller output value.
 *         previousIterm:      Value of the integral term of a previous control loop iteration.
 *         previousErrorValue: Value of the error of a previous control loop iteration.
 *         errorValue:         Value of the current system error.
 *         kP:				   Proportional gain.
 *         kI:                 Integral gain.
 *         kD:                 Derivative gain.
 * @retval None
 */
void CSM_CalculatePID(float * PID_Output, float * previousIterm, float * previousErrorValue, float errorValue, float kP, float kI, float kD);

/*
 * @brief  Resets the PID controller errors and integral terms values.
 * @param  None
 * @retval None
 */
void CSM_ResetPID(void);

/* --- Public variable definitions ------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
void FreeRTOS_CreateTimers(void) {

    /* Timer 2: BatteryLevelAlarm */
    Timer_Handle_BatteryLevelAlarm = xTimerCreate("BatteryLevelAlarm", pdMS_TO_TICKS(200), pdTRUE, (void *)0, Timer_Callback_BatteryLevelAlarm);
    if (NULL != Timer_Handle_BatteryLevelAlarm) {
        /* Start timer */
        xTimerStart(Timer_Handle_BatteryLevelAlarm, 0);
    }

    /* Timer 3: FlightLights */
    Timer_Handle_FlightLights = xTimerCreate("FlightLights", pdMS_TO_TICKS(100), pdTRUE, (void *)0, Timer_Callback_FlightLights);
    if (NULL != Timer_Handle_FlightLights) {
        /* Start timer */
        xTimerStart(Timer_Handle_FlightLights, 0);
    }

    /* Timer 4: ControlSystem */
    Timer_Handle_ControlSystem = xTimerCreate("ControlSystem", pdMS_TO_TICKS(1), pdTRUE, (void *)0, Timer_Callback_ControlSystem);
    if (NULL != Timer_Handle_ControlSystem) {
        /* Start timer */
        xTimerStart(Timer_Handle_ControlSystem, 0);
    }
}

void FreeRTOS_CreateQueues(void) {

    /* Queue 1: USB Communication - Logging */
    Queue_Handle_USB_Communication_Info = xQueueCreate(USB_COMMUNICATION_INFO_QUEUE_SIZE, sizeof(uint8_t *));
    configASSERT(Queue_Handle_USB_Communication_Info != NULL);

    /* Queue 2: USB Communication - Debugging */
    Queue_Handle_USB_Communication_Debug = xQueueCreate(USB_COMMUNICATION_DEBUG_QUEUE_SIZE, sizeof(uint8_t *));
    configASSERT(Queue_Handle_USB_Communication_Debug != NULL);
}

void FreeRTOS_CreateMutexes(void) {

    /* Mutex 1: Log Debugging String */
    Mutex_Handle_LogDebuggingString = xSemaphoreCreateMutex();
    configASSERT(Mutex_Handle_LogDebuggingString != NULL);

    /* Mutex 2: Log Information String */
    Mutex_Handle_LogInformationString = xSemaphoreCreateMutex();
    configASSERT(Mutex_Handle_LogInformationString != NULL);
}

void FreeRTOS_CreateTasks(void) {

    BaseType_t ret;

    /* Task 1: ControlSystem */
    ret = xTaskCreate(Task_ControlSystem, "Task_ControlSystem", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_CONTROLSYSTEM_PRIORITY), &Task_Handle_ControlSystem);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
    /* Task 2: USB_Communication */
    ret = xTaskCreate(Task_USB_Communication, "Task_USB_Communication", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_USBCOMMUNICATION_PRIORITY), &Task_Handle_USB_Communication);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);
    /* Suspend it */
    vTaskSuspend(Task_Handle_USB_Communication);

    /* Task 3: Debugging */
    ret = xTaskCreate(Task_Debugging, "Task_Debugging", (3 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_DEBUGGING_PRIORITY), &Task_Handle_Debugging);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);
    /* Suspend it */
    vTaskSuspend(Task_Handle_Debugging);
#endif

    /* Task 4: BatteryLevel */
    ret = xTaskCreate(Task_BatteryLevel, "Task_BatteryLevel", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYLEVEL_PRIORITY), &Task_Handle_BatteryLevel);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    /* Task 5: BatteryAlarm */
    ret = xTaskCreate(Task_BatteryAlarm, "Task_BatteryAlarm", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYALARM_PRIORITY), &Task_Handle_BatteryAlarm);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    /* Task 6: HeartbeatLight */
    ret = xTaskCreate(Task_HeartbeatLight, "Task_HeartbeatLight", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_HEARTBEATLIGHT_PRIORITY), &Task_Handle_HeartbeatLight);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);

    /* Task 7: FlightLights */
    ret = xTaskCreate(Task_FlightLights, "Task_FlightLights", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTLIGHTS_PRIORITY), &Task_Handle_FlightLights);
    /* Check the task was created successfully. */
    configASSERT(ret == pdPASS);
}

void Task_ControlSystem(void * ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(4);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_ControlSystem = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {

        /* Calibrate GY-87 gyroscope sensor */
        if (false == gyroscopeCalibrationIsDone) {
            GY87_gyroscopeCalibrationValues = GY87_CalibrateGyroscope(hgy87, !((bool_t)GY87_CALIBRATION_EN));
            gyroscopeCalibrationIsDone = true;
        }
        /* Calibrate GY-87 accelerometer sensor */
        if (false == accelerometerCalibrationIsDone) {
            GY87_accelerometerCalibrationValues = GY87_CalibrateAccelerometer(hgy87, !((bool_t)GY87_CALIBRATION_EN));
            accelerometerCalibrationIsDone = true;
        }
        /* Check that both sensors are calibrated */
        if (gyroscopeCalibrationIsDone && accelerometerCalibrationIsDone && false == FlightController_isInitialized) {
            FlightController_isInitialized = true;

/* Reasume supsended debugging tasks */
#if (MAIN_APP_LOGGING_DEBUGGING == 1)
            vTaskResume(Task_Handle_USB_Communication);
            vTaskResume(Task_Handle_Debugging);
#endif
        }

        /* Control system processing */
        if (FlightController_isInitialized && 0 == CONTROL_SYSTEM_MODE) {

            /* Read FS-A8S channels */
            for (uint8_t i = 0; i < FSA8S_CHANNELS; i++) {
                FSA8S_channelValues[i] = FSA8S_ReadChannel(rc_controller, channels[i]);
            }

            /* Read GY-87 gyroscope sensor */
            GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues);

            /* Read GY-87 accelerometer sensor */
            GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues);

            /* Calculate Kalman angles */
            Kalman_CalculateAngle(&Kalman_predictionValue_rollAngle, &Kalman_uncertaintyValue_rollAngle, GY87_gyroscopeValues.rotationRateRoll, GY87_accelerometerValues.angleRoll);
            Kalman_CalculateAngle(&Kalman_predictionValue_pitchAngle, &Kalman_uncertaintyValue_pitchAngle, GY87_gyroscopeValues.rotationRatePitch, GY87_accelerometerValues.anglePitch);

        } else if (FlightController_isInitialized && 1 == CONTROL_SYSTEM_MODE) {

            /* Avoid uncontrolled motor start */
            while (false == throttleStick_startedDown) {

                /* Read throttle input from radio controller */
                reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                if (15 > reference_throttle) {

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
                CSM_ResetPID();

            } else {

                /* Check if timer has expired */
                if (Timer_Flag_ControlSystem) {

                    /* Read input throttle from radio controller */
                    reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                    /* Check if throttle stick is low */
                    if (CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE > reference_throttle) {

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
                        CSM_ResetPID();

                    } else {

                        /* Read GY-87 gyroscope sensor */
                        GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues);
                        /* Read GY-87 accelerometer sensor */
                        GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues);

                        /* Calculate Kalman roll angle */
                        Kalman_CalculateAngle(&Kalman_predictionValue_rollAngle, &Kalman_uncertaintyValue_rollAngle, GY87_gyroscopeValues.rotationRateRoll, GY87_accelerometerValues.angleRoll);
                        /* Calculate Kalman pitch angle */
                        Kalman_CalculateAngle(&Kalman_predictionValue_pitchAngle, &Kalman_uncertaintyValue_pitchAngle, GY87_gyroscopeValues.rotationRatePitch, GY87_accelerometerValues.anglePitch);

                        /* Read inputs from radio controller */
                        reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
                        reference_rollValue = FSA8S_ReadChannel(rc_controller, CHANNEL_1);
                        reference_pitchValue = FSA8S_ReadChannel(rc_controller, CHANNEL_2);
                        reference_yawValue = FSA8S_ReadChannel(rc_controller, CHANNEL_4);

                        /* Adjust and limit throttle input */
                        if (CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE < reference_throttle) {
                            reference_throttle = CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE;
                        }

                        /* Calculate desired angles by mapping radio controller values to angles */
                        reference_rollAngle = 0.03 * (reference_rollValue - 500);
                        reference_pitchAngle = 0.03 * (reference_pitchValue - 500);

                        /* Calculate angles errors */
                        error_rollAngle = reference_rollAngle - Kalman_predictionValue_rollAngle;
                        error_pitchAngle = reference_pitchAngle - Kalman_predictionValue_pitchAngle;

                        /* Calculate PID for roll angle */
                        CSM_CalculatePID(&pidOutput_rollAngle, &previousIterm_rollAngle, &previousErrorValue_rollAngle, error_rollAngle, KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE);
                        /* Calculate PID for pitch angle */
                        CSM_CalculatePID(&pidOutput_pitchAngle, &previousIterm_pitchAngle, &previousErrorValue_pitchAngle, error_pitchAngle, KP_PITCH_ANGLE, KI_PITCH_ANGLE, KD_PITCH_ANGLE);

                        /* Calculate desired rates */
                        desiredValue_rollRate = pidOutput_rollAngle;
                        desiredValue_pitchRate = pidOutput_pitchAngle;
                        desiredValue_yawRate = 0.03 * (reference_yawValue - 500);

                        /* Calculate rates errors */
                        error_rollRate = desiredValue_rollRate - GY87_gyroscopeValues.rotationRateRoll;
                        error_pitchRate = desiredValue_pitchRate - GY87_gyroscopeValues.rotationRatePitch;
                        error_yawRate = desiredValue_yawRate - GY87_gyroscopeValues.rotationRateYaw;

                        /* Calculate PID for roll rate */
                        CSM_CalculatePID(&pidOutput_rollRate, &previousIterm_rollRate, &previousErrorValue_rollRate, error_rollRate, KP_ROLL_RATE, KI_ROLL_RATE, KD_ROLL_RATE);
                        /* Calculate PID for pitch rate */
                        CSM_CalculatePID(&pidOutput_pitchRate, &previousIterm_pitchRate, &previousErrorValue_pitchRate, error_pitchRate, KP_PITCH_RATE, KI_PITCH_RATE, KD_PITCH_RATE);
                        /* Calculate PID for yaw rate */
                        CSM_CalculatePID(&pidOutput_yawRate, &previousIterm_yawRate, &previousErrorValue_yawRate, error_yawRate, KP_YAW_RATE, KI_YAW_RATE, KD_YAW_RATE);

                        /* Calculate motors speed */
                        motorSpeed1 = (reference_throttle - pidOutput_rollRate - pidOutput_pitchRate - pidOutput_yawRate) / 10;
                        motorSpeed2 = (reference_throttle + pidOutput_rollRate + pidOutput_pitchRate - pidOutput_yawRate) / 10;
                        motorSpeed3 = (reference_throttle + pidOutput_rollRate - pidOutput_pitchRate + pidOutput_yawRate) / 10;
                        motorSpeed4 = (reference_throttle - pidOutput_rollRate + pidOutput_pitchRate + pidOutput_yawRate) / 10;

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
                        if (ESC_MINIMUM_SPEED > motorSpeed1)
                            motorSpeed1 = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > motorSpeed2)
                            motorSpeed2 = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > motorSpeed3)
                            motorSpeed3 = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > motorSpeed4)
                            motorSpeed4 = ESC_MINIMUM_SPEED;

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
                    Timer_Flag_ControlSystem = false;
                }
            }
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_ControlSystem = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_USB_Communication(void * ptr) {
    (void)ptr;

    /* Message pointer variables */
    uint8_t * logInformationString = NULL;
    uint8_t * logDebuggingString = NULL;

    /* Timing parameters */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(100);
    const TickType_t xQueueWait = pdMS_TO_TICKS(5);
    const TickType_t xLogRetryDelay = pdMS_TO_TICKS(5);
    const uint8_t maxLogRetries = 3;

    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_USB_Communication = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {
        /* Process info messages with mutex protection */
        if (xQueueReceive(Queue_Handle_USB_Communication_Info, &logInformationString, xQueueWait) == pdPASS) {
            /* Validate pointer before using */
            if (logInformationString != NULL) {
                /* Take mutex to ensure exclusive access to the message buffer */
                if (xSemaphoreTake(Mutex_Handle_LogInformationString, pdMS_TO_TICKS(10)) == pdTRUE) {
                    /* Send with retry limit to prevent indefinite blocking */
                    uint8_t retries = 0;
                    while (LOG((uint8_t *)logInformationString, LOG_INFORMATION) == false) {
                        retries++;
                        if (retries >= maxLogRetries) {
                            /* Log failure, could add error handling here */
                            break;
                        }
                        vTaskDelay(xLogRetryDelay);
                    }
                    /* Release mutex after processing */
                    xSemaphoreGive(Mutex_Handle_LogInformationString);
                }
            }
        }

        /* Process debug messages with same mutex protection pattern */
        if (xQueueReceive(Queue_Handle_USB_Communication_Debug, &logDebuggingString, xQueueWait) == pdPASS) {
            if (logDebuggingString != NULL) {
                /* Take mutex for debug message access */
                if (xSemaphoreTake(Mutex_Handle_LogDebuggingString, pdMS_TO_TICKS(10)) == pdTRUE) {
                    /* Send with retry limit */
                    uint8_t retries = 0;
                    while (LOG((uint8_t *)logDebuggingString, LOG_DEBUGGING) == false) {
                        retries++;
                        if (retries >= maxLogRetries) {
                            break;
                        }
                        vTaskDelay(xLogRetryDelay);
                    }
                    /* Release mutex */
                    xSemaphoreGive(Mutex_Handle_LogDebuggingString);
                }
            }
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_USB_Communication = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_Debugging(void * ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(100);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_Debugging = uxTaskGetStackHighWaterMark(NULL);
#endif

    static uint8_t debuggingStr[MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE];
    uint8_t * debuggingStrPtr = debuggingStr;
    uint32_t system_tick = 0;
    uint16_t written_chars = 0;

    while (1) {
        /* Log system time*/
        system_tick = xTaskGetTickCount();
        snprintf((char *)debuggingStr_systemTime, 16 * sizeof(uint8_t), "T_%lu", (system_tick * 1000 / configTICK_RATE_HZ));

#if (MAIN_APP_DEBUGGING_FSA8S_MAIN == 1)
        /* Log channel values */
        snprintf((char *)debuggingStr_FSA8S_main, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%d/%d_%d/%d_%d", FSA8S_CHANNEL_VALUES_1, FSA8S_channelValues[0], FSA8S_CHANNEL_VALUES_2, FSA8S_channelValues[1], FSA8S_CHANNEL_VALUES_3, FSA8S_channelValues[2],
                 FSA8S_CHANNEL_VALUES_4, FSA8S_channelValues[3], FSA8S_CHANNEL_VALUES_6, FSA8S_channelValues[5]);
#endif

#if (MAIN_APP_DEBUGGING_FSA8S_AUX == 1)
        /* Log channel values */
        snprintf((char *)debuggingStr_FSA8S_aux, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%d/%d_%d/%d_%d", FSA8S_CHANNEL_VALUES_5, FSA8S_channelValues[4], FSA8S_CHANNEL_VALUES_7, FSA8S_channelValues[6], FSA8S_CHANNEL_VALUES_8, FSA8S_channelValues[7],
                 FSA8S_CHANNEL_VALUES_9, FSA8S_channelValues[8], FSA8S_CHANNEL_VALUES_10, FSA8S_channelValues[9]);
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_CALIBRATION_VALUES)
        /* Log GY87 gyroscope calibration values */
        snprintf((char *)debuggingStr_GY87_gyroscopeCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL, GY87_gyroscopeCalibrationValues.calibrationValueRateRoll,
                 GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH, GY87_gyroscopeCalibrationValues.calibrationValueRatePitch, GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW, GY87_gyroscopeCalibrationValues.calibrationValueRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_CALIBRATION_VALUES)
        /* Log GY87 accelerometer calibration values */
        snprintf((char *)debuggingStr_GY87_accelerometerCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_ACC_CALIBRATION_VALUES_LINEAR_X, GY87_accelerometerCalibrationValues.calibrationValuelinearAccelerationX,
                 GY87_ACC_CALIBRATION_VALUES_LINEAR_Y, GY87_accelerometerCalibrationValues.calibrationValuelinearAccelerationY, GY87_ACC_CALIBRATION_VALUES_LINEAR_Z, GY87_accelerometerCalibrationValues.calibrationValuelinearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_VALUES == 1)
        /* Log GY87 gyroscope values */
        snprintf((char *)debuggingStr_GY87_gyroscopeValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", GY87_GYRO_VALUES_ROT_RATE_ROLL, GY87_gyroscopeValues.rotationRateRoll, GY87_GYRO_VALUES_ROT_RATE_PITCH,
                 GY87_gyroscopeValues.rotationRatePitch, GY87_GYRO_VALUES_ROT_RATE_YAW, GY87_gyroscopeValues.rotationRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_VALUES == 1)
        /* Log GY87 accelerometer values */
        snprintf((char *)debuggingStr_GY87_accelerometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_ACC_VALUES_LINEAR_X, GY87_accelerometerValues.linearAccelerationX, GY87_ACC_VALUES_LINEAR_Y,
                 GY87_accelerometerValues.linearAccelerationY, GY87_ACC_VALUES_LINEAR_Z, GY87_accelerometerValues.linearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_ANGLES == 1)
        /* Log GY87 accelerometer angles */
        snprintf((char *)debuggingStr_GY87_accelerometerAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f", GY87_ACC_VALUES_ANGLE_ROLL, GY87_accelerometerValues.angleRoll, GY87_ACC_VALUES_ANGLE_PITCH, GY87_accelerometerValues.anglePitch);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES == 1)
        /* Read GY87 magnetometer values */
        GY87_ReadMagnetometer(hgy87, &GY87_magnetometerValues);
        /* Log GY87 magnetometer values */
        snprintf((char *)debuggingStr_GY87_magnetometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_MAG_VALUES_MAG_FIELD_X, GY87_magnetometerValues.magneticFieldX, GY87_MAG_VALUES_MAG_FIELD_Y,
                 GY87_magnetometerValues.magneticFieldY, GY87_MAG_VALUES_MAG_FIELD_Z, GY87_magnetometerValues.magneticFieldZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING == 1)
        /* Read GY87 magnetometer heading */
        GY87_magnetometerHeadingValue = GY87_ReadMagnetometerHeading(hgy87);
        /* Log GY87 magnetometer heading */
        snprintf((char *)debuggingStr_GY87_magnetometerHeadingValue, 16 * sizeof(uint8_t), (const char *)"/%d_%.2f", GY87_MAG_HEADING, GY87_magnetometerHeadingValue);
#endif

#if (MAIN_APP_DEBUGGING_GY87_TEMPERATURE == 1)
        /* Read GY87 temperature sensor */
        GY87_temperature = GY87_ReadTemperatureSensor(hgy87);
        /*  Log GY87 temperature value */
        snprintf((char *)debuggingStr_GY87_temperature, 16 * sizeof(uint8_t), (const char *)"/%d_%.1f", GY87_TEMPERATURE, GY87_temperature);
#endif

#if (MAIN_APP_DEBUGGING_ESCS == 1)
        /* Log ESC values*/
        snprintf((char *)debuggingStr_ESCs, 40 * sizeof(uint8_t), (const char *)"/%d_%d/%d_%d/%d_%d/%d_%d", ESC_1, (uint16_t)ESC_speeds[0], ESC_2, (uint16_t)ESC_speeds[1], ESC_3, (uint16_t)ESC_speeds[2], ESC_4, (uint16_t)ESC_speeds[3]);
#endif

#if (MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL == 1)
        /* Log flight controller battery level */
        snprintf((char *)debuggingStr_BatteryLevel, 10 * sizeof(uint8_t), (const char *)"/%d_%.2f", FLIGHT_CONTROLLER_BATTERY_LEVEL, (double)FlightController_batteryLevelValue);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES == 1)
        /* Log control system reference values */
        snprintf((char *)debuggingStr_ControlSystem_ReferenceAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_REFERENCE_THROTTLE, reference_throttle, CONTROLSYSTEM_REFERENCE_ROLL_ANGLE, reference_rollAngle,
                 CONTROLSYSTEM_REFERENCE_PITCH_ANGLE, reference_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_KALMAN_ANGLES == 1)
        /* Log control system angles */
        snprintf((char *)debuggingStr_ControlSystem_KalmanAngles, 30 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f", CONTROLSYSTEM_KALMAN_ROLL_ANGLE, Kalman_predictionValue_rollAngle, CONTROLSYSTEM_KALMAN_PITCH_ANGLE,
                 Kalman_predictionValue_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_ERRORS == 1)
        /* Log control system angles errors */
        snprintf((char *)debuggingStr_ControlSystem_AnglesErrors, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f", CONTROLSYSTEM_ERROR_ROLL_ANGLE, error_rollAngle, CONTROLSYSTEM_ERROR_PITCH_ANGLE, error_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_PID == 1)
        /* Log control system angles PID */
        snprintf((char *)debuggingStr_ControlSystem_AnglesPID, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f", CONTROLSYSTEM_PID_OUTPUT_ROLL_ANGLE, pidOutput_rollAngle, CONTROLSYSTEM_PID_OUTPUT_PITCH_ANGLE, pidOutput_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_RATE == 1)
        /* Log control system reference rates */
        snprintf((char *)debuggingStr_ControlSystem_ReferenceRates, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_REFERENCE_ROLL_RATE, desiredValue_rollRate, CONTROLSYSTEM_REFERENCE_PITCH_RATE, desiredValue_pitchRate,
                 CONTROLSYSTEM_REFERENCE_YAW_RATE, desiredValue_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_ERRORS == 1)
        /* Log control system rates errors */
        snprintf((char *)debuggingStr_ControlSystem_RatesErrors, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_ERROR_ROLL_RATE, error_rollRate, CONTROLSYSTEM_ERROR_PITCH_RATE, error_pitchRate,
                 CONTROLSYSTEM_ERROR_YAW_RATE, error_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_PID == 1)
        /* Log control system rates PID */
        snprintf((char *)debuggingStr_ControlSystem_RatesPID, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_PID_OUTPUT_ROLL_RATE, pidOutput_rollRate, CONTROLSYSTEM_PID_OUTPUT_PITCH_RATE, pidOutput_pitchRate,
                 CONTROLSYSTEM_PID_OUTPUT_YAW_RATE, pidOutput_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK)
        /* Log tasks stack high watermark */
        snprintf((char *)debuggingStr_TasksStackHighWatermark, 80 * sizeof(uint8_t), (const char *)"/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld", TASK_STACK_WATERMARK_ONOFFBUTTON, Task_StackHighWatermark_OnOffButton,
                 TASK_STACK_WATERMARK_CONTROLSYSTEM, Task_StackHighWatermark_ControlSystem, TASK_STACK_WATERMARK_USBCOMMUNICATION, Task_StackHighWatermark_USB_Communication, TASK_STACK_WATERMARK_DEBUGGING, Task_StackHighWatermark_Debugging,
                 TASK_STACK_WATERMARK_BATTERYLEVEL, Task_StackHighWatermark_BatteryLevel, TASK_STACK_WATERMARK_BATTERYALARM, Task_StackHighWatermark_BatteryAlarm, TASK_STACK_WATERMARK_HEARTBEATLIGHT, Task_StackHighWatermark_HeartbeatLight,
                 TASK_STACK_WATERMARK_FLIGHTLIGHTS, Task_StackHighWatermark_FlightLights);
#endif

        /* Concatenate all debugging strings and send to queue */
        xSemaphoreTake(Mutex_Handle_LogDebuggingString, xTaskPeriod);
        {
            memset(debuggingStr, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE * sizeof(uint8_t));
            written_chars = 0;
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_systemTime);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_FSA8S_main);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_FSA8S_aux);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_gyroscopeCalibrationValues);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_gyroscopeValues);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerCalibrationValues);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerValues);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerAngles);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_magnetometerValues);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_magnetometerHeadingValue);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_temperature);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ESCs);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_BatteryLevel);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ReferenceAngles);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_KalmanAngles);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_AnglesErrors);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_AnglesPID);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ReferenceRates);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_RatesErrors);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_RatesPID);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_TasksStackHighWatermark);
            written_chars += snprintf((char *)debuggingStr + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "\n");

            xQueueSend(Queue_Handle_USB_Communication_Debug, &debuggingStrPtr, xTaskPeriod);
        }
        xSemaphoreGive(Mutex_Handle_LogDebuggingString);

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_Debugging = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_OnOffButton(void * ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_OnOffButton = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {

        /* Check On/Off Button status */
        if (!HAL_GPIO_ReadPin(PW_ON_OFF_DRIVER_INPUT_GPIO_Port, PW_ON_OFF_DRIVER_INPUT_Pin)) {
            /* User is trying to turn it on or off */
            if (!Timer_Flag_OnOffButton) {

                xTimerStart(Timer_Handle_OnOffButton, 0);
                Timer_Flag_OnOffButton = true;
            }
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get initial stack watermark */
        Task_StackHighWatermark_OnOffButton = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelay(xTaskPeriod);
    }
}

void Task_BatteryLevel(void * ptr) {
    (void)ptr;

    uint16_t adcValue;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(1000);

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_BatteryLevel = uxTaskGetStackHighWaterMark(NULL);
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
        FlightController_batteryLevelValue = FlightController_batteryLevelValue * 3.363636 + BATTERY_LEVEL_CALIBRATION_OFFSET;

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get initial stack watermark */
        Task_StackHighWatermark_BatteryLevel = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelay(xTaskPeriod);
    }
}

void Task_BatteryAlarm(void * ptr) {
    (void)ptr;

    uint8_t alarmSequence[] = {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t alarmSequenceSize = sizeof(alarmSequence);
    uint8_t alarmSequenceCursor = 0;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_BatteryAlarm = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {

        if (FlightController_batteryLevelValue < BATTERY_ALARM_THRESHOLD) {

            if (Timer_Flag_BatteryLevelAlarm) {
                /* If timer expired */
                /* Parse alarm sequence */
                alarmSequenceCursor++;
                if (alarmSequenceSize <= alarmSequenceCursor) {
                    alarmSequenceCursor = 0;
                }

                /* Write to buzzer */
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, alarmSequence[alarmSequenceCursor]);

                /* Reset Timer2 flag */
                Timer_Flag_BatteryLevelAlarm = false;
            }

        } else {

            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get initial stack watermark */
        Task_StackHighWatermark_BatteryAlarm = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelay(xTaskPeriod);
    }
}

void Task_HeartbeatLight(void * ptr) {
    (void)ptr;

    uint8_t ledState = GPIO_PIN_RESET;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(HEARTBEAT_PERIOD / 2);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_HeartbeatLight = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {

        if (FlightController_isInitialized) {
            /* Toggle LED and update state */
            ledState = (ledState == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get initial stack watermark */
        Task_StackHighWatermark_HeartbeatLight = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_FlightLights(void * ptr) {
    (void)ptr;

    typedef struct {
        uint8_t led1[8];
        uint8_t led2[8];
        uint8_t led3[8];
        uint8_t led4[8];
        uint8_t size;
    } LightSequence;

    /* Consolidated sequence definitions */
    static const LightSequence sequences[3] = {/* Sequence A */
                                               {
                                                   {1, 0, 0, 0, 0, 0, 0, 0}, /* LED1 */
                                                   {0, 0, 1, 0, 0, 0, 0, 0}, /* LED2 */
                                                   {1, 0, 0, 0, 0, 0, 0, 0}, /* LED3 */
                                                   {0, 0, 1, 0, 0, 0, 0, 0}, /* LED4 */
                                                   8                         /* size */
                                               },
                                               /* Sequence B */
                                               {
                                                   {1, 0, 1, 0, 0, 0, 0, 0}, /* LED1 */
                                                   {0, 0, 0, 0, 1, 0, 1, 0}, /* LED2 */
                                                   {1, 0, 1, 0, 0, 0, 0, 0}, /* LED3 */
                                                   {0, 0, 0, 0, 1, 0, 1, 0}, /* LED4 */
                                                   8                         /* size */
                                               },
                                               /* Sequence C */
                                               {
                                                   {1, 0, 1, 0, 0, 0, 0, 0}, /* LED1 */
                                                   {0, 0, 0, 0, 1, 0, 0, 0}, /* LED2 */
                                                   {1, 0, 1, 0, 0, 0, 0, 0}, /* LED3 */
                                                   {0, 0, 0, 0, 1, 0, 0, 0}, /* LED4 */
                                                   8                         /* size */
                                               }};

    const uint16_t CHANNEL_THRESHOLD_LOW = 250;
    const uint16_t CHANNEL_THRESHOLD_MID = 750;
    const uint16_t LIGHTS_ENABLE_THRESHOLD = 500;

    uint8_t activeSequence = 0;
    uint8_t sequenceCursor = 0;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
    /* Get initial stack watermark */
    Task_StackHighWatermark_FlightLights = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (1) {
        /* Flight lights enabled check */
        if (FSA8S_channelValues[9] >= LIGHTS_ENABLE_THRESHOLD) {
            /* Select sequence based on channel 8 value */
            if (FSA8S_channelValues[8] <= CHANNEL_THRESHOLD_LOW) {
                activeSequence = 0;
            } else if (FSA8S_channelValues[8] <= CHANNEL_THRESHOLD_MID) {
                activeSequence = 1;
            } else {
                activeSequence = 2;
            }

            /* Adjust sequence speed based on potentiometer */
            Timer_AutoReloadTime_FlightLights = 200 + FSA8S_channelValues[7] / 5;

            /* Update lights if timer expired */
            if (Timer_Flag_FlightLights) {
                /* Advance sequence with bounds check */
                sequenceCursor = (sequenceCursor + 1) % sequences[activeSequence].size;

                /* Update all LEDs */
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, sequences[activeSequence].led1[sequenceCursor]);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, sequences[activeSequence].led2[sequenceCursor]);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, sequences[activeSequence].led3[sequenceCursor]);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, sequences[activeSequence].led4[sequenceCursor]);

                /* Reset timer flag */
                Timer_Flag_FlightLights = false;
            }
        } else {
            /* Turn off all lights when disabled */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        Task_StackHighWatermark_FlightLights = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Use vTaskDelayUntil for precise timing */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Kalman_CalculateAngle(float * kalmanState, float * kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {

    // float kalmanGain;

    // *kalmanState = *kalmanState + CONTROL_SYSTEM_LOOP_PERIOD_S * kalmanInput;
    // *kalmanUncertainty = *kalmanUncertainty + CONTROL_SYSTEM_LOOP_PERIOD_S * CONTROL_SYSTEM_LOOP_PERIOD_S * 4 * 4;
    // kalmanGain = *kalmanUncertainty * 1 / (1 * *kalmanUncertainty + 3 * 3);
    // *kalmanState = *kalmanState + kalmanGain * (kalmanMeasurement - *kalmanState);
    // *kalmanUncertainty = (1 - kalmanGain) * *kalmanUncertainty;

    /* Process noise variance (Q) - how much we trust the gyro */
    const float processNoise = 16.0f; // 4^2
    /* Measurement noise variance (R) - how much we trust the accelerometer */
    const float measurementNoise = 9.0f; // 3^2
    float kalmanGain;

    /* Prediction step - Project the state ahead using gyro data */
    *kalmanState = *kalmanState + CONTROL_SYSTEM_LOOP_PERIOD_S * kalmanInput;

    /* Project the error covariance ahead */
    *kalmanUncertainty = *kalmanUncertainty + (CONTROL_SYSTEM_LOOP_PERIOD_S * CONTROL_SYSTEM_LOOP_PERIOD_S * processNoise);

    /* Ensure uncertainty doesn't become negative due to numerical errors */
    if (*kalmanUncertainty < 0.0f) {
        *kalmanUncertainty = 0.001f;
    }

    /* Update step - Compute the Kalman gain */
    kalmanGain = *kalmanUncertainty / (*kalmanUncertainty + measurementNoise);

    /* Clamp Kalman gain to valid range */
    if (kalmanGain < 0.0f)
        kalmanGain = 0.0f;
    if (kalmanGain > 1.0f)
        kalmanGain = 1.0f;

    /* Update estimate with measurement */
    *kalmanState = *kalmanState + kalmanGain * (kalmanMeasurement - *kalmanState);

    /* Update the error covariance */
    *kalmanUncertainty = (1.0f - kalmanGain) * *kalmanUncertainty;
}

void CSM_CalculatePID(float * PID_Output, float * previousIterm, float * previousErrorValue, float errorValue, float kP, float kI, float kD) {

    float Pterm;
    float Iterm;
    float Dterm;
    float pidOutputValue;

    /* Calculate proportional term */
    Pterm = kP * errorValue;

    /* Calculate integral term */
    Iterm = *previousIterm + kI * ((*previousErrorValue + errorValue) / 2) * CONTROL_SYSTEM_LOOP_PERIOD_S;
    /* Clamp integral term to avoid integral wind-up */
    if (-CONTROL_SYSTEM_PID_ITERM_LIMIT > Iterm) {
        Iterm = -CONTROL_SYSTEM_PID_ITERM_LIMIT;
    } else if (CONTROL_SYSTEM_PID_ITERM_LIMIT < Iterm) {
        Iterm = CONTROL_SYSTEM_PID_ITERM_LIMIT;
    }

    /* Calculate derivative term */
    Dterm = kD * (errorValue - *previousErrorValue) / CONTROL_SYSTEM_LOOP_PERIOD_S;

    /* Calculate PID output */
    pidOutputValue = Pterm + Iterm + Dterm;
    /* Limit the PID output */
    if (-CONTROL_SYSTEM_PID_OUTPUT_LIMIT > pidOutputValue) {
        pidOutputValue = -CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    } else if (CONTROL_SYSTEM_PID_OUTPUT_LIMIT < pidOutputValue) {
        pidOutputValue = CONTROL_SYSTEM_PID_OUTPUT_LIMIT;
    }

    /* Return values */
    *PID_Output = pidOutputValue;
    *previousErrorValue = errorValue;
    *previousIterm = Iterm;
}

void CSM_ResetPID(void) {

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
}

/* --- Private callback function implementation ------------------------------------------------ */
void Timer_Callback_OnOffButton(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer_AutoReloadTime_OnOffButton) / xTimerPeriod)) {
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
                vTaskSuspend(Task_Handle_HeartbeatLight);
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
        Timer_Flag_OnOffButton = false;
    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer_Callback_BatteryLevelAlarm(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer_AutoReloadTime_BatteryLevelAlarm) / xTimerPeriod)) {

        /* Set Timer2 flag to true */
        Timer_Flag_BatteryLevelAlarm = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer_Callback_FlightLights(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer_AutoReloadTime_FlightLights) / xTimerPeriod)) {

        /* Set Timer3 flag to true */
        Timer_Flag_FlightLights = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

void Timer_Callback_ControlSystem(TimerHandle_t xTimer) {

    /* Get no. of times this timer has expired */
    uint32_t ulCount = (uint32_t)pvTimerGetTimerID(xTimer);

    /* Get timer period */
    uint32_t xTimerPeriod = xTimerGetPeriod(xTimer);

    /* Increment the count */
    ulCount++;

    if (ulCount >= (pdMS_TO_TICKS(Timer_AutoReloadTime_ControlSystem) / xTimerPeriod)) {

        /* Set Timer3 flag to true */
        Timer_Flag_ControlSystem = true;

        /* Reset timer count */
        vTimerSetTimerID(xTimer, (void *)0);

    } else {
        /* Store the incremented count back into the timer's ID */
        vTimerSetTimerID(xTimer, (void *)ulCount);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    // /* Task: FlightController_OnOffButton */
    // BaseType_t ret = xTaskCreate(FlightController_OnOffButton, "FlightController_OnOffButton", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_ONOFFBUTTON_PRIORITY), &Task_Handle_OnOffButton);
    // /* Check the task was created successfully. */
    // configASSERT(ret == pdPASS);

    /* Timer1: OnOffButton */
    Timer_Handle_OnOffButton = xTimerCreate("OnOffButton", pdMS_TO_TICKS(100), pdTRUE, (void *)0, Timer_Callback_OnOffButton);

    /* Turn on-board LED off */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

    /* Check if flight controller is already running */
    FlightController_isRunning = 1; // DELETE THIS LINE
    if (FlightController_isRunning) {

        /* Initialize drivers */
        rc_controller = FSA8S_Init(&huart2);
        if (NULL == rc_controller) {
            /* Turn on-board LED off */
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
            while (1)
                ;
        }

        hgy87 = GY87_Init(&hi2c1);
        if (NULL == hgy87) {
            /* Turn on-board LED off */
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
            while (1)
                ;
        }

        hesc = ESC_Init(&htim3);
        if (NULL == hesc) {
            /* Turn on-board LED off */
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
            while (1)
                ;
        }

        /* Create system timers, queues, mutexes and tasks */
        FreeRTOS_CreateTimers();
        FreeRTOS_CreateQueues();
        FreeRTOS_CreateMutexes();
        FreeRTOS_CreateTasks();
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
