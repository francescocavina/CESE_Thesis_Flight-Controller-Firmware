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
#include "error_led.h"
#include "control_system_support.h"
#include "user_settings.h"
#include "control_system_settings.h"
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
#define TASK_CONTROLSYSTEM_PRIORITY    (6)
#define TASK_USBCOMMUNICATION_PRIORITY (5)
#define TASK_DEBUGGING_PRIORITY        (4)
#define TASK_BATTERYLEVEL_PRIORITY     (2)
#define TASK_BATTERYALARM_PRIORITY     (2)
#define TASK_HEARTBEATLIGHT_PRIORITY   (2)
#define TASK_FLIGHTLIGHTS_PRIORITY     (2)
/* Logging and Debugging Settings */
#define MAIN_APP_LOGGING_DEBUGGING                               (1)
#define MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE                   (1000)
#define MAIN_APP_DEBUGGING_FSA8S_MAIN                            (1)
#define MAIN_APP_DEBUGGING_FSA8S_AUX                             (1)
#define MAIN_APP_DEBUGGING_GY87_GYROSCOPE_CALIBRATION_VALUES     (1)
#define MAIN_APP_DEBUGGING_GY87_GYROSCOPE_VALUES                 (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_CALIBRATION_VALUES (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_VALUES             (1)
#define MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_ANGLES             (1)
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES              (1) // Not necessary
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING             (1) // Not necessary
#define MAIN_APP_DEBUGGING_GY87_TEMPERATURE                      (1) // Not necessary
#define MAIN_APP_DEBUGGING_ESCS                                  (1)
#define MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL       (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES        (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_KALMAN_ANGLES           (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_ERRORS           (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_PID              (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_RATE          (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_ERRORS            (0)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_PID               (0)
#define MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK             (1)
/* Drivers Settings */
#define USB_COMMUNICATION_INFO_QUEUE_SIZE  (32)
#define USB_COMMUNICATION_DEBUG_QUEUE_SIZE (1)

/* Battery Level Settings */
#define BATTERY_LEVEL_CALIBRATION_OFFSET (0.76)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Flight Controller State */
static bool_t FlightController_isRunning = false;
static bool_t FlightController_isInitialized = false;

/* Drivers Handles */
static IBUS_HandleTypeDef_t * rc_controller = NULL;
static GY87_HandleTypeDef_t * hgy87 = NULL;
static ESC_HandleTypeDef_t * hesc = NULL;

/* Timers Handles */
static TimerHandle_t Timer_Handle_OnOffButton = NULL;
/* Queues Handles */
static QueueHandle_t Queue_Handle_ControlSystemValues_Debug = NULL; // Queue to send pointers to Task_Debugging with Task_ControlSystem variables
static QueueHandle_t Queue_Handle_USB_Communication_Info = NULL;    // Queue to send pointers to Task_USB_Communication with information strings
static QueueHandle_t Queue_Handle_USB_Communication_Debug = NULL;   // Queue to send pointers to Task_USB_Communication with debugging strings
static QueueHandle_t Queue_Handle_FlightLights_Commands = NULL;     // Queue to send pointers to Task_FlightLights with flight lights radio controller commands
static QueueHandle_t Queue_Handle_BatteryLevel = NULL;              // Queue to send battery level from Task_BatteryLevel to Task_BatteryAlarm
/* Semaphores Handles */
static SemaphoreHandle_t Semaphore_Handle_controlSystemValuesSwap = NULL;
static SemaphoreHandle_t Semaphore_Handle_debuggingBufferSwap = NULL;
static SemaphoreHandle_t Semaphore_Handle_flightLightsBufferSwap = NULL;

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
static uint16_t Timer_AutoReloadTime_OnOffButton = PW_ON_OFF_DRIVER_TIME;

/* Control System Variables Debugging */
static ControlSystemValues_t controlSystemValues_A;
static ControlSystemValues_t controlSystemValues_B;
static ControlSystemValues_t * controlSystemActiveValues_TaskControlSystem = &controlSystemValues_A; // Task_ControlSystem writes here
static ControlSystemValues_t * controlSystemActiveValues_TaskDebugging = &controlSystemValues_B;     // Task_Debugging reads from here

/* Flight Lights Commands */
static uint16_t FlightLights_Buffer_A[3] = {0};
static uint16_t FlightLights_Buffer_B[3] = {0};
static uint16_t * FlightLightsActiveBuffer_TaskControlSystem = FlightLights_Buffer_A; // Task_ControlSystem writes here
static uint16_t * FlightLightsActiveBuffer_TaskFlightLights = FlightLights_Buffer_B;  // Task_FlightLights reads from here here

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
static uint8_t debuggingStr_A[MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE];
static uint8_t debuggingStr_B[MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE];
static uint8_t * debuggingActiveBuffer_TaskDebugging = debuggingStr_A;        // Task_Debugging writes here
static uint8_t * debuggingActiveBuffer_TaskUSBCommunication = debuggingStr_B; // Task_USB_Communication reads from here
static uint8_t debuggingStr_SystemTime[16] = {0};                             // Size checked
static uint8_t debuggingStr_FSA8S_main[50] = {0};                             // Size checked
static uint8_t debuggingStr_FSA8S_aux[50] = {0};                              // Size checked
static uint8_t debuggingStr_GY87_gyroscopeCalibrationValues[40] = {0};        // Size checked
static uint8_t debuggingStr_GY87_gyroscopeValues[40] = {0};                   // Size checked
static uint8_t debuggingStr_GY87_accelerometerCalibrationValues[40] = {0};    // Size checked
static uint8_t debuggingStr_GY87_accelerometerValues[40] = {0};               // Size checked
static uint8_t debuggingStr_GY87_accelerometerAngles[40] = {0};               // Size checked
static uint8_t debuggingStr_GY87_magnetometerValues[40] = {0};                // Size checked
static uint8_t debuggingStr_GY87_magnetometerHeadingValue[16] = {0};          // Size checked
static uint8_t debuggingStr_GY87_temperature[16] = {0};                       // Size checked
static uint8_t debuggingStr_ESCs[40] = {0};                                   // Size checked
static uint8_t debuggingStr_BatteryLevel[10] = {0};                           // Size checked
static uint8_t debuggingStr_ControlSystem_ReferenceAngles[40] = {0};
static uint8_t debuggingStr_ControlSystem_KalmanAngles[30] = {0}; // Size checked
static uint8_t debuggingStr_ControlSystem_AnglesErrors[40] = {0};
static uint8_t debuggingStr_ControlSystem_AnglesPID[40] = {0};
static uint8_t debuggingStr_ControlSystem_ReferenceRates[40] = {0};
static uint8_t debuggingStr_ControlSystem_RatesErrors[40] = {0};
static uint8_t debuggingStr_ControlSystem_RatesPID[40] = {0};
static uint8_t debuggingStr_TasksStackHighWatermark[80] = {0}; // Size checked

/* FS-A8S Radio Controller Variables */
static FSA8S_CHANNEL_t channels[FSA8S_CHANNELS] = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10};
static uint16_t FSA8S_channelValues[FSA8S_CHANNELS] = {0};

/* GY87 IMU Variables */
static bool_t gyroscopeCalibrationIsDone = false;
static GY87_gyroscopeCalibrationValues_t GY87_gyroscopeCalibrationValues = {false, true, 0.0, 0.0, 0.0};
static GY87_gyroscopeValues_t GY87_gyroscopeValues;
static bool_t accelerometerCalibrationIsDone = false;
static GY87_accelerometerCalibrationValues_t GY87_accelerometerCalibrationValues = {false, true, 0.0, 0.0, 0.0};
static GY87_accelerometerValues_t GY87_accelerometerValues;
static GY87_magnetometerValues_t GY87_magnetometerValues;
static float GY87_magnetometerHeadingValue = 0;
static float GY87_temperature = 0;

/* Control System Mode 1 */
/* ESCs Values*/
uint8_t ESC_speeds[4] = {0};
/* Throttle stick check */
static bool_t throttleStick_startedDown = false;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Creates system software timers.
 * @param  None
 * @retval true if the timers are created successfully, false otherwise.
 */
bool_t FreeRTOS_CreateTimers(void);

/*
 * @brief  Creates system software queues.
 * @param  None
 * @retval true if the queues are created successfully, false otherwise.
 */
bool_t FreeRTOS_CreateQueues(void);

/*
 * @brief  Creates system software semaphores.
 * @param  None
 * @retval true if the semaphores are created successfully, false otherwise.
 */
bool_t FreeRTOS_CreateSemaphores(void);

/*
 * @brief  Creates other system tasks.
 * @param  None
 * @retval true if the tasks are created successfully, false otherwise.
 */
bool_t FreeRTOS_CreateTasks(void);

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

/* --- Public variable definitions ------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
bool_t FreeRTOS_CreateTimers(void) {

    /* Successfully created all timers */
    return true;
}

bool_t FreeRTOS_CreateQueues(void) {

    /* Queue 1: Control System Values - Debugging */
    Queue_Handle_ControlSystemValues_Debug = xQueueCreate(1, sizeof(ControlSystemValues_t *));
    if (Queue_Handle_ControlSystemValues_Debug == NULL) {
        return false;
    }

    /* Queue 2: USB Communication - Logging */
    Queue_Handle_USB_Communication_Info = xQueueCreate(USB_COMMUNICATION_INFO_QUEUE_SIZE, sizeof(uint8_t *));
    if (Queue_Handle_USB_Communication_Info == NULL) {
        return false;
    }

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
    /* Queue 3: USB Communication - Debugging */
    Queue_Handle_USB_Communication_Debug = xQueueCreate(USB_COMMUNICATION_DEBUG_QUEUE_SIZE, sizeof(uint8_t *));
    if (Queue_Handle_USB_Communication_Debug == NULL) {
        return false;
    }
#endif

    /* Queue 4: Flight Lights - Commands */
    Queue_Handle_FlightLights_Commands = xQueueCreate(1, sizeof(uint16_t *));
    if (Queue_Handle_FlightLights_Commands == NULL) {
        return false;
    }

    /* Queue 5: Battery Level */
    Queue_Handle_BatteryLevel = xQueueCreate(1, sizeof(float));
    if (Queue_Handle_BatteryLevel == NULL) {
        return false;
    }

    /* Successfully created all queues */
    return true;
}

bool_t FreeRTOS_CreateSemaphores(void) {

    /* Semaphore 1: Control System Values */
    Semaphore_Handle_controlSystemValuesSwap = xSemaphoreCreateBinary();
    if (Semaphore_Handle_controlSystemValuesSwap == NULL) {
        return false;
    }
    /* Initialize the semaphore given */
    xSemaphoreGive(Semaphore_Handle_controlSystemValuesSwap);

    /* Semaphore 2: Debugging */
    Semaphore_Handle_debuggingBufferSwap = xSemaphoreCreateBinary();
    if (Semaphore_Handle_debuggingBufferSwap == NULL) {
        return false;
    }
    /* Initialize the semaphore given */
    xSemaphoreGive(Semaphore_Handle_debuggingBufferSwap);

    /* Semaphore 3: Flight Lights Commands */
    Semaphore_Handle_flightLightsBufferSwap = xSemaphoreCreateBinary();
    if (Semaphore_Handle_flightLightsBufferSwap == NULL) {
        return false;
    }
    xSemaphoreGive(Semaphore_Handle_flightLightsBufferSwap);

    /* Successfully created all mutexes */
    return true;
}

bool_t FreeRTOS_CreateTasks(void) {

    /* Task 1: ControlSystem */
    xTaskCreate(Task_ControlSystem, "Task_ControlSystem", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_CONTROLSYSTEM_PRIORITY), &Task_Handle_ControlSystem);
    if (Task_Handle_ControlSystem == NULL) {
        return false;
    }

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
    /* Task 2: USB_Communication */
    xTaskCreate(Task_USB_Communication, "Task_USB_Communication", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_USBCOMMUNICATION_PRIORITY), &Task_Handle_USB_Communication);
    if (Task_Handle_USB_Communication == NULL) {
        return false;
    }

    /* Task 3: Debugging */
    xTaskCreate(Task_Debugging, "Task_Debugging", (3 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_DEBUGGING_PRIORITY), &Task_Handle_Debugging);
    if (Task_Handle_Debugging == NULL) {
        return false;
    }
#endif

    /* Task 4: BatteryLevel */
    xTaskCreate(Task_BatteryLevel, "Task_BatteryLevel", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYLEVEL_PRIORITY), &Task_Handle_BatteryLevel);
    if (Task_Handle_BatteryLevel == NULL) {
        return false;
    }

    /* Task 5: BatteryAlarm */
    xTaskCreate(Task_BatteryAlarm, "Task_BatteryAlarm", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYALARM_PRIORITY), &Task_Handle_BatteryAlarm);
    if (Task_Handle_BatteryAlarm == NULL) {
        return false;
    }

    /* Task 6: HeartbeatLight */
    xTaskCreate(Task_HeartbeatLight, "Task_HeartbeatLight", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_HEARTBEATLIGHT_PRIORITY), &Task_Handle_HeartbeatLight);
    if (Task_Handle_HeartbeatLight == NULL) {
        return false;
    }

    /* Task 7: FlightLights */
    xTaskCreate(Task_FlightLights, "Task_FlightLights", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTLIGHTS_PRIORITY), &Task_Handle_FlightLights);
    if (Task_Handle_FlightLights == NULL) {
        return false;
    }

    /* Successfully created all tasks */
    return true;
}

void Task_ControlSystem(void * ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(4);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {

        /* Calibrate GY-87 gyroscope sensor */
        if (false == gyroscopeCalibrationIsDone) {
            GY87_CalibrateGyroscope(hgy87, &GY87_gyroscopeCalibrationValues, !((bool_t)GY87_CALIBRATION_EN));
            gyroscopeCalibrationIsDone = GY87_gyroscopeCalibrationValues.calibrationDone;
        }
        /* Calibrate GY-87 accelerometer sensor */
        if (false == accelerometerCalibrationIsDone) {
            GY87_CalibrateAccelerometer(hgy87, &GY87_accelerometerCalibrationValues, !((bool_t)GY87_CALIBRATION_EN));
            accelerometerCalibrationIsDone = GY87_accelerometerCalibrationValues.calibrationDone;
        }
        /* Check that both sensors are calibrated */
        if (gyroscopeCalibrationIsDone && accelerometerCalibrationIsDone && false == FlightController_isInitialized) {
            FlightController_isInitialized = true;
        }

        /* Control system processing */
        if (FlightController_isInitialized && 0 == CONTROL_SYSTEM_MODE) {

            /* Read FS-A8S channels */
            for (uint8_t i = 0; i < FSA8S_CHANNELS; i++) {
                FSA8S_channelValues[i] = FSA8S_ReadChannel(rc_controller, channels[i]);
            }

            // /* Read GY-87 gyroscope sensor */
            GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues, &GY87_gyroscopeCalibrationValues);

            // /* Read GY-87 accelerometer sensor */
            GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues, &GY87_accelerometerCalibrationValues);

/* Read GY-87 magnetometer values */
#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES == 1)
            GY87_ReadMagnetometer(hgy87, &GY87_magnetometerValues);
#endif

/* Read GY-87 magnetometer heading */
#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING == 1)
            GY87_magnetometerHeadingValue = GY87_ReadMagnetometerHeading(hgy87);
#endif

/* Read GY-87 temperature sensor */
#if (MAIN_APP_DEBUGGING_GY87_TEMPERATURE == 1)
            GY87_temperature = GY87_ReadTemperatureSensor(hgy87);
#endif

            // /* Calculate Kalman angles */
            // Kalman_CalculateAngle(&Kalman_predictionValue_rollAngle, &Kalman_uncertaintyValue_rollAngle, GY87_gyroscopeValues.rotationRateRoll, GY87_accelerometerValues.angleRoll);
            // Kalman_CalculateAngle(&Kalman_predictionValue_pitchAngle, &Kalman_uncertaintyValue_pitchAngle, GY87_gyroscopeValues.rotationRatePitch, GY87_accelerometerValues.anglePitch);

        } else if (FlightController_isInitialized && 1 == CONTROL_SYSTEM_MODE) {
        }

        /* Clear buffer */
        memset(controlSystemActiveValues_TaskControlSystem, 0, sizeof(ControlSystemValues_t));

        /* Radio Controller Readings */
        for (int8_t i = 0; i < FSA8S_CHANNELS; i++) {
            controlSystemActiveValues_TaskControlSystem->radioController_channelValues[i] = FSA8S_channelValues[i];
        }
        /* IMU Calibration (Gyroscope) */
        controlSystemActiveValues_TaskControlSystem->gyroCalibration_calibrationDone = GY87_gyroscopeCalibrationValues.calibrationDone;
        controlSystemActiveValues_TaskControlSystem->gyroCalibration_fixedCalibration_en = GY87_gyroscopeCalibrationValues.fixedCalibration_en;
        controlSystemActiveValues_TaskControlSystem->gyroCalibration_rotationRateRoll = GY87_gyroscopeCalibrationValues.calibrationRateRoll;
        controlSystemActiveValues_TaskControlSystem->gyroCalibration_rotationRatePitch = GY87_gyroscopeCalibrationValues.calibrationRatePitch;
        controlSystemActiveValues_TaskControlSystem->gyroCalibration_rotationRateYaw = GY87_gyroscopeCalibrationValues.calibrationRateYaw;
        /* IMU Measurements (controlSystemActiveValues_TaskControlSystem->Gyroscope) */
        controlSystemActiveValues_TaskControlSystem->gyroMeasurement_rotationRateRoll = GY87_gyroscopeValues.rotationRateRoll;
        controlSystemActiveValues_TaskControlSystem->gyroMeasurement_rotationRatePitch = GY87_gyroscopeValues.rotationRatePitch;
        controlSystemActiveValues_TaskControlSystem->gyroMeasurement_rotationRateYaw = GY87_gyroscopeValues.rotationRateYaw;
        /* IMU Calibration (Accelerometer) */
        controlSystemActiveValues_TaskControlSystem->accCalibration_calibrationDone = GY87_accelerometerCalibrationValues.calibrationDone;
        controlSystemActiveValues_TaskControlSystem->accCalibration_fixedCalibration_en = GY87_accelerometerCalibrationValues.fixedCalibration_en;
        controlSystemActiveValues_TaskControlSystem->accCalibration_linearAccelerationX = GY87_accelerometerCalibrationValues.calibrationLinearAccelerationX;
        controlSystemActiveValues_TaskControlSystem->accCalibration_linearAccelerationY = GY87_accelerometerCalibrationValues.calibrationLinearAccelerationY;
        controlSystemActiveValues_TaskControlSystem->accCalibration_linearAccelerationZ = GY87_accelerometerCalibrationValues.calibrationLinearAccelerationZ;
        /* IMU Measurements (Accelerometer) */
        controlSystemActiveValues_TaskControlSystem->accMeasurement_linearAccelerationX = GY87_accelerometerValues.linearAccelerationX;
        controlSystemActiveValues_TaskControlSystem->accMeasurement_linearAccelerationY = GY87_accelerometerValues.linearAccelerationY;
        controlSystemActiveValues_TaskControlSystem->accMeasurement_linearAccelerationZ = GY87_accelerometerValues.linearAccelerationZ;
        controlSystemActiveValues_TaskControlSystem->accMeasurement_angleRoll = GY87_accelerometerValues.angleRoll;
        controlSystemActiveValues_TaskControlSystem->accMeasurement_anglePitch = GY87_accelerometerValues.anglePitch;
        /* IMU Measurements (Magnetometer) */
        controlSystemActiveValues_TaskControlSystem->magMeasurement_magneticFieldX = GY87_magnetometerValues.magneticFieldX;
        controlSystemActiveValues_TaskControlSystem->magMeasurement_magneticFieldY = GY87_magnetometerValues.magneticFieldY;
        controlSystemActiveValues_TaskControlSystem->magMeasurement_magneticFieldZ = GY87_magnetometerValues.magneticFieldZ;
        controlSystemActiveValues_TaskControlSystem->magMeasurement_magneticHeading = GY87_magnetometerHeadingValue;
        /* IMU Measurements (Temperature Sensor) */
        controlSystemActiveValues_TaskControlSystem->temperature = GY87_temperature;
        /* ESCs Values*/
        controlSystemActiveValues_TaskControlSystem->ESC1_speed = ESC_speeds[0];
        controlSystemActiveValues_TaskControlSystem->ESC2_speed = ESC_speeds[1];
        controlSystemActiveValues_TaskControlSystem->ESC3_speed = ESC_speeds[2];
        controlSystemActiveValues_TaskControlSystem->ESC4_speed = ESC_speeds[3];

        /* Try to swap buffers when ready to send */
        if (xSemaphoreTake(Semaphore_Handle_controlSystemValuesSwap, 0) == pdTRUE) {
            /* Swap buffers as Task_ControlSystem is not using 'controlSystemActiveValues_TaskControlSystem' */
            ControlSystemValues_t * tempValues = controlSystemActiveValues_TaskControlSystem;
            controlSystemActiveValues_TaskControlSystem = controlSystemActiveValues_TaskDebugging;
            controlSystemActiveValues_TaskDebugging = tempValues;

            /* Signal Task_USB_Communication */
            ControlSystemValues_t * logControlSystemValues = controlSystemActiveValues_TaskDebugging;
            xQueueSend(Queue_Handle_ControlSystemValues_Debug, &logControlSystemValues, 0);
        }

        /* Clear buffer */
        memset(FlightLightsActiveBuffer_TaskControlSystem, 0, sizeof(FlightLights_Buffer_A));

        FlightLightsActiveBuffer_TaskControlSystem[0] = FSA8S_channelValues[7];
        FlightLightsActiveBuffer_TaskControlSystem[1] = FSA8S_channelValues[8];
        FlightLightsActiveBuffer_TaskControlSystem[2] = FSA8S_channelValues[9];

        /* Flight Lights buffer swap implementation */
        if (xSemaphoreTake(Semaphore_Handle_flightLightsBufferSwap, 0) == pdTRUE) {
            /* Swap buffers for Flight Lights */
            uint16_t * tempFlightLightsBuffer = FlightLightsActiveBuffer_TaskControlSystem;
            FlightLightsActiveBuffer_TaskControlSystem = FlightLightsActiveBuffer_TaskFlightLights;
            FlightLightsActiveBuffer_TaskFlightLights = tempFlightLightsBuffer;

            /* Signal Task_FlightLights */
            xQueueSend(Queue_Handle_FlightLights_Commands, &FlightLightsActiveBuffer_TaskFlightLights, 0);
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

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(10);

    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {

        if (xQueueReceive(Queue_Handle_USB_Communication_Info, &logInformationString, 0) == pdPASS) {
            /* Validate pointer before using */
            if (logInformationString != NULL) {
                /* Single attempt without retries */
                LOG((uint8_t *)logInformationString, LOG_INFORMATION);
            }
        }

        if (xQueueReceive(Queue_Handle_USB_Communication_Debug, &logDebuggingString, 0) == pdPASS) {
            if (logDebuggingString != NULL) {
                /* Send debug message through USB */
                LOG((uint8_t *)logDebuggingString, LOG_DEBUGGING);

                /* Allow Task_Debugging to use this buffer again */
                xSemaphoreGive(Semaphore_Handle_debuggingBufferSwap);
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
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(10);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    ControlSystemValues_t * logControlSystemValues;
    uint32_t system_tick = 0;
    uint16_t written_chars = 0;

    float FlightController_batteryLevel;

    while (1) {

        if (!FlightController_isInitialized) {
            vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
            continue;
        }

        if (xQueueReceive(Queue_Handle_ControlSystemValues_Debug, &logControlSystemValues, 0) == pdPASS) {
            if (logControlSystemValues != NULL) {

                /* Log system time*/
                system_tick = xTaskGetTickCount();
                snprintf((char *)debuggingStr_SystemTime, 16 * sizeof(uint8_t), "T_%lu", (system_tick * 1000 / configTICK_RATE_HZ));

#if (MAIN_APP_DEBUGGING_FSA8S_MAIN == 1)
                /* Log channel values */
                snprintf((char *)debuggingStr_FSA8S_main, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%d/%d_%d/%d_%d", FSA8S_CHANNEL_VALUES_3, logControlSystemValues->radioController_channelValues[2], FSA8S_CHANNEL_VALUES_1,
                         logControlSystemValues->radioController_channelValues[0], FSA8S_CHANNEL_VALUES_2, logControlSystemValues->radioController_channelValues[1], FSA8S_CHANNEL_VALUES_4, logControlSystemValues->radioController_channelValues[3],
                         FSA8S_CHANNEL_VALUES_6, logControlSystemValues->radioController_channelValues[5]);
#endif

#if (MAIN_APP_DEBUGGING_FSA8S_AUX == 1)
                /* Log channel values */
                snprintf((char *)debuggingStr_FSA8S_aux, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%d/%d_%d/%d_%d", FSA8S_CHANNEL_VALUES_5, logControlSystemValues->radioController_channelValues[4], FSA8S_CHANNEL_VALUES_7,
                         logControlSystemValues->radioController_channelValues[6], FSA8S_CHANNEL_VALUES_8, logControlSystemValues->radioController_channelValues[7], FSA8S_CHANNEL_VALUES_9, logControlSystemValues->radioController_channelValues[8],
                         FSA8S_CHANNEL_VALUES_10, logControlSystemValues->radioController_channelValues[9]);
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_CALIBRATION_VALUES)
                /* Log GY87 gyroscope calibration values */
                snprintf((char *)debuggingStr_GY87_gyroscopeCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL, logControlSystemValues->gyroCalibration_rotationRateRoll,
                         GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH, logControlSystemValues->gyroCalibration_rotationRatePitch, GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW, logControlSystemValues->gyroCalibration_rotationRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_CALIBRATION_VALUES)
                /* Log GY87 accelerometer calibration values */
                snprintf((char *)debuggingStr_GY87_accelerometerCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_ACC_CALIBRATION_VALUES_LINEAR_X, logControlSystemValues->accCalibration_linearAccelerationX,
                         GY87_ACC_CALIBRATION_VALUES_LINEAR_Y, logControlSystemValues->accCalibration_linearAccelerationY, GY87_ACC_CALIBRATION_VALUES_LINEAR_Z, logControlSystemValues->accCalibration_linearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_VALUES == 1)
                /* Log GY87 gyroscope values */
                snprintf((char *)debuggingStr_GY87_gyroscopeValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", GY87_GYRO_VALUES_ROT_RATE_ROLL, logControlSystemValues->gyroMeasurement_rotationRateRoll,
                         GY87_GYRO_VALUES_ROT_RATE_PITCH, logControlSystemValues->gyroMeasurement_rotationRatePitch, GY87_GYRO_VALUES_ROT_RATE_YAW, logControlSystemValues->gyroMeasurement_rotationRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_VALUES == 1)
                /* Log GY87 accelerometer values */
                snprintf((char *)debuggingStr_GY87_accelerometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_ACC_VALUES_LINEAR_X, logControlSystemValues->accCalibration_linearAccelerationX, GY87_ACC_VALUES_LINEAR_Y,
                         logControlSystemValues->accCalibration_linearAccelerationY, GY87_ACC_VALUES_LINEAR_Z, logControlSystemValues->accCalibration_linearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_ANGLES == 1)
                /* Log GY87 accelerometer angles */
                snprintf((char *)debuggingStr_GY87_accelerometerAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f", GY87_ACC_VALUES_ANGLE_ROLL, logControlSystemValues->accMeasurement_angleRoll, GY87_ACC_VALUES_ANGLE_PITCH,
                         logControlSystemValues->accMeasurement_anglePitch);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES == 1)
                /* Log GY87 magnetometer values */
                snprintf((char *)debuggingStr_GY87_magnetometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f", GY87_MAG_VALUES_MAG_FIELD_X, logControlSystemValues->magMeasurement_magneticFieldX, GY87_MAG_VALUES_MAG_FIELD_Y,
                         logControlSystemValues->magMeasurement_magneticFieldY, GY87_MAG_VALUES_MAG_FIELD_Z, logControlSystemValues->magMeasurement_magneticFieldZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING == 1)
                /* Log GY87 magnetometer heading */
                snprintf((char *)debuggingStr_GY87_magnetometerHeadingValue, 16 * sizeof(uint8_t), (const char *)"/%d_%.2f", GY87_MAG_HEADING, logControlSystemValues->magMeasurement_magneticHeading);
#endif

#if (MAIN_APP_DEBUGGING_GY87_TEMPERATURE == 1)
                /* Log GY87 temperature value */
                snprintf((char *)debuggingStr_GY87_temperature, 16 * sizeof(uint8_t), (const char *)"/%d_%.1f", GY87_TEMPERATURE, logControlSystemValues->temperature);
#endif

#if (MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL == 1)
                /* Read flight controller battery level */
                xQueuePeek(Queue_Handle_BatteryLevel, &FlightController_batteryLevel, 0);
                /* Log flight controller battery level */
                snprintf((char *)debuggingStr_BatteryLevel, 10 * sizeof(uint8_t), (const char *)"/%d_%.2f", FLIGHT_CONTROLLER_BATTERY_LEVEL, (double)FlightController_batteryLevel);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES == 1)
                /* Log control system reference values */
                snprintf((char *)debuggingStr_ControlSystem_ReferenceAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_REFERENCE_THROTTLE, reference_throttle, CONTROLSYSTEM_REFERENCE_ROLL_ANGLE,
                         reference_rollAngle, CONTROLSYSTEM_REFERENCE_PITCH_ANGLE, reference_pitchAngle);
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
                snprintf((char *)debuggingStr_ControlSystem_ReferenceRates, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f", CONTROLSYSTEM_REFERENCE_ROLL_RATE, desiredValue_rollRate, CONTROLSYSTEM_REFERENCE_PITCH_RATE,
                         desiredValue_pitchRate, CONTROLSYSTEM_REFERENCE_YAW_RATE, desiredValue_yawRate);
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

#if (MAIN_APP_DEBUGGING_ESCS == 1)
                /* Log ESC values*/
                snprintf((char *)debuggingStr_ESCs, 40 * sizeof(uint8_t), (const char *)"/%d_%d/%d_%d/%d_%d/%d_%d", ESC_1, logControlSystemValues->ESC1_speed, ESC_2, logControlSystemValues->ESC2_speed, ESC_3, logControlSystemValues->ESC3_speed,
                         ESC_4, logControlSystemValues->ESC4_speed);
#endif

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK)
                /* Log tasks stack high watermark */
                snprintf((char *)debuggingStr_TasksStackHighWatermark, 80 * sizeof(uint8_t), (const char *)"/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld", TASK_STACK_WATERMARK_ONOFFBUTTON, Task_StackHighWatermark_OnOffButton,
                         TASK_STACK_WATERMARK_CONTROLSYSTEM, Task_StackHighWatermark_ControlSystem, TASK_STACK_WATERMARK_USBCOMMUNICATION, Task_StackHighWatermark_USB_Communication, TASK_STACK_WATERMARK_DEBUGGING, Task_StackHighWatermark_Debugging,
                         TASK_STACK_WATERMARK_BATTERYLEVEL, Task_StackHighWatermark_BatteryLevel, TASK_STACK_WATERMARK_BATTERYALARM, Task_StackHighWatermark_BatteryAlarm, TASK_STACK_WATERMARK_HEARTBEATLIGHT, Task_StackHighWatermark_HeartbeatLight,
                         TASK_STACK_WATERMARK_FLIGHTLIGHTS, Task_StackHighWatermark_FlightLights);
#endif
            }

            /* Allow Task_ControlSystem to use this buffer again */
            xSemaphoreGive(Semaphore_Handle_controlSystemValuesSwap);
        }

        /* Concatenate all debugging strings */
        memset(debuggingActiveBuffer_TaskDebugging, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE);
        written_chars = 0;
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_SystemTime);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_FSA8S_main);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_FSA8S_aux);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_gyroscopeCalibrationValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_gyroscopeValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerCalibrationValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_accelerometerAngles);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_magnetometerValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_magnetometerHeadingValue);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_GY87_temperature);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ESCs);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_BatteryLevel);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ReferenceAngles);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_KalmanAngles);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_AnglesErrors);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_AnglesPID);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ReferenceRates);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_RatesErrors);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_RatesPID);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_TasksStackHighWatermark);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "\n");

        /* Try to swap buffers when ready to send */
        if (xSemaphoreTake(Semaphore_Handle_debuggingBufferSwap, 0) == pdTRUE) {
            /* Swap buffers as Task_USB_Communication is not using 'debuggingActiveBuffer_TaskUSBCommunication' */
            uint8_t * tempBuffer = debuggingActiveBuffer_TaskUSBCommunication;
            debuggingActiveBuffer_TaskUSBCommunication = debuggingActiveBuffer_TaskDebugging;
            debuggingActiveBuffer_TaskDebugging = tempBuffer;

            /* Signal Task_USB_Communication */
            uint8_t * logDebuggingString = debuggingActiveBuffer_TaskUSBCommunication;
            xQueueSend(Queue_Handle_USB_Communication_Debug, &logDebuggingString, 0);
        }

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
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

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
        /* Get stack watermark */
        Task_StackHighWatermark_OnOffButton = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_BatteryLevel(void * ptr) {
    (void)ptr;

    uint16_t adcValue;
    float FlightController_batteryLevel;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(1000);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

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
        FlightController_batteryLevel = FlightController_batteryLevel * 3.363636 + BATTERY_LEVEL_CALIBRATION_OFFSET;

        /* Send to queue, overwriting old value */
        xQueueOverwrite(Queue_Handle_BatteryLevel, &FlightController_batteryLevel);

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_BatteryLevel = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_BatteryAlarm(void * ptr) {
    (void)ptr;

    uint8_t alarmSequence[] = {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t alarmSequenceSize = sizeof(alarmSequence);
    uint8_t alarmSequenceCursor = 0;

    const uint16_t ALARM_SEQUENCE_DELAY = 200;

    /* For main task timing */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* For alarm sequence timing */
    TickType_t xAlarmLastWakeTime = xLastWakeTime;
    TickType_t xAlarmPeriod = pdMS_TO_TICKS(ALARM_SEQUENCE_DELAY);

    float FlightController_batteryLevel;

    while (1) {
        if (xQueuePeek(Queue_Handle_BatteryLevel, &FlightController_batteryLevel, 0) == pdTRUE) {
            if (FlightController_batteryLevel < BATTERY_ALARM_THRESHOLD) {
                /* Check if it's time to update the alarm sequence */
                if ((xTaskGetTickCount() - xAlarmLastWakeTime) >= xAlarmPeriod) {
                    /* Parse alarm sequence */
                    alarmSequenceCursor++;
                    if (alarmSequenceSize <= alarmSequenceCursor) {
                        alarmSequenceCursor = 0;
                    }

                    /* Write to buzzer */
                    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, alarmSequence[alarmSequenceCursor]);

                    /* Update alarm timing reference point */
                    xAlarmLastWakeTime = xTaskGetTickCount();
                }
            } else {
                /* Turn buzzer off when battery level is OK */
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);

                /* Reset sequence cursor when alarm is off */
                alarmSequenceCursor = 0;
                /* Reset alarm timing reference point */
                xAlarmLastWakeTime = xTaskGetTickCount();
            }
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_BatteryAlarm = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_HeartbeatLight(void * ptr) {
    (void)ptr;

    uint8_t ledState = GPIO_PIN_RESET;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(HEARTBEAT_PERIOD / 2);
    /* Get initial tick count */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {

        if (FlightController_isInitialized) {
            /* Toggle LED and update state */
            ledState = (ledState == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
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

    uint16_t * flightLightsCommands = NULL;
    uint16_t flightLightCommand_sequenceSpeed = 0;
    uint16_t flightLightCommand_sequenceSelect = 0;
    uint16_t flightLightCommand_sequenceEnable = 0;

    const uint16_t CHANNEL_THRESHOLD_LOW = 250;
    const uint16_t CHANNEL_THRESHOLD_MID = 750;
    const uint16_t LIGHTS_ENABLE_THRESHOLD = 500;
    const uint16_t BASE_SEQUENCE_DELAY = 200;

    uint8_t activeSequence = 0;
    uint8_t sequenceCursor = 0;
    uint16_t currentSequenceDelay = BASE_SEQUENCE_DELAY;

    /* For main task timing */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* For LED sequence timing */
    TickType_t xSequenceLastWakeTime = xLastWakeTime;
    TickType_t xSequencePeriod = pdMS_TO_TICKS(currentSequenceDelay);

    while (1) {
        /* Read the flight lights commands */
        if (xQueueReceive(Queue_Handle_FlightLights_Commands, &flightLightsCommands, 0) == pdPASS) {
            flightLightCommand_sequenceSpeed = flightLightsCommands[0];
            flightLightCommand_sequenceSelect = flightLightsCommands[1];
            flightLightCommand_sequenceEnable = flightLightsCommands[2];

            /* Allow Task_ControlSystem to use this buffer again */
            xSemaphoreGive(Semaphore_Handle_flightLightsBufferSwap);
        }

        /* Flight lights enabled check */
        if (flightLightCommand_sequenceEnable >= LIGHTS_ENABLE_THRESHOLD) {
            /* Select sequence based on channel 9 value */
            if (flightLightCommand_sequenceSelect <= CHANNEL_THRESHOLD_LOW) {
                activeSequence = 0;
            } else if (flightLightCommand_sequenceSelect <= CHANNEL_THRESHOLD_MID) {
                activeSequence = 1;
            } else {
                activeSequence = 2;
            }

            /* Adjust sequence speed based on potentiometer */
            currentSequenceDelay = BASE_SEQUENCE_DELAY + flightLightCommand_sequenceSpeed / 5;
            xSequencePeriod = pdMS_TO_TICKS(currentSequenceDelay);

            /* Check if it's time to update the LED sequence */
            if ((xTaskGetTickCount() - xSequenceLastWakeTime) >= xSequencePeriod) {
                /* Advance sequence with bounds check */
                sequenceCursor = (sequenceCursor + 1) % sequences[activeSequence].size;

                /* Update all LEDs */
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, sequences[activeSequence].led1[sequenceCursor]);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, sequences[activeSequence].led2[sequenceCursor]);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, sequences[activeSequence].led3[sequenceCursor]);
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, sequences[activeSequence].led4[sequenceCursor]);

                /* Update sequence timing reference point */
                xSequenceLastWakeTime = xTaskGetTickCount();
            }
        } else {
            /* Turn off all lights when disabled */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

            /* Reset sequence cursor when lights are disabled */
            sequenceCursor = 0;
            /* Reset sequence timing reference point */
            xSequenceLastWakeTime = xTaskGetTickCount();
        }

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_FlightLights = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
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

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    // /* Task: FlightController_OnOffButton */
    // BaseType_t ret = xTaskCreate(FlightController_OnOffButton, "FlightController_OnOffButton", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTCONTROLLER_ONOFFBUTTON_PRIORITY), &Task_Handle_OnOffButton);
    // /* Check the task was created successfully. */
    // configASSERT(ret == pdPASS);

    /* Timer1: OnOffButton */
    // Timer_Handle_OnOffButton = xTimerCreate("OnOffButton", pdMS_TO_TICKS(100), pdTRUE, (void *)0, Timer_Callback_OnOffButton);

    /* Turn on-board LED off */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

    /* Check if flight controller is already running */
    FlightController_isRunning = 1; // DELETE THIS LINE
    if (FlightController_isRunning) {

        /* Initialize drivers */
        rc_controller = FSA8S_Init(&huart2);
        if (NULL == rc_controller) {
            /* Error */
            ErrorLED_Start(FSA8S_InitializationError);
        }
        hgy87 = GY87_Init(&hi2c1);
        if (NULL == hgy87) {
            /* Error */
            ErrorLED_Start(GY87_InitializationError);
        }
        hesc = ESC_Init(&htim3);
        if (NULL == hesc) {
            /* Error */
            ErrorLED_Start(ESC_InitializationError);
        }

        /* Create system timers, queues, semaphores and tasks */
        if (FreeRTOS_CreateTimers() == false) {
            /* Error */
            ErrorLED_Start(FreeRTOS_TimersCreationError);
        }
        if (FreeRTOS_CreateQueues() == false) {
            /* Error */
            ErrorLED_Start(FreeRTOS_QueuesCreationError);
        }
        if (FreeRTOS_CreateSemaphores() == false) {
            /* Error */
            ErrorLED_Start(FreeRTOS_SemaphoresCreationError);
        }
        if (FreeRTOS_CreateTasks() == false) {
            /* Error */
            ErrorLED_Start(FreeRTOS_TasksCreationError);
        }
    }

    /* Initialize buffer structures */
    memset(&controlSystemValues_A, 0, sizeof(ControlSystemValues_t));
    memset(&controlSystemValues_B, 0, sizeof(ControlSystemValues_t));
    memset(debuggingStr_A, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE);
    memset(debuggingStr_B, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE);
}

/* --- End of file ----------------------------------------------------------------------------- */
