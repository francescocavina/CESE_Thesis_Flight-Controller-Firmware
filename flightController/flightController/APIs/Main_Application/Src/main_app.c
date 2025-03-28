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
#include "control_system_settings.h"
#include "control_system_support.h"
#include "debug_signals_ids.h"
#include "error_led.h"
#include "system_failure_test.h"
#include "user_settings.h"

#include "ESC_UAI.h"
#include "FSA8S_driver_UAI.h"
#include "LoggingSystem_UAI.h"
#include "MPU6050_driver_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
/* FreeRTOS General Settings */
#define USE_FREERTOS                  // Remove comment when using FreeRTOS
#define DEFAULT_TASK_DELAY            (100)
#define TEST_SYSTEM_FAILURE_PROCEDURE (1)
/* FreeRTOS Tasks Priorities */
#define TASK_ONOFFBUTTON_PRIORITY      (3)
#define TASK_STARTUP_PRIORITY          (3)
#define TASK_CONTROLSYSTEM_PRIORITY    (6)
#define TASK_USBCOMMUNICATION_PRIORITY (5)
#define TASK_DEBUGGING_PRIORITY        (4)
#define TASK_BATTERYLEVEL_PRIORITY     (2)
#define TASK_BATTERYALARM_PRIORITY     (2)
#define TASK_HEARTBEATLIGHT_PRIORITY   (2)
#define TASK_FLIGHTLIGHTS_PRIORITY     (2)
/* FreeRTOS Queues Sizes */
#define USB_COMMUNICATION_INFO_QUEUE_SIZE  (32)
#define USB_COMMUNICATION_DEBUG_QUEUE_SIZE (1)
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
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES              (1) // Not necessary for control system
#define MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING             (1) // Not necessary for control system
#define MAIN_APP_DEBUGGING_GY87_TEMPERATURE                      (1) // Not necessary for control system
#define MAIN_APP_DEBUGGING_ESCS                                  (1)
#define MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL       (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_VALUES        (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES        (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_KALMAN_ANGLES           (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_ERRORS           (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_PID_OUTPUT       (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_RATE          (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_ERRORS            (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_PID_OUTPUT        (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_MOTORS_SPEEDS           (1)
#define MAIN_APP_DEBUGGING_CONTROLSYSTEM_AUXILIAR                (1)
#define MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK             (1)
/* Battery Level Settings */
#define BATTERY_LEVEL_CALIBRATION_OFFSET (0.76)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Flight Controller State */
static bool_t FlightController_isRunning                          = false;
static bool_t FlightController_isInitialized                      = false;

/* Drivers Handles */
static IBUS_HandleTypeDef_t *rc_controller                        = NULL;
static GY87_HandleTypeDef_t *hgy87                                = NULL;
static ESC_HandleTypeDef_t  *hesc                                 = NULL;

/* Timers Handles */
static TimerHandle_t Timer_Handle_OnOffButton                     = NULL;
/* Queues Handles */
static QueueHandle_t Queue_Handle_ControlSystemValues_Debug       = NULL; // Queue to send pointers to Task_Debugging with Task_ControlSystem variables
static QueueHandle_t Queue_Handle_USB_Communication_Info          = NULL; // Queue to send pointers to Task_USB_Communication with information strings
static QueueHandle_t Queue_Handle_USB_Communication_Debug         = NULL; // Queue to send pointers to Task_USB_Communication with debugging strings
static QueueHandle_t Queue_Handle_FlightLights_Commands           = NULL; // Queue to send pointers to Task_FlightLights with flight lights radio controller commands
static QueueHandle_t Queue_Handle_BatteryLevel                    = NULL; // Queue to send battery level from Task_BatteryLevel to Task_BatteryAlarm
/* Semaphores Handles */
static SemaphoreHandle_t Semaphore_Handle_controlSystemValuesSwap = NULL;
static SemaphoreHandle_t Semaphore_Handle_debuggingBufferSwap     = NULL;
static SemaphoreHandle_t Semaphore_Handle_flightLightsBufferSwap  = NULL;
/* Tasks Handles */
static TaskHandle_t Task_Handle_OnOffButton                       = NULL;
static TaskHandle_t Task_Handle_StartUp                           = NULL;
static TaskHandle_t Task_Handle_ControlSystem                     = NULL;
static TaskHandle_t Task_Handle_USB_Communication                 = NULL;
static TaskHandle_t Task_Handle_Debugging                         = NULL;
static TaskHandle_t Task_Handle_BatteryLevel                      = NULL;
static TaskHandle_t Task_Handle_BatteryAlarm                      = NULL;
static TaskHandle_t Task_Handle_HeartbeatLight                    = NULL;
static TaskHandle_t Task_Handle_FlightLights                      = NULL;
/* Timers Variables */
static bool_t   Timer_Flag_OnOffButton                            = false;
static uint16_t Timer_AutoReloadTime_OnOffButton                  = PW_ON_OFF_DRIVER_TIME;

/* Control System Variables Debugging */
#if (MAIN_APP_LOGGING_DEBUGGING == 1)
static ControlSystemValues_t  controlSystemValues_A;
static ControlSystemValues_t  controlSystemValues_B;
static ControlSystemValues_t *controlSystemActiveValues_TaskControlSystem = &controlSystemValues_A; // Task_ControlSystem writes here
static ControlSystemValues_t *controlSystemActiveValues_TaskDebugging     = &controlSystemValues_B; // Task_Debugging reads from here
#endif

/* Flight Lights Commands */
static uint16_t  FlightLights_Buffer_A[3]                   = {0};
static uint16_t  FlightLights_Buffer_B[3]                   = {0};
static uint16_t *FlightLightsActiveBuffer_TaskControlSystem = FlightLights_Buffer_A; // Task_ControlSystem writes here
static uint16_t *FlightLightsActiveBuffer_TaskFlightLights  = FlightLights_Buffer_B; // Task_FlightLights reads from here here

/* Stack High Watermarks Variables */
#if (MAIN_APP_LOGGING_DEBUGGING == 1)
static UBaseType_t Task_StackHighWatermark_OnOffButton       = 0;
static UBaseType_t Task_StackHighWatermark_ControlSystem     = 0;
static UBaseType_t Task_StackHighWatermark_USB_Communication = 0;
static UBaseType_t Task_StackHighWatermark_Debugging         = 0;
static UBaseType_t Task_StackHighWatermark_BatteryLevel      = 0;
static UBaseType_t Task_StackHighWatermark_BatteryAlarm      = 0;
static UBaseType_t Task_StackHighWatermark_HeartbeatLight    = 0;
static UBaseType_t Task_StackHighWatermark_FlightLights      = 0;
#endif

/* Debugging Variables */
#if (MAIN_APP_LOGGING_DEBUGGING == 1)
static uint8_t  debuggingStr_A[MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE];
static uint8_t  debuggingStr_B[MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE];
static uint8_t *debuggingActiveBuffer_TaskDebugging                  = debuggingStr_A; // Task_Debugging writes here
static uint8_t *debuggingActiveBuffer_TaskUSBCommunication           = debuggingStr_B; // Task_USB_Communication reads from here
static uint8_t  debuggingStr_SystemTime[16]                          = {0};            // Size checked
static uint8_t  debuggingStr_FSA8S_main[50]                          = {0};            // Size checked
static uint8_t  debuggingStr_FSA8S_aux[50]                           = {0};            // Size checked
static uint8_t  debuggingStr_GY87_gyroscopeCalibrationValues[40]     = {0};            // Size checked
static uint8_t  debuggingStr_GY87_gyroscopeValues[40]                = {0};            // Size checked
static uint8_t  debuggingStr_GY87_accelerometerCalibrationValues[40] = {0};            // Size checked
static uint8_t  debuggingStr_GY87_accelerometerValues[40]            = {0};            // Size checked
static uint8_t  debuggingStr_GY87_accelerometerAngles[40]            = {0};            // Size checked
static uint8_t  debuggingStr_GY87_magnetometerValues[40]             = {0};            // Size checked
static uint8_t  debuggingStr_GY87_magnetometerHeadingValue[16]       = {0};            // Size checked
static uint8_t  debuggingStr_GY87_temperature[16]                    = {0};            // Size checked
static uint8_t  debuggingStr_BatteryLevel[10]                        = {0};            // Size checked
static uint8_t  debuggingStr_ControlSystem_referenceValues[40]       = {0};
static uint8_t  debuggingStr_ControlSystem_referenceAngles[40]       = {0};
static uint8_t  debuggingStr_ControlSystem_KalmanAngles[30]          = {0};            // Size checked
static uint8_t  debuggingStr_ControlSystem_anglesErrors[40]          = {0};
static uint8_t  debuggingStr_ControlSystem_anglesPID[40]             = {0};
static uint8_t  debuggingStr_ControlSystem_referenceRates[40]        = {0};
static uint8_t  debuggingStr_ControlSystem_ratesErrors[40]           = {0};
static uint8_t  debuggingStr_ControlSystem_ratesPID[40]              = {0};
static uint8_t  debuggingStr_ControlSystem_motorsSpeeds[40]          = {0};
static uint8_t  debuggingStr_ControlSystem_Auxiliar[40]              = {0};
static uint8_t  debuggingStr_ESCs[40]                                = {0};
static uint8_t  debuggingStr_TasksStackHighWatermark[80]             = {0}; // Size checked
#endif

/* Control System */
static ControlSystemValues_t controlSystemValues;
static FSA8S_CHANNEL_t       channels[FSA8S_CHANNELS] = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10};

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Initializes the system variables.
 * @param  None
 * @retval None
 */
void Initialize_SystemVariables(void);

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
void Task_ControlSystem(void *ptr);

/*
 * @brief  Task: Sends data through USB port.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_USB_Communication(void *ptr);

/*
 * @brief  Task: Logs flight controller data.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_Debugging(void *ptr);

/*
 * @brief  Task: Reads the on-board on/off button and turns on/off the flight controller.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_OnOffButton(void *ptr);

/*
 * @brief  Task: Initializes and configures the flight controller system.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_Init(void *ptr);

/*
 * @brief  Task: Reads the flight controller battery level and gives a signal whenever the level
 *               is below a threshold set by the user.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_BatteryLevel(void *ptr);

/*
 * @brief  Task: Activates an alarm whenever the battery level is below an user-defined threshold.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_BatteryAlarm(void *ptr);

/*
 * @brief  Task: Blinks an on-board-LED.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_HeartbeatLight(void *ptr);

/*
 * @brief  Task: Produces blinking sequences with the 4 flight lights.
 * @param  Task pointer: not used.
 * @retval None
 */
void Task_FlightLights(void *ptr);

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
 * @brief  WWDG Early Wakeup Callback: Handles actions when window watchdog early warning occurs
 * @param  WWDG_HandleTypeDef pointer to the watchdog handle structure
 * @retval None
 */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);

/* --- Public variable definitions ------------------------------------------------------------- */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern WWDG_HandleTypeDef hwwdg;

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
void Initialize_SystemVariables(void) {
    /* Control System Variables */
    controlSystemValues.throttleStick_startedDown                     = false;

    controlSystemValues.gyroCalibration.calibrationDone               = false;
    controlSystemValues.gyroCalibration.fixedCalibration_en           = true;
    controlSystemValues.gyroCalibration.calibrationRateRoll           = 0.0;
    controlSystemValues.gyroCalibration.calibrationRatePitch          = 0.0;
    controlSystemValues.gyroCalibration.calibrationRateYaw            = 0.0;

    controlSystemValues.accCalibration.calibrationDone                = false;
    controlSystemValues.accCalibration.fixedCalibration_en            = true;
    controlSystemValues.accCalibration.calibrationLinearAccelerationX = 0.0;
    controlSystemValues.accCalibration.calibrationLinearAccelerationY = 0.0;
    controlSystemValues.accCalibration.calibrationLinearAccelerationZ = 0.0;

    controlSystemValues.KalmanPrediction_rollAngle                    = 0;
    controlSystemValues.KalmanPrediction_pitchAngle                   = 0;
    controlSystemValues.KalmanUncertainty_rollAngle                   = 2 * 2;
    controlSystemValues.KalmanUncertainty_pitchAngle                  = 2 * 2;
}

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

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
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
#endif

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

    /* Task 3: ControlSystem */
    xTaskCreate(Task_ControlSystem, "Task_ControlSystem", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_CONTROLSYSTEM_PRIORITY), &Task_Handle_ControlSystem);
    if (Task_Handle_ControlSystem == NULL) {
        return false;
    }

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
    /* Task 4: USB_Communication */
    xTaskCreate(Task_USB_Communication, "Task_USB_Communication", (4 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_USBCOMMUNICATION_PRIORITY), &Task_Handle_USB_Communication);
    if (Task_Handle_USB_Communication == NULL) {
        return false;
    }

    /* Task 5: Debugging */
    xTaskCreate(Task_Debugging, "Task_Debugging", (3 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_DEBUGGING_PRIORITY), &Task_Handle_Debugging);
    if (Task_Handle_Debugging == NULL) {
        return false;
    }
#endif

    /* Task 6: BatteryLevel */
    xTaskCreate(Task_BatteryLevel, "Task_BatteryLevel", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYLEVEL_PRIORITY), &Task_Handle_BatteryLevel);
    if (Task_Handle_BatteryLevel == NULL) {
        return false;
    }

    /* Task 7: BatteryAlarm */
    xTaskCreate(Task_BatteryAlarm, "Task_BatteryAlarm", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_BATTERYALARM_PRIORITY), &Task_Handle_BatteryAlarm);
    if (Task_Handle_BatteryAlarm == NULL) {
        return false;
    }

    /* Task 8: HeartbeatLight */
    xTaskCreate(Task_HeartbeatLight, "Task_HeartbeatLight", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_HEARTBEATLIGHT_PRIORITY), &Task_Handle_HeartbeatLight);
    if (Task_Handle_HeartbeatLight == NULL) {
        return false;
    }

    /* Task 9: FlightLights */
    xTaskCreate(Task_FlightLights, "Task_FlightLights", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_FLIGHTLIGHTS_PRIORITY), &Task_Handle_FlightLights);
    if (Task_Handle_FlightLights == NULL) {
        return false;
    }

    /* Successfully created all tasks */
    return true;
}

void Task_ControlSystem(void *ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod     = pdMS_TO_TICKS(4);
    /* Get initial tick count */
    TickType_t xLastWakeTime         = xTaskGetTickCount();
    /* Variables to measure control loop period */
    TickType_t xPreviousLastWakeTime = 0;
    TickType_t xActualWakeTime       = 0;
    TickType_t xControlLoopPeriod    = 0;
    /* Variables to measure the execution time of the task */
    TickType_t xTaskStartTime;

    while (1) {
        /* Record task start time for execution measurement */
        xTaskStartTime        = xTaskGetTickCount();

        /* Measure actual wake-up time */
        xActualWakeTime       = xTaskGetTickCount();
        /* Calculate period between ACTUAL wake times */
        xControlLoopPeriod    = xActualWakeTime - xPreviousLastWakeTime;
        /* Store current actual time for next iteration */
        xPreviousLastWakeTime = xActualWakeTime;

        /* Calibrate GY-87 gyroscope sensor */
        if (false == controlSystemValues.gyroCalibration.calibrationDone) {
            vTaskDelay(pdMS_TO_TICKS(200));
            GY87_CalibrateGyroscope(hgy87, &controlSystemValues.gyroCalibration, !((bool_t)GY87_CALIBRATION_EN));
        }
        /* Calibrate GY-87 accelerometer sensor */
        if (false == controlSystemValues.accCalibration.calibrationDone) {
            vTaskDelay(pdMS_TO_TICKS(200));
            GY87_CalibrateAccelerometer(hgy87, &controlSystemValues.accCalibration, !((bool_t)GY87_CALIBRATION_EN));
        }
        /* Check that both sensors are calibrated */
        if (controlSystemValues.gyroCalibration.calibrationDone && controlSystemValues.accCalibration.calibrationDone && false == FlightController_isInitialized) {
            /* Beep to indicate that the system is ready */
#if (INITIALIZATION_READY_BEEPING == 1)
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#endif
            /* Set global flags */
            FlightController_isInitialized = true;
        }

        /* Refresh the WWDG */
        HAL_WWDG_Refresh(&hwwdg);

        /* System Failure Simulation */
        // #if (TEST_SYSTEM_FAILURE_PROCEDURE == 1)
        //         controlSystemValues.radioController_channelValues[4] = FSA8S_ReadChannel(rc_controller, CHANNEL_5);
        //         Test_SystemFailureProcedure(controlSystemValues.radioController_channelValues[4]);
        // #endif

        /* Control system processing */
        if (FlightController_isInitialized && 0 == CONTROLSYSTEM_MODE) {

            /* Read FS-A8S channels */
            for (uint8_t i = 0; i < FSA8S_CHANNELS; i++) {
                controlSystemValues.radioController_channelValues[i] = FSA8S_ReadChannel(rc_controller, channels[i]);
            }

            /* Read GY-87 gyroscope sensor */
            GY87_ReadGyroscope(hgy87, &controlSystemValues.gyroMeasurement, &controlSystemValues.gyroCalibration);

            /* Read GY-87 accelerometer sensor */
            GY87_ReadAccelerometer(hgy87, &controlSystemValues.accMeasurement, &controlSystemValues.accCalibration);

/* Read GY-87 magnetometer values */
#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES == 1)
            GY87_ReadMagnetometer(hgy87, &controlSystemValues.magMeasurement);
#endif

/* Read GY-87 magnetometer heading */
#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING == 1)
            controlSystemValues.magMeasurement_magneticHeading = GY87_ReadMagnetometerHeading(hgy87);
#endif

/* Read GY-87 temperature sensor */
#if (MAIN_APP_DEBUGGING_GY87_TEMPERATURE == 1)
            controlSystemValues.temperature = GY87_ReadTemperatureSensor(hgy87);
#endif

            /* Calculate Kalman angles */
            Kalman_CalculateAngle(&controlSystemValues.KalmanPrediction_rollAngle, &controlSystemValues.KalmanUncertainty_rollAngle, controlSystemValues.gyroMeasurement.rotationRateRoll, controlSystemValues.accMeasurement.angleRoll);
            Kalman_CalculateAngle(&controlSystemValues.KalmanPrediction_pitchAngle, &controlSystemValues.KalmanUncertainty_pitchAngle, controlSystemValues.gyroMeasurement.rotationRatePitch, controlSystemValues.accMeasurement.anglePitch);

        } else if (FlightController_isInitialized && 1 == CONTROLSYSTEM_MODE) {

            /* Read FS-A8S channels  for flight lights */
            controlSystemValues.radioController_channelValues[7] = FSA8S_ReadChannel(rc_controller, CHANNEL_8);
            controlSystemValues.radioController_channelValues[8] = FSA8S_ReadChannel(rc_controller, CHANNEL_9);
            controlSystemValues.radioController_channelValues[9] = FSA8S_ReadChannel(rc_controller, CHANNEL_10);

            /* Avoid uncontrolled motor start */

            if (false == controlSystemValues.throttleStick_startedDown) {
                /* Read throttle input from radio controller */
                controlSystemValues.radioController_channelValues[3] = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
                controlSystemValues.reference_throttle               = controlSystemValues.radioController_channelValues[3];

                if (15 > controlSystemValues.reference_throttle) {
                    controlSystemValues.throttleStick_startedDown = true;
                } else {
                    controlSystemValues.throttleStick_startedDown = false;
                }

            } else {
                /* Check if ESCs are enabled (Switch B on radio controller) */
                controlSystemValues.radioController_channelValues[5] = FSA8S_ReadChannel(rc_controller, CHANNEL_6);
                if (500 <= controlSystemValues.radioController_channelValues[5]) {
                    controlSystemValues.ESC_isEnabled = true;
                } else {
                    controlSystemValues.ESC_isEnabled = false;
                }

                /* Turn off motors in case ESCs are disabled */
                if (false == controlSystemValues.ESC_isEnabled) {

                    /* Save motors speed */
                    controlSystemValues.ESC1_speed = 0;
                    controlSystemValues.ESC2_speed = 0;
                    controlSystemValues.ESC3_speed = 0;
                    controlSystemValues.ESC4_speed = 0;

                    /* Turn off motors */
                    // ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues.ESC4_speed);
                    // ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues.ESC2_speed);
                    // ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues.ESC3_speed);
                    // ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues.ESC1_speed);

                    /* Reset PID variables */
                    CSM_ResetPID();

                } else {

                    /* Read input throttle from radio controller */
                    controlSystemValues.reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

                    /* Check if throttle stick is low */
                    if (CONTROLSYSTEM_MINIMUM_INPUT_THROTTLE > controlSystemValues.reference_throttle) {

                        /* Save motors speed */
                        controlSystemValues.ESC1_speed = 0;
                        controlSystemValues.ESC2_speed = 0;
                        controlSystemValues.ESC3_speed = 0;
                        controlSystemValues.ESC4_speed = 0;

                        /* Turn off motors */
                        // ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues.ESC4_speed);
                        // ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues.ESC2_speed);
                        // ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues.ESC3_speed);
                        // ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues.ESC1_speed);

                        /* Reset PID variables */
                        CSM_ResetPID();

                    } else {

                        /* Read GY-87 gyroscope sensor */
                        GY87_ReadGyroscope(hgy87, &controlSystemValues.gyroMeasurement, &controlSystemValues.gyroCalibration);
                        /* Read GY-87 accelerometer sensor */
                        GY87_ReadAccelerometer(hgy87, &controlSystemValues.accMeasurement, &controlSystemValues.accCalibration);

                        /* Calculate Kalman angles */
                        Kalman_CalculateAngle(&controlSystemValues.KalmanPrediction_rollAngle, &controlSystemValues.KalmanUncertainty_rollAngle, controlSystemValues.gyroMeasurement.rotationRateRoll, controlSystemValues.accMeasurement.angleRoll);
                        Kalman_CalculateAngle(&controlSystemValues.KalmanPrediction_pitchAngle, &controlSystemValues.KalmanUncertainty_pitchAngle, controlSystemValues.gyroMeasurement.rotationRatePitch, controlSystemValues.accMeasurement.anglePitch);

                        /* Read inputs from radio controller */
                        controlSystemValues.radioController_channelValues[0] = FSA8S_ReadChannel(rc_controller, CHANNEL_1);
                        controlSystemValues.radioController_channelValues[1] = FSA8S_ReadChannel(rc_controller, CHANNEL_2);
                        controlSystemValues.radioController_channelValues[2] = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
                        controlSystemValues.radioController_channelValues[3] = FSA8S_ReadChannel(rc_controller, CHANNEL_4);
                        controlSystemValues.reference_throttle               = (float)controlSystemValues.radioController_channelValues[2];
                        controlSystemValues.reference_rollValue              = (float)controlSystemValues.radioController_channelValues[0];
                        controlSystemValues.reference_pitchValue             = (float)controlSystemValues.radioController_channelValues[1];
                        controlSystemValues.reference_yawValue               = (float)controlSystemValues.radioController_channelValues[3];

                        /* Adjust and limit throttle input */
                        if (CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE < controlSystemValues.reference_throttle) {
                            controlSystemValues.reference_throttle = CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE;
                        }

                        /* Calculate desired angles by mapping radio controller values to angles */
                        controlSystemValues.reference_rollAngle  = 0.03 * (controlSystemValues.reference_rollValue - 500);
                        controlSystemValues.reference_pitchAngle = 0.03 * (controlSystemValues.reference_pitchValue - 500);

                        /* Calculate angles errors */
                        controlSystemValues.error_rollAngle      = controlSystemValues.reference_rollAngle - controlSystemValues.KalmanPrediction_rollAngle;
                        controlSystemValues.error_pitchAngle     = controlSystemValues.reference_pitchAngle - controlSystemValues.KalmanPrediction_pitchAngle;

                        /* Calculate PID for roll angle */
                        CSM_CalculatePID(&controlSystemValues.PID_Output_rollAngle, &controlSystemValues.PID_previousIterm_rollAngle, &controlSystemValues.PID_previousError_rollAngle, controlSystemValues.error_rollAngle, CONTROLSYSTEM_KP_ROLL_ANGLE, CONTROLSYSTEM_KI_ROLL_ANGLE, CONTROLSYSTEM_KD_ROLL_ANGLE);
                        /* Calculate PID for pitch angle */
                        CSM_CalculatePID(&controlSystemValues.PID_Output_pitchAngle, &controlSystemValues.PID_previousIterm_pitchAngle, &controlSystemValues.PID_previousError_pitchAngle, controlSystemValues.error_pitchAngle, CONTROLSYSTEM_KP_PITCH_ANGLE, CONTROLSYSTEM_KI_PITCH_ANGLE, CONTROLSYSTEM_KD_PITCH_ANGLE);

                        /* Calculate desired rates */
                        controlSystemValues.reference_rollRate  = controlSystemValues.PID_Output_rollAngle;
                        controlSystemValues.reference_pitchRate = controlSystemValues.PID_Output_pitchAngle;
                        controlSystemValues.reference_yawRate   = 0.03 * (controlSystemValues.reference_yawValue - 500);

                        /* Calculate rates errors */
                        controlSystemValues.error_rollRate      = controlSystemValues.reference_rollRate - controlSystemValues.gyroMeasurement.rotationRateRoll;
                        controlSystemValues.error_pitchRate     = controlSystemValues.reference_pitchRate - controlSystemValues.gyroMeasurement.rotationRatePitch;
                        controlSystemValues.error_yawRate       = controlSystemValues.reference_yawRate - controlSystemValues.gyroMeasurement.rotationRateYaw;

                        /* Calculate PID for roll rate */
                        CSM_CalculatePID(&controlSystemValues.PID_Output_rollRate, &controlSystemValues.PID_previousIterm_rollRate, &controlSystemValues.PID_previousError_rollRate, controlSystemValues.error_rollRate, CONTROLSYSTEM_KP_ROLL_RATE, CONTROLSYSTEM_KI_ROLL_RATE, CONTROLSYSTEM_KD_ROLL_RATE);
                        /* Calculate PID for pitch rate */
                        CSM_CalculatePID(&controlSystemValues.PID_Output_pitchRate, &controlSystemValues.PID_previousIterm_pitchRate, &controlSystemValues.PID_previousError_pitchRate, controlSystemValues.error_pitchRate, CONTROLSYSTEM_KP_PITCH_RATE, CONTROLSYSTEM_KI_PITCH_RATE, CONTROLSYSTEM_KD_PITCH_RATE);
                        /* Calculate PID for yaw rate */
                        CSM_CalculatePID(&controlSystemValues.PID_Output_yawRate, &controlSystemValues.PID_previousIterm_yawRate, &controlSystemValues.PID_previousError_yawRate, controlSystemValues.error_yawRate, CONTROLSYSTEM_KP_YAW_RATE, CONTROLSYSTEM_KI_YAW_RATE, CONTROLSYSTEM_KD_YAW_RATE);

                        /* Calculate motors speed */
                        controlSystemValues.motor1_speed = (controlSystemValues.reference_throttle - controlSystemValues.PID_Output_rollRate - controlSystemValues.PID_Output_pitchRate - controlSystemValues.PID_Output_yawRate) / 10;
                        controlSystemValues.motor2_speed = (controlSystemValues.reference_throttle + controlSystemValues.PID_Output_rollRate + controlSystemValues.PID_Output_pitchRate - controlSystemValues.PID_Output_yawRate) / 10;
                        controlSystemValues.motor3_speed = (controlSystemValues.reference_throttle + controlSystemValues.PID_Output_rollRate - controlSystemValues.PID_Output_pitchRate + controlSystemValues.PID_Output_yawRate) / 10;
                        controlSystemValues.motor4_speed = (controlSystemValues.reference_throttle - controlSystemValues.PID_Output_rollRate + controlSystemValues.PID_Output_pitchRate + controlSystemValues.PID_Output_yawRate) / 10;

                        /* Adjust and limit motors maximum speed */
                        if (ESC_MAXIMUM_SPEED < controlSystemValues.motor1_speed)
                            controlSystemValues.motor1_speed = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < controlSystemValues.motor2_speed)
                            controlSystemValues.motor2_speed = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < controlSystemValues.motor3_speed)
                            controlSystemValues.motor3_speed = ESC_MAXIMUM_SPEED;
                        if (ESC_MAXIMUM_SPEED < controlSystemValues.motor4_speed)
                            controlSystemValues.motor4_speed = ESC_MAXIMUM_SPEED;

                        /* Adjust and limit motors minimum speed */
                        if (ESC_MINIMUM_SPEED > controlSystemValues.motor1_speed)
                            controlSystemValues.motor1_speed = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > controlSystemValues.motor2_speed)
                            controlSystemValues.motor2_speed = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > controlSystemValues.motor3_speed)
                            controlSystemValues.motor3_speed = ESC_MINIMUM_SPEED;
                        if (ESC_MINIMUM_SPEED > controlSystemValues.motor4_speed)
                            controlSystemValues.motor4_speed = ESC_MINIMUM_SPEED;

                        /* Save motors speed */
                        controlSystemValues.ESC1_speed = controlSystemValues.motor1_speed;
                        controlSystemValues.ESC2_speed = controlSystemValues.motor2_speed;
                        controlSystemValues.ESC3_speed = controlSystemValues.motor3_speed;
                        controlSystemValues.ESC4_speed = controlSystemValues.motor4_speed;

                        /* Set motors speed */
                        // ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues.ESC4_speed);
                        // ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues.ESC2_speed);
                        // ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues.ESC3_speed);
                        // ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues.ESC1_speed);
                    }
                }
            }
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
        /* Control Loop Period */
        controlSystemValues.controlLoopPeriod = xControlLoopPeriod;
        /* Task Execution Time */
        controlSystemValues.taskExecutionTime = xTaskGetTickCount() - xTaskStartTime;
        /* Clear buffer */
        memset(controlSystemActiveValues_TaskControlSystem, 0, sizeof(ControlSystemValues_t));
        memcpy(controlSystemActiveValues_TaskControlSystem, &controlSystemValues, sizeof(ControlSystemValues_t));
        /* Try to swap buffers when ready to send */
        if (xSemaphoreTake(Semaphore_Handle_controlSystemValuesSwap, 0) == pdTRUE) {
            /* Swap buffers as Task_ControlSystem is not using 'controlSystemActiveValues_TaskControlSystem' */
            ControlSystemValues_t *tempValues             = controlSystemActiveValues_TaskControlSystem;
            controlSystemActiveValues_TaskControlSystem   = controlSystemActiveValues_TaskDebugging;
            controlSystemActiveValues_TaskDebugging       = tempValues;

            /* Signal Task_USB_Communication */
            ControlSystemValues_t *logControlSystemValues = controlSystemActiveValues_TaskDebugging;
            xQueueSend(Queue_Handle_ControlSystemValues_Debug, &logControlSystemValues, 0);
        }
#endif

        /* Clear buffer */
        memset(FlightLightsActiveBuffer_TaskControlSystem, 0, sizeof(FlightLights_Buffer_A));
        /* Load radio controller values into the buffer */
        FlightLightsActiveBuffer_TaskControlSystem[0] = controlSystemValues.radioController_channelValues[7];
        FlightLightsActiveBuffer_TaskControlSystem[1] = controlSystemValues.radioController_channelValues[8];
        FlightLightsActiveBuffer_TaskControlSystem[2] = controlSystemValues.radioController_channelValues[9];
        /* Flight Lights buffer swap implementation */
        if (xSemaphoreTake(Semaphore_Handle_flightLightsBufferSwap, 0) == pdTRUE) {
            /* Swap buffers for Flight Lights */
            uint16_t *tempFlightLightsBuffer           = FlightLightsActiveBuffer_TaskControlSystem;
            FlightLightsActiveBuffer_TaskControlSystem = FlightLightsActiveBuffer_TaskFlightLights;
            FlightLightsActiveBuffer_TaskFlightLights  = tempFlightLightsBuffer;
            /* Signal Task_FlightLights */
            xQueueSend(Queue_Handle_FlightLights_Commands, &FlightLightsActiveBuffer_TaskFlightLights, 0);
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_ControlSystem = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_USB_Communication(void *ptr) {
    (void)ptr;

    /* Message pointer variables */
    uint8_t *logInformationString = NULL;
    uint8_t *logDebuggingString   = NULL;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod  = pdMS_TO_TICKS(10);

    /* Get initial tick count */
    TickType_t xLastWakeTime      = xTaskGetTickCount();

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

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_USB_Communication = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_Debugging(void *ptr) {
    (void)ptr;

#if (MAIN_APP_LOGGING_DEBUGGING == 1)
    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(10);
    /* Get initial tick count */
    TickType_t xLastWakeTime     = xTaskGetTickCount();

    ControlSystemValues_t *logControlSystemValues;
    uint32_t               system_tick   = 0;
    uint16_t               written_chars = 0;

    float FlightController_batteryLevel;
    uint8_t debuggingStr_ESCsState[4] = {0};
    uint8_t debuggingStr_throttleStick_startedDown[4] = {0};
    uint8_t debuggingStr_flightLightsState[4] = {0};
    uint8_t debuggingStr_flightLightsType[4]  = {0};
    uint16_t debuggingValue_flightLightsSpeed = 0;

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
                if (logControlSystemValues->radioController_channelValues[5] >= 500) {
                    snprintf((char *)debuggingStr_ESCsState, 4 * sizeof(uint8_t), "ON");
                } else {
                    snprintf((char *)debuggingStr_ESCsState, 4 * sizeof(uint8_t), "OFF");
                }
                snprintf((char *)debuggingStr_FSA8S_main, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%d/%d_%d/%d_%s",
                         DEBUG_FSA8S_CHANNEL_VALUES_3, logControlSystemValues->radioController_channelValues[2],
                         DEBUG_FSA8S_CHANNEL_VALUES_1, logControlSystemValues->radioController_channelValues[0],
                         DEBUG_FSA8S_CHANNEL_VALUES_2, logControlSystemValues->radioController_channelValues[1],
                         DEBUG_FSA8S_CHANNEL_VALUES_4, logControlSystemValues->radioController_channelValues[3],
                         DEBUG_FSA8S_CHANNEL_VALUES_6, debuggingStr_ESCsState);
#endif

#if (MAIN_APP_DEBUGGING_FSA8S_AUX == 1)
                /* Log channel values */
                if (logControlSystemValues->radioController_channelValues[9] >= 500) {
                    snprintf((char *)debuggingStr_flightLightsState, 4 * sizeof(uint8_t), "ON");
                } else {
                    snprintf((char *)debuggingStr_flightLightsState, 4 * sizeof(uint8_t), "OFF");
                }
                if (logControlSystemValues->radioController_channelValues[8] <= 250) {
                    snprintf((char *)debuggingStr_flightLightsType, 4 * sizeof(uint8_t), "C");
                } else if (logControlSystemValues->radioController_channelValues[8] > 250 && logControlSystemValues->radioController_channelValues[8] <= 750) {
                    snprintf((char *)debuggingStr_flightLightsType, 4 * sizeof(uint8_t), "B");
                } else if (logControlSystemValues->radioController_channelValues[8] > 750 && logControlSystemValues->radioController_channelValues[8] <= 1000) {
                    snprintf((char *)debuggingStr_flightLightsType, 4 * sizeof(uint8_t), "A");
                }
                debuggingValue_flightLightsSpeed = 100 - logControlSystemValues->radioController_channelValues[7] / 10;
                if (debuggingValue_flightLightsSpeed == 0) {
                    debuggingValue_flightLightsSpeed = 1;
                }
                snprintf((char *)debuggingStr_FSA8S_aux, 50 * sizeof(uint8_t), "/%d_%d/%d_%d/%d_%s/%d_%s/%d_%d",
                         DEBUG_FSA8S_CHANNEL_VALUES_5, logControlSystemValues->radioController_channelValues[4],
                         DEBUG_FSA8S_CHANNEL_VALUES_7, logControlSystemValues->radioController_channelValues[6],
                         DEBUG_FSA8S_CHANNEL_VALUES_10, debuggingStr_flightLightsState,
                         DEBUG_FSA8S_CHANNEL_VALUES_9, debuggingStr_flightLightsType,
                         DEBUG_FSA8S_CHANNEL_VALUES_8, debuggingValue_flightLightsSpeed);     
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_CALIBRATION_VALUES)
                /* Log GY87 gyroscope calibration values */
                snprintf((char *)debuggingStr_GY87_gyroscopeCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL, logControlSystemValues->gyroCalibration.calibrationRateRoll,
                         DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH, logControlSystemValues->gyroCalibration.calibrationRatePitch,
                         DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW, logControlSystemValues->gyroCalibration.calibrationRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_CALIBRATION_VALUES)
                /* Log GY87 accelerometer calibration values */
                snprintf((char *)debuggingStr_GY87_accelerometerCalibrationValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f",
                         DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_X, logControlSystemValues->accCalibration.calibrationLinearAccelerationX,
                         DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Y, logControlSystemValues->accCalibration.calibrationLinearAccelerationY,
                         DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Z, logControlSystemValues->accCalibration.calibrationLinearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_GYROSCOPE_VALUES == 1)
                /* Log GY87 gyroscope values */
                snprintf((char *)debuggingStr_GY87_gyroscopeValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_GY87_GYRO_VALUES_ROT_RATE_ROLL, logControlSystemValues->gyroMeasurement.rotationRateRoll,
                         DEBUG_GY87_GYRO_VALUES_ROT_RATE_PITCH, logControlSystemValues->gyroMeasurement.rotationRatePitch,
                         DEBUG_GY87_GYRO_VALUES_ROT_RATE_YAW, logControlSystemValues->gyroMeasurement.rotationRateYaw);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_VALUES == 1)
                /* Log GY87 accelerometer values */
                snprintf((char *)debuggingStr_GY87_accelerometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f",
                         DEBUG_GY87_ACC_VALUES_LINEAR_X, logControlSystemValues->accMeasurement.linearAccelerationX,
                         DEBUG_GY87_ACC_VALUES_LINEAR_Y, logControlSystemValues->accMeasurement.linearAccelerationY,
                         DEBUG_GY87_ACC_VALUES_LINEAR_Z, logControlSystemValues->accMeasurement.linearAccelerationZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_ACCELEROMETER_ANGLES == 1)
                /* Log GY87 accelerometer angles */
                snprintf((char *)debuggingStr_GY87_accelerometerAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f",
                         DEBUG_GY87_ACC_VALUES_ANGLE_ROLL, logControlSystemValues->accMeasurement.angleRoll,
                         DEBUG_GY87_ACC_VALUES_ANGLE_PITCH, logControlSystemValues->accMeasurement.anglePitch);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_VALUES == 1)
                /* Log GY87 magnetometer values */
                snprintf((char *)debuggingStr_GY87_magnetometerValues, 40 * sizeof(uint8_t), (const char *)"/%d_%.3f/%d_%.3f/%d_%.3f",
                         DEBUG_GY87_MAG_VALUES_MAG_FIELD_X, logControlSystemValues->magMeasurement.magneticFieldX,
                         DEBUG_GY87_MAG_VALUES_MAG_FIELD_Y, logControlSystemValues->magMeasurement.magneticFieldY,
                         DEBUG_GY87_MAG_VALUES_MAG_FIELD_Z, logControlSystemValues->magMeasurement.magneticFieldZ);
#endif

#if (MAIN_APP_DEBUGGING_GY87_MAGNETOMETER_HEADING == 1)
                /* Log GY87 magnetometer heading */
                snprintf((char *)debuggingStr_GY87_magnetometerHeadingValue, 16 * sizeof(uint8_t), (const char *)"/%d_%.2f",
                         DEBUG_GY87_MAG_HEADING, logControlSystemValues->magMeasurement_magneticHeading);
#endif

#if (MAIN_APP_DEBUGGING_GY87_TEMPERATURE == 1)
                /* Log GY87 temperature value */
                snprintf((char *)debuggingStr_GY87_temperature, 16 * sizeof(uint8_t), (const char *)"/%d_%.1f",
                         DEBUG_GY87_TEMPERATURE, logControlSystemValues->temperature);
#endif

#if (MAIN_APP_DEBUGGING_FLIGHT_CONTROLLER_BATTERY_LEVEL == 1)
                /* Read flight controller battery level */
                xQueuePeek(Queue_Handle_BatteryLevel, &FlightController_batteryLevel, 0);
                /* Log flight controller battery level */
                snprintf((char *)debuggingStr_BatteryLevel, 10 * sizeof(uint8_t), (const char *)"/%d_%.2f",
                         DEBUG_FLIGHT_CONTROLLER_BATTERY_LEVEL, (double)FlightController_batteryLevel);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_VALUES == 1)
                /* Log control system reference values */
                snprintf((char *)debuggingStr_ControlSystem_referenceValues, 40 * sizeof(uint8_t), (const char *)"/%d_%d/%d_%d/%d_%d/%d_%d",
                         DEBUG_CONTROLSYSTEM_REFERENCE_THROTTLE, (uint16_t)logControlSystemValues->reference_throttle,
                         DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_VALUE, (uint16_t)logControlSystemValues->reference_rollValue,
                         DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_VALUE, (uint16_t)logControlSystemValues->reference_pitchValue,
                         DEBUG_CONTROLSYSTEM_REFERENCE_YAW_VALUE, (uint16_t)logControlSystemValues->reference_yawValue);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_ANGLES == 1)
                /* Log control system reference values */
                snprintf((char *)debuggingStr_ControlSystem_referenceAngles, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_ANGLE, logControlSystemValues->reference_rollAngle,
                         DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_ANGLE, logControlSystemValues->reference_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_KALMAN_ANGLES == 1)
                /* Log control system angles */
                snprintf((char *)debuggingStr_ControlSystem_KalmanAngles, 30 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_KALMAN_ROLL_ANGLE, logControlSystemValues->KalmanPrediction_rollAngle,
                         DEBUG_CONTROLSYSTEM_KALMAN_PITCH_ANGLE, logControlSystemValues->KalmanPrediction_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_ERRORS == 1)
                /* Log control system angles errors */
                snprintf((char *)debuggingStr_ControlSystem_anglesErrors, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_ERROR_ROLL_ANGLE, logControlSystemValues->error_rollAngle,
                         DEBUG_CONTROLSYSTEM_ERROR_PITCH_ANGLE, logControlSystemValues->error_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_ANGLES_PID_OUTPUT == 1)
                /* Log control system angles PID */
                snprintf((char *)debuggingStr_ControlSystem_anglesPID, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_ANGLE, logControlSystemValues->PID_Output_rollAngle,
                         DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_ANGLE, logControlSystemValues->PID_Output_pitchAngle);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_REFERENCE_RATE == 1)
                /* Log control system reference rates */
                snprintf((char *)debuggingStr_ControlSystem_referenceRates, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_RATE, logControlSystemValues->reference_rollRate,
                         DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_RATE, logControlSystemValues->reference_pitchRate,
                         DEBUG_CONTROLSYSTEM_REFERENCE_YAW_RATE, logControlSystemValues->reference_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_ERRORS == 1)
                /* Log control system rates errors */
                snprintf((char *)debuggingStr_ControlSystem_ratesErrors, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_ERROR_ROLL_RATE, logControlSystemValues->error_rollRate,
                         DEBUG_CONTROLSYSTEM_ERROR_PITCH_RATE, logControlSystemValues->error_pitchRate,
                         DEBUG_CONTROLSYSTEM_ERROR_YAW_RATE, logControlSystemValues->error_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_RATES_PID_OUTPUT == 1)
                /* Log control system rates PID */
                snprintf((char *)debuggingStr_ControlSystem_ratesPID, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_RATE, logControlSystemValues->PID_Output_rollRate,
                         DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_RATE, logControlSystemValues->PID_Output_pitchRate,
                         DEBUG_CONTROLSYSTEM_PID_OUTPUT_YAW_RATE, logControlSystemValues->PID_Output_yawRate);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_MOTORS_SPEEDS == 1)
                /* Log ESC values*/
                snprintf((char *)debuggingStr_ControlSystem_motorsSpeeds, 40 * sizeof(uint8_t), (const char *)"/%d_%.2f/%d_%.2f/%d_%.2f/%d_%.2f",
                         DEBUG_CONTROLSYSTEM_MOTOR_SPEED_1, logControlSystemValues->motor1_speed,
                         DEBUG_CONTROLSYSTEM_MOTOR_SPEED_2, logControlSystemValues->motor2_speed,
                         DEBUG_CONTROLSYSTEM_MOTOR_SPEED_3, logControlSystemValues->motor3_speed,
                         DEBUG_CONTROLSYSTEM_MOTOR_SPEED_4, logControlSystemValues->motor4_speed);
#endif

#if (MAIN_APP_DEBUGGING_CONTROLSYSTEM_AUXILIAR == 1)
                /* Log control system auxiliar values */
                if (logControlSystemValues->throttleStick_startedDown == true) {
                    snprintf((char *)debuggingStr_throttleStick_startedDown, 4 * sizeof(uint8_t), "YES");
                } else {
                    snprintf((char *)debuggingStr_throttleStick_startedDown, 4 * sizeof(uint8_t), "NO");
                }
                if (logControlSystemValues->ESC_isEnabled == true) {
                    snprintf((char *)debuggingStr_ESCsState, 4 * sizeof(uint8_t), "ON");
                } else {
                    snprintf((char *)debuggingStr_ESCsState, 4 * sizeof(uint8_t), "OFF");
                }
                snprintf((char *)debuggingStr_ControlSystem_Auxiliar, 40 * sizeof(uint8_t), (const char *)"/%d_%lu/%d_%lu/%d_%s/%d_%s",
                         DEBUG_CONTROLSYSTEM_LOOP_PERIOD_MEASURED, (logControlSystemValues->controlLoopPeriod * 1000 / configTICK_RATE_HZ),
                         DEBUG_CONTROLSYSTEM_TASK_EXECUTION_TIME, (logControlSystemValues->taskExecutionTime * 1000 / configTICK_RATE_HZ),
                         DEBUG_CONTROLSYSTEM_THROTTLE_STICK_STARTED_DOWN, debuggingStr_throttleStick_startedDown,
                         DEBUG_CONTROLSYSTEM_ESCS_ENABLED, debuggingStr_ESCsState);
#endif

#if (MAIN_APP_DEBUGGING_ESCS == 1)
                /* Log ESC values*/
                snprintf((char *)debuggingStr_ESCs, 60 * sizeof(uint8_t), (const char *)"/%d_%d/%d_%d/%d_%d/%d_%d",
                         DEBUG_ESC_1, logControlSystemValues->ESC1_speed,
                         DEBUG_ESC_2, logControlSystemValues->ESC2_speed,
                         DEBUG_ESC_3, logControlSystemValues->ESC3_speed,
                         DEBUG_ESC_4, logControlSystemValues->ESC4_speed);
#endif

#if (MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK)
                /* Log tasks stack high watermark */
                snprintf((char *)debuggingStr_TasksStackHighWatermark, 80 * sizeof(uint8_t), (const char *)"/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld/%d_%ld",
                         DEBUG_TASK_STACK_WATERMARK_ONOFFBUTTON, Task_StackHighWatermark_OnOffButton,
                         DEBUG_TASK_STACK_WATERMARK_CONTROLSYSTEM, Task_StackHighWatermark_ControlSystem,
                         DEBUG_TASK_STACK_WATERMARK_USBCOMMUNICATION, Task_StackHighWatermark_USB_Communication,
                         DEBUG_TASK_STACK_WATERMARK_DEBUGGING, Task_StackHighWatermark_Debugging,
                         DEBUG_TASK_STACK_WATERMARK_BATTERYLEVEL, Task_StackHighWatermark_BatteryLevel,
                         DEBUG_TASK_STACK_WATERMARK_BATTERYALARM, Task_StackHighWatermark_BatteryAlarm,
                         DEBUG_TASK_STACK_WATERMARK_HEARTBEATLIGHT, Task_StackHighWatermark_HeartbeatLight,
                         DEBUG_TASK_STACK_WATERMARK_FLIGHTLIGHTS, Task_StackHighWatermark_FlightLights);
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
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_BatteryLevel);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_referenceValues);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_referenceAngles);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_KalmanAngles);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_anglesErrors);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_anglesPID);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_referenceRates);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ratesErrors);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_ratesPID);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_motorsSpeeds);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ControlSystem_Auxiliar);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_ESCs);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "%s", debuggingStr_TasksStackHighWatermark);
        written_chars += snprintf((char *)debuggingActiveBuffer_TaskDebugging + written_chars, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE - written_chars, "\n");

        /* Try to swap buffers when ready to send */
        if (xSemaphoreTake(Semaphore_Handle_debuggingBufferSwap, 0) == pdTRUE) {
            /* Swap buffers as Task_USB_Communication is not using 'debuggingActiveBuffer_TaskUSBCommunication' */
            uint8_t *tempBuffer                        = debuggingActiveBuffer_TaskUSBCommunication;
            debuggingActiveBuffer_TaskUSBCommunication = debuggingActiveBuffer_TaskDebugging;
            debuggingActiveBuffer_TaskDebugging        = tempBuffer;
            /* Signal Task_USB_Communication */
            uint8_t *logDebuggingString                = debuggingActiveBuffer_TaskUSBCommunication;
            xQueueSend(Queue_Handle_USB_Communication_Debug, &logDebuggingString, 0);
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_Debugging = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
#endif
}

void Task_OnOffButton(void *ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    /* Get initial tick count */
    TickType_t xLastWakeTime     = xTaskGetTickCount();

    while (1) {

        /* Check On/Off Button status */
        if (!HAL_GPIO_ReadPin(PW_ON_OFF_DRIVER_INPUT_GPIO_Port, PW_ON_OFF_DRIVER_INPUT_Pin)) {
            /* User is trying to turn it on or off */
            if (!Timer_Flag_OnOffButton) {

                xTimerStart(Timer_Handle_OnOffButton, 0);
                Timer_Flag_OnOffButton = true;
            }
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_OnOffButton = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_StartUp(void *ptr) {
    (void)ptr;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    /* Get initial tick count */
    TickType_t xLastWakeTime     = xTaskGetTickCount();

    while (1) {
        /* Check if flight controller is already running */
        if (FlightController_isRunning) {

            /* Turn on-board LED off */
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

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

/* Initialize buffer structures */
#if (MAIN_APP_LOGGING_DEBUGGING == 1)
            memset(&controlSystemValues_A, 0, sizeof(ControlSystemValues_t));
            memset(&controlSystemValues_B, 0, sizeof(ControlSystemValues_t));
            memset(debuggingStr_A, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE);
            memset(debuggingStr_B, 0, MAIN_APP_LOGGING_DEBUGGING_BUFFER_SIZE);
#endif

            /* Delete this task, as initialization must happen only once */
            vTaskDelete(Task_Handle_StartUp);
        }

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_BatteryLevel(void *ptr) {
    (void)ptr;

    uint16_t adcValue;
    float    FlightController_batteryLevel;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(1000);
    /* Get initial tick count */
    TickType_t xLastWakeTime     = xTaskGetTickCount();

    while (1) {
        if (FlightController_isInitialized) {
            /* Start ADC Conversion */
            HAL_ADC_Start(&hadc1);

            /* Poll ADC peripheral */
            HAL_ADC_PollForConversion(&hadc1, 1);

            /* Read ADC value */
            adcValue                      = HAL_ADC_GetValue(&hadc1);

            /* Convert ADC value to real value */
            FlightController_batteryLevel = (adcValue * 3.3) / 4096;

            /* Correct real value, as when battery full, ADC input is not 3.3V */
            FlightController_batteryLevel = FlightController_batteryLevel * 1.046046;

            /* Map real value to battery levels */
            FlightController_batteryLevel = FlightController_batteryLevel * 3.363636 + BATTERY_LEVEL_CALIBRATION_OFFSET;

            /* Send to queue, overwriting old value */
            xQueueOverwrite(Queue_Handle_BatteryLevel, &FlightController_batteryLevel);

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
            /* Get stack watermark */
            Task_StackHighWatermark_BatteryLevel = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_BatteryAlarm(void *ptr) {
    (void)ptr;

    uint8_t alarmSequence[]             = {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t alarmSequenceSize           = sizeof(alarmSequence);
    uint8_t alarmSequenceCursor         = 0;

    const uint16_t ALARM_SEQUENCE_DELAY = 200;

    /* For main task timing */
    const TickType_t xTaskPeriod        = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    TickType_t       xLastWakeTime      = xTaskGetTickCount();

    /* For alarm sequence timing */
    TickType_t xAlarmLastWakeTime       = xLastWakeTime;
    TickType_t xAlarmPeriod             = pdMS_TO_TICKS(ALARM_SEQUENCE_DELAY);

    float FlightController_batteryLevel;

    while (1) {
        if (FlightController_isInitialized) {
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
                    xAlarmLastWakeTime  = xTaskGetTickCount();
                }
            }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
            /* Get stack watermark */
            Task_StackHighWatermark_BatteryAlarm = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_HeartbeatLight(void *ptr) {
    (void)ptr;

    uint8_t ledState             = GPIO_PIN_RESET;

    /* Change delay from time in [ms] to ticks */
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(HEARTBEAT_PERIOD / 2);
    /* Get initial tick count */
    TickType_t xLastWakeTime     = xTaskGetTickCount();

    while (1) {

        if (FlightController_isInitialized) {
            /* Toggle LED and update state */
            ledState = (ledState == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
        /* Get stack watermark */
        Task_StackHighWatermark_HeartbeatLight = uxTaskGetStackHighWaterMark(NULL);
#endif

        /* Set task time delay */
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
    }
}

void Task_FlightLights(void *ptr) {
    (void)ptr;

    typedef struct {
        uint8_t led1[8];
        uint8_t led2[8];
        uint8_t led3[8];
        uint8_t led4[8];
        uint8_t size;
    } LightSequence;

    /* Consolidated sequence definitions */
    static const LightSequence sequences[3]     = {                              /* Sequence A */
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

    uint16_t *flightLightsCommands              = NULL;
    uint16_t  flightLightCommand_sequenceSpeed  = 0;
    uint16_t  flightLightCommand_sequenceSelect = 0;
    uint16_t  flightLightCommand_sequenceEnable = 0;

    const uint16_t CHANNEL_THRESHOLD_LOW        = 250;
    const uint16_t CHANNEL_THRESHOLD_MID        = 750;
    const uint16_t LIGHTS_ENABLE_THRESHOLD      = 500;
    const uint16_t BASE_SEQUENCE_DELAY          = 200;

    uint8_t  activeSequence                     = 0;
    uint8_t  sequenceCursor                     = 0;
    uint16_t currentSequenceDelay               = BASE_SEQUENCE_DELAY;

    /* For main task timing */
    const TickType_t xTaskPeriod                = pdMS_TO_TICKS(DEFAULT_TASK_DELAY);
    TickType_t       xLastWakeTime              = xTaskGetTickCount();

    /* For LED sequence timing */
    TickType_t xSequenceLastWakeTime            = xLastWakeTime;
    TickType_t xSequencePeriod                  = pdMS_TO_TICKS(currentSequenceDelay);

    while (1) {
        /* Read the flight lights commands */
        if (xQueueReceive(Queue_Handle_FlightLights_Commands, &flightLightsCommands, 0) == pdPASS) {
            flightLightCommand_sequenceSpeed  = flightLightsCommands[0];
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
            xSequencePeriod      = pdMS_TO_TICKS(currentSequenceDelay);

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
            sequenceCursor        = 0;
            /* Reset sequence timing reference point */
            xSequenceLastWakeTime = xTaskGetTickCount();
        }

#if (MAIN_APP_LOGGING_DEBUGGING == 1 && MAIN_APP_DEBUGGING_TASK_STACK_HIGH_WATERMARK == 1)
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
    uint32_t ulCount      = (uint32_t)pvTimerGetTimerID(xTimer);

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

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg) {
    if (!FlightController_isInitialized) {
        /* During initialization phase, refresh to prevent reset */
        HAL_WWDG_Refresh(hwwdg);
    } else {
        /* Emergency shutdown - Stop all motors */
        if (hesc != NULL) {
            // ESC_SetSpeed(hesc, hesc->esc1, 0);
            // ESC_SetSpeed(hesc, hesc->esc2, 0);
            // ESC_SetSpeed(hesc, hesc->esc3, 0);
            // ESC_SetSpeed(hesc, hesc->esc4, 0);
        }

        /* Don't refresh - let system reset */
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
void FlightController_Init(void) {

    /* Task 1: OnOffButton */
    xTaskCreate(Task_OnOffButton, "Task_OnOffButton", (1 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_ONOFFBUTTON_PRIORITY), &Task_Handle_OnOffButton);
    if (Task_Handle_OnOffButton == NULL) {
        /* Error */
        ErrorLED_Start(FreeRTOS_TasksCreationError);
    }

    /* Task 2: StartUp */
    xTaskCreate(Task_StartUp, "Task_StartUp", (2 * configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + (uint32_t)TASK_STARTUP_PRIORITY), &Task_Handle_StartUp);
    if (Task_Handle_StartUp == NULL) {
        /* Error */
        ErrorLED_Start(FreeRTOS_TasksCreationError);
    }

    /* Timer1: OnOffButton */
    Timer_Handle_OnOffButton = xTimerCreate("Timer_OnOffButton", pdMS_TO_TICKS(100), pdTRUE, (void *)0, Timer_Callback_OnOffButton);
    if (Timer_Handle_OnOffButton == NULL) {
        /* Error */
        ErrorLED_Start(FreeRTOS_TimersCreationError);
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
