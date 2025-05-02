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
 * @file:    control_system_support.h
 * @date:    04/01/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:
 *
 * @details:
 */

#ifndef CONTROL_SYSTEM_SUPPORT_H
#define CONTROL_SYSTEM_SUPPORT_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "control_system_settings.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "MPU6050_driver_UAI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
/**
 * @brief bool_t type declaration.
 */
typedef bool bool_t;

/*
 * @brief Control System State Machine states.
 */
typedef enum {
    CONTROL_SYSTEM_STATE_INIT,
    CONTROL_SYSTEM_STATE_SAFE_START_CHECK,
    CONTROL_SYSTEM_STATE_RUNNING,
    CONTROL_SYSTEM_STATE_RESTART,
    CONTROL_SYSTEM_STATE_SAFE_RESTART_CHECK,
} ControlSystem_StateMachine_t;

typedef enum {
    RC_CHANNEL_MOVEMENT_ROLL              = 0,
    RC_CHANNEL_MOVEMENT_PITCH             = 1,
    RC_CHANNEL_MOVEMENT_THROTTLE          = 2,
    RC_CHANNEL_MOVEMENT_YAW               = 3,
    RC_CHANNEL_SAFETY_SYSTEM_FAILURE_TEST = 4,
    RC_CHANNEL_SAFETY_ESC_ON_OFF          = 5,
    RC_CHANNEL_TBD_1                      = 6,
    RC_CHANNEL_FLIGHT_LIGHTS_SPEED        = 7,
    RC_CHANNEL_FLIGHT_LIGHTS_SEQUENCE     = 8,
    RC_CHANNEL_FLIGHT_LIGHTS_ON_OFF       = 9,
} RadioController_Channels_t;

/*
 * @brief Control System Values structure.
 */
typedef struct {
    /* Safety */
    bool_t ESC_isEnabled;
    bool_t ESC_startedOff;
    bool_t radioController_startedConnected;
    bool_t throttleStick_startedDown;
    bool_t safeStart;
    bool_t safeRestart;
    /* State Machine */
    ControlSystem_StateMachine_t stateMachine_currentState;
    /* Radio Controller Readings */
    uint16_t radioController_channelValues[FSA8S_CHANNELS];
    /* References (Values) */
    float reference_throttle;
    float reference_rollValue;
    float reference_pitchValue;
    float reference_yawValue;
    /* References (Angles) */
    float reference_rollAngle;
    float reference_pitchAngle;
    /* IMU Calibration (Gyroscope) */
    GY87_gyroscopeCalibrationValues_t gyroCalibration;
    /* IMU Measurements (Gyroscope) */
    GY87_gyroscopeValues_t gyroMeasurement;
    /* IMU Calibration (Accelerometer) */
    GY87_accelerometerCalibrationValues_t accCalibration;
    /* IMU Measurements (Accelerometer) */
    GY87_accelerometerValues_t accMeasurement;
    /* IMU Measurements (Magnetometer) */
    GY87_magnetometerValues_t magMeasurement;
    float                     magMeasurement_magneticHeading;
    /* IMU Measurements (Temperature Sensor) */
    float temperature;
    /* IMU Correction */
    float IMU_Offset[3];
    float correctedLinearAccelerationX;
    float correctedLinearAccelerationY;
    float correctedLinearAccelerationZ;
    float correctedRollAngle;
    float correctedPitchAngle;
    /* Kalman Filter Variables */
    float KalmanPrediction_rollAngle;
    float KalmanPrediction_pitchAngle;
    float KalmanUncertainty_rollAngle;
    float KalmanUncertainty_pitchAngle;
    float KalmanGain_rollAngle;
    float KalmanGain_pitchAngle;
    /* Errors: Angles */
    float error_rollAngle;
    float error_pitchAngle;
    /* PID Gains: Angles */
    float PID_kP_rollAngle;
    float PID_kI_rollAngle;
    float PID_kD_rollAngle;
    float PID_kP_pitchAngle;
    float PID_kI_pitchAngle;
    float PID_kD_pitchAngle;
    /* PID (Angles): Previous Errors */
    float PID_previousError_rollAngle;
    float PID_previousError_pitchAngle;
    /* PID (Angles): Previous Integral Terms */
    float PID_previousIterm_rollAngle;
    float PID_previousIterm_pitchAngle;
    /* PID Outputs: Angles */
    float PID_Output_rollAngle;
    float PID_Output_pitchAngle;
    /* References: Rates */
    float reference_rollRate;
    float reference_pitchRate;
    float reference_yawRate;
    /* Errors: Rates */
    float error_rollRate;
    float error_pitchRate;
    float error_yawRate;
    /* PID Gains: Rates */
    float PID_kP_rollRate;
    float PID_kI_rollRate;
    float PID_kD_rollRate;
    float PID_kP_pitchRate;
    float PID_kI_pitchRate;
    float PID_kD_pitchRate;
    float PID_kP_yawRate;
    float PID_kI_yawRate;
    float PID_kD_yawRate;
    /* PID (Rates): Previous Errors */
    float PID_previousError_rollRate;
    float PID_previousError_pitchRate;
    float PID_previousError_yawRate;
    /* PID (Rates): Previous Integral Terms */
    float PID_previousIterm_rollRate;
    float PID_previousIterm_pitchRate;
    float PID_previousIterm_yawRate;
    /* PID Outputs: Rates */
    float PID_Output_rollRate;
    float PID_Output_pitchRate;
    float PID_Output_yawRate;
    /* Motors Speeds */
    float motor1_speed;
    float motor2_speed;
    float motor3_speed;
    float motor4_speed;
    /* ESCs Values*/
    uint8_t ESC1_speed;
    uint8_t ESC2_speed;
    uint8_t ESC3_speed;
    uint8_t ESC4_speed;
    /* Control Loop Period */
    uint32_t controlLoopPeriod;
    /* Task Execution Time */
    uint32_t taskExecutionTime;
} ControlSystemValues_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */

/*
 * @brief  Initializes the state machine variables.
 * @param  None
 * @retval None
 */
void CS_StateMachine_Init(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Checks that control system starts with ESCs disabled and throttle stick down.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_StateMachine_SafeStartCheck(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Runs the control system state machine.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_StateMachine_Running(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Restarts the control system state machine.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_StateMachine_Restart(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Checks that control system restarts with throttle stick down.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_StateMachine_SafeRestartCheck(ControlSystemValues_t *controlSystemValues);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEM_SUPPORT_H */

/* --- End of file ----------------------------------------------------------------------------- */
