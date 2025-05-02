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
 * @file:    CONTROLSYSTEM_support.c
 * @date:    04/01/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:
 *
 * @details:
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "control_system_support.h"
#include "control_system_settings.h"

#include "ESC_UAI.h"
#include "FSA8S_driver_UAI.h"
#include "MPU6050_driver_UAI.h"

#include <math.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif
#define RADIANS_TO_DEGREES_CONST (180.0f / M_PI)
#define DEGREES_TO_RADIANS_CONST (M_PI / 180.0f)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Drivers Handles */
extern IBUS_HandleTypeDef_t *rc_controller;
extern GY87_HandleTypeDef_t *hgy87;
extern ESC_HandleTypeDef_t  *hesc;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief  Corrects the acceleration measurements using the gyroscope measurements.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 *         correctionEnabled:   Flag to enable or disable the correction.
 * @retval None
 */
void CS_CorrectAcceleration(ControlSystemValues_t *controlSystemValues, bool_t correctionEnabled);

/*
 * @brief  Calculates the angle using the acceleration measurements.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_CalculateAnglesFromAccelerations(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Calculates an angle using a Kalman filter.
 * @param  TODO
 * @retval None
 */
void CS_Kalman_CalculateAngle(float *predictedAngle, float *predictedAngleUncertainty, float *gain, float gyro_rotationRate, float acc_calculatedAngle);

/*
 * @brief  Applies deadband to radio controller movement values.
 * @param  reference_movementValue: Pointer to the reference movement value.
 * @retval None
 */
void CS_ApplyRadioControllerDeadBand(float *reference_movementValue);

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
void CS_CalculatePID(float *PID_Output, float *previousIterm, float *previousErrorValue, float errorValue, float kP, float kI, float kD);

/*
 * @brief  Resets the PID controller errors, integral terms and output values.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_ResetPID(ControlSystemValues_t *controlSystemValues);

/*
 * @brief  Limits motors speed.
 * @param  controlSystemValues: Pointer to the Control System Values structure.
 * @retval None
 */
void CS_CheckForMotorsSpeedLimits(ControlSystemValues_t *controlSystemValues);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
void CS_CorrectAcceleration(ControlSystemValues_t *controlSystemValues, bool_t correctionEnabled) {

    /* Correct linear acceleration measurements due to sensor misposition */
    if (correctionEnabled) {
        controlSystemValues->IMU_Offset[0] = 0.035f; // X-axis offset (meters)
        controlSystemValues->IMU_Offset[1] = 0.00f;  // Y-axis offset (meters)
        controlSystemValues->IMU_Offset[2] = 0.00f;  // Z-axis offset (meters)

        /* Convert gyro from degrees/s to radians/s */
        float gyro[3];
        gyro[0] = controlSystemValues->gyroMeasurement.rotationRateRoll * DEGREES_TO_RADIANS_CONST;
        gyro[1] = controlSystemValues->gyroMeasurement.rotationRatePitch * DEGREES_TO_RADIANS_CONST;
        gyro[2] = controlSystemValues->gyroMeasurement.rotationRateYaw * DEGREES_TO_RADIANS_CONST;

        /* Compute omega × r */
        float omega_cross_r[3];
        omega_cross_r[0] = gyro[1] * controlSystemValues->IMU_Offset[2] - gyro[2] * controlSystemValues->IMU_Offset[1];
        omega_cross_r[1] = gyro[2] * controlSystemValues->IMU_Offset[0] - gyro[0] * controlSystemValues->IMU_Offset[2];
        omega_cross_r[2] = gyro[0] * controlSystemValues->IMU_Offset[1] - gyro[1] * controlSystemValues->IMU_Offset[0];

        /* Compute centrifugal acceleration: omega × (omega × r) */
        float centrifugal[3];
        centrifugal[0]                                    = gyro[1] * omega_cross_r[2] - gyro[2] * omega_cross_r[1];
        centrifugal[1]                                    = gyro[2] * omega_cross_r[0] - gyro[0] * omega_cross_r[2];
        centrifugal[2]                                    = gyro[0] * omega_cross_r[1] - gyro[1] * omega_cross_r[0];

        /* Subtract centrifugal acceleration from measured acceleration */
        controlSystemValues->correctedLinearAccelerationX = controlSystemValues->accMeasurement.linearAccelerationX - centrifugal[0];
        controlSystemValues->correctedLinearAccelerationY = controlSystemValues->accMeasurement.linearAccelerationY - centrifugal[1];
        controlSystemValues->correctedLinearAccelerationZ = controlSystemValues->accMeasurement.linearAccelerationZ - centrifugal[2];
    } else {
        /* If correction is disabled, use raw accelerometer values */
        controlSystemValues->correctedLinearAccelerationX = controlSystemValues->accMeasurement.linearAccelerationX;
        controlSystemValues->correctedLinearAccelerationY = controlSystemValues->accMeasurement.linearAccelerationY;
        controlSystemValues->correctedLinearAccelerationZ = controlSystemValues->accMeasurement.linearAccelerationZ;
    }
}

void CS_CalculateAnglesFromAccelerations(ControlSystemValues_t *controlSystemValues) {

    /* Calculate roll angle */
    controlSystemValues->correctedRollAngle  = atan2f(controlSystemValues->correctedLinearAccelerationY, controlSystemValues->correctedLinearAccelerationZ) * RADIANS_TO_DEGREES_CONST;

    /* Calculate pitch angle */
    controlSystemValues->correctedPitchAngle = atan2f(-controlSystemValues->correctedLinearAccelerationX, sqrtf(controlSystemValues->correctedLinearAccelerationY * controlSystemValues->correctedLinearAccelerationY + controlSystemValues->correctedLinearAccelerationZ * controlSystemValues->correctedLinearAccelerationZ)) * RADIANS_TO_DEGREES_CONST;
}

void CS_Kalman_CalculateAngle(float *predictedAngle, float *predictedAngleUncertainty, float *gain, float gyro_rotationRate, float acc_calculatedAngle) {

    /* Process noise variance (Q) (How much the gyro can be trusted: std = 4.0 degrees/s) */
    const float processNoise     = 16.0f;
    /* Measurement noise variance (R) (How much the accelerometer can be trusted: std = 3.0 degrees) */
    const float measurementNoise = 9.0f;

    /* Predict the angle with the last prediction and gyroscope rotation rate integration */
    *predictedAngle              = *predictedAngle + CONTROLSYSTEM_LOOP_PERIOD_S * gyro_rotationRate;

    /* Estimate uncertainy of the predicted angle with the gyroscope error */
    *predictedAngleUncertainty   = *predictedAngleUncertainty + (CONTROLSYSTEM_LOOP_PERIOD_S * CONTROLSYSTEM_LOOP_PERIOD_S * processNoise);

    /* Ensure the uncertainty doesn't become negative*/
    if (*predictedAngleUncertainty < 0.0f) {
        *predictedAngleUncertainty = 0.001f;
    }

    /* Calculate the Kalman Gain with the estimated uncertainty and the accelerometer error */
    *gain = *predictedAngleUncertainty / (*predictedAngleUncertainty + measurementNoise);

    /* Clamp Kalman Gain to a valid range */
    if (*gain < 0.0f)
        *gain = 0.0f;
    if (*gain > 1.0f)
        *gain = 1.0f;

    /* Update predicted angle with the calculated angle using the accelerometer measurements */
    *predictedAngle            = *predictedAngle + *gain * (acc_calculatedAngle - *predictedAngle);

    /* Reestimate the uncertainy of the predicted angle */
    *predictedAngleUncertainty = (1.0f - *gain) * *predictedAngleUncertainty;
}

void CS_ApplyRadioControllerDeadBand(float *reference_movementValue) {

    /* Apply deadband to avoid raido controller stick noise */
    if (fabsf(*reference_movementValue - 500) < CONTROLSYSTEM_RC_DEADBAND) {
        *reference_movementValue = 500;
    }
}

void CS_CalculatePID(float *PID_Output, float *previousIterm, float *previousErrorValue, float errorValue, float kP, float kI, float kD) {

    float Pterm;
    float Iterm;
    float Dterm;
    float pidOutputValue;

    /* Calculate proportional term */
    Pterm = kP * errorValue;

    /* Calculate integral term */
    Iterm = *previousIterm + kI * ((*previousErrorValue + errorValue) / 2) * CONTROLSYSTEM_LOOP_PERIOD_S;
    /* Clamp integral term to avoid integral wind-up */
    if (-CONTROLSYSTEM_PID_ITERM_LIMIT > Iterm) {
        Iterm = -CONTROLSYSTEM_PID_ITERM_LIMIT;
    } else if (CONTROLSYSTEM_PID_ITERM_LIMIT < Iterm) {
        Iterm = CONTROLSYSTEM_PID_ITERM_LIMIT;
    }

    /* Calculate derivative term */
    Dterm          = kD * (errorValue - *previousErrorValue) / CONTROLSYSTEM_LOOP_PERIOD_S;

    /* Calculate PID output */
    pidOutputValue = Pterm + Iterm + Dterm;
    /* Limit the PID output */
    if (-CONTROLSYSTEM_PID_OUTPUT_LIMIT > pidOutputValue) {
        pidOutputValue = -CONTROLSYSTEM_PID_OUTPUT_LIMIT;
    } else if (CONTROLSYSTEM_PID_OUTPUT_LIMIT < pidOutputValue) {
        pidOutputValue = CONTROLSYSTEM_PID_OUTPUT_LIMIT;
    }

    /* Return values */
    *PID_Output         = pidOutputValue;
    *previousErrorValue = errorValue;
    *previousIterm      = Iterm;
}

void CS_ResetPID(ControlSystemValues_t *controlSystemValues) {

    /* Reset PID (Angles): Previous Errors */
    controlSystemValues->PID_previousError_rollAngle  = 0;
    controlSystemValues->PID_previousError_pitchAngle = 0;
    /* Reset PID (Angles): Previous Integral Terms */
    controlSystemValues->PID_previousIterm_rollAngle  = 0;
    controlSystemValues->PID_previousIterm_pitchAngle = 0;
    /* Reset PID Outputs: Angles */
    controlSystemValues->PID_Output_rollAngle         = 0;
    controlSystemValues->PID_Output_pitchAngle        = 0;
    /* Reset PID (Rates): Previous Errors */
    controlSystemValues->PID_previousError_rollRate   = 0;
    controlSystemValues->PID_previousError_pitchRate  = 0;
    controlSystemValues->PID_previousError_yawRate    = 0;
    /* Reset PID (Rates): Previous Integral Terms */
    controlSystemValues->PID_previousIterm_rollRate   = 0;
    controlSystemValues->PID_previousIterm_pitchRate  = 0;
    controlSystemValues->PID_previousIterm_yawRate    = 0;
    /* Reset PID Outputs: Rates */
    controlSystemValues->PID_Output_rollRate          = 0;
    controlSystemValues->PID_Output_pitchRate         = 0;
    controlSystemValues->PID_Output_yawRate           = 0;
}

void CS_CheckForMotorsSpeedLimits(ControlSystemValues_t *controlSystemValues) {

    /* Adjust and limit motors minimum speed */
    // if (ESC_MINIMUM_SPEED > controlSystemValues->motor1_speed)
    //     controlSystemValues->motor1_speed = ESC_MINIMUM_SPEED;
    // if (ESC_MINIMUM_SPEED > controlSystemValues->motor2_speed)
    //     controlSystemValues->motor2_speed = ESC_MINIMUM_SPEED;
    // if (ESC_MINIMUM_SPEED > controlSystemValues->motor3_speed)
    //     controlSystemValues->motor3_speed = ESC_MINIMUM_SPEED;
    // if (ESC_MINIMUM_SPEED > controlSystemValues->motor4_speed)
    //     controlSystemValues->motor4_speed = ESC_MINIMUM_SPEED;

    /* Adjust and limit motors maximum speed */
    if (ESC_MAXIMUM_SPEED < controlSystemValues->motor1_speed)
        controlSystemValues->motor1_speed = ESC_MAXIMUM_SPEED;
    if (ESC_MAXIMUM_SPEED < controlSystemValues->motor2_speed)
        controlSystemValues->motor2_speed = ESC_MAXIMUM_SPEED;
    if (ESC_MAXIMUM_SPEED < controlSystemValues->motor3_speed)
        controlSystemValues->motor3_speed = ESC_MAXIMUM_SPEED;
    if (ESC_MAXIMUM_SPEED < controlSystemValues->motor4_speed)
        controlSystemValues->motor4_speed = ESC_MAXIMUM_SPEED;
}

/* --- Public function implementation ---------------------------------------------------------- */
void CS_StateMachine_Init(ControlSystemValues_t *controlSystemValues) {

    /* Save motors speed */
    controlSystemValues->ESC1_speed = 0;
    controlSystemValues->ESC2_speed = 0;
    controlSystemValues->ESC3_speed = 0;
    controlSystemValues->ESC4_speed = 0;

    /* Turn motors off */
    ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues->ESC4_speed);
    ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues->ESC2_speed);
    ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues->ESC3_speed);
    ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues->ESC1_speed);

    /* Reset PID variables */
    CS_ResetPID(controlSystemValues);
}

void CS_StateMachine_SafeStartCheck(ControlSystemValues_t *controlSystemValues) {

    uint16_t channelValue         = 0;

    /* Read radio controller channel */
    bool_t radioController_Status = FSA8S_ReadChannel(rc_controller, CHANNEL_1, &channelValue);

    /* Check if the radio controller started connected */
    if (radioController_Status == false) {
        controlSystemValues->radioController_startedConnected = false;
    } else {
        controlSystemValues->radioController_startedConnected = true;
    }

    /* Check if the ESCs are started off and the throttle stick is started down */
    controlSystemValues->ESC_startedOff            = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[RC_CHANNEL_SAFETY_ESC_ON_OFF] <= 500);
    controlSystemValues->throttleStick_startedDown = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_THROTTLE] <= 15);

    if (controlSystemValues->ESC_startedOff == true && controlSystemValues->throttleStick_startedDown == true) {
        controlSystemValues->safeStart   = true;
        controlSystemValues->safeRestart = true;
    } else {
        controlSystemValues->safeStart   = false;
        controlSystemValues->safeRestart = false;
    }
}

void CS_StateMachine_Running(ControlSystemValues_t *controlSystemValues) {

    /* Read GY-87 gyroscope sensor */
    GY87_ReadGyroscope(hgy87, &controlSystemValues->gyroMeasurement, &controlSystemValues->gyroCalibration);
    /* Read GY-87 accelerometer sensor */
    GY87_ReadAccelerometer(hgy87, &controlSystemValues->accMeasurement, &controlSystemValues->accCalibration);

    /* Correct acceleration */
    CS_CorrectAcceleration(controlSystemValues, true);
    /* Calculate angles using accelerations */
    CS_CalculateAnglesFromAccelerations(controlSystemValues);

    /* Calculate Kalman angles */
    CS_Kalman_CalculateAngle(&controlSystemValues->KalmanPrediction_rollAngle, &controlSystemValues->KalmanUncertainty_rollAngle, &controlSystemValues->KalmanGain_rollAngle, controlSystemValues->gyroMeasurement.rotationRateRoll, controlSystemValues->correctedRollAngle);
    CS_Kalman_CalculateAngle(&controlSystemValues->KalmanPrediction_pitchAngle, &controlSystemValues->KalmanUncertainty_pitchAngle, &controlSystemValues->KalmanGain_pitchAngle, controlSystemValues->gyroMeasurement.rotationRatePitch, controlSystemValues->correctedPitchAngle);

    /* Read inputs from radio controller */
    controlSystemValues->reference_throttle   = (float)controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_THROTTLE];
    controlSystemValues->reference_rollValue  = (float)controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_ROLL];
    controlSystemValues->reference_pitchValue = (float)controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_PITCH];
    controlSystemValues->reference_yawValue   = (float)controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_YAW];

    /* Adjust and limit throttle input */
    if (CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE < controlSystemValues->reference_throttle) {
        controlSystemValues->reference_throttle = CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE;
    }

    /* Apply radio controller dead band to movement sticks */
    CS_ApplyRadioControllerDeadBand(&controlSystemValues->reference_rollValue);
    CS_ApplyRadioControllerDeadBand(&controlSystemValues->reference_pitchValue);
    CS_ApplyRadioControllerDeadBand(&controlSystemValues->reference_yawValue);

    /* Calculate desired angles by mapping radio controller values to angles */
    controlSystemValues->reference_rollAngle  = 0.03 * (controlSystemValues->reference_rollValue - 500);
    controlSystemValues->reference_pitchAngle = 0.03 * (controlSystemValues->reference_pitchValue - 500);

    /* Calculate angles errors */
    controlSystemValues->error_rollAngle      = controlSystemValues->reference_rollAngle - controlSystemValues->KalmanPrediction_rollAngle;
    controlSystemValues->error_pitchAngle     = controlSystemValues->reference_pitchAngle - controlSystemValues->KalmanPrediction_pitchAngle;

    /* Calculate PID for roll angle */
    CS_CalculatePID(&controlSystemValues->PID_Output_rollAngle, &controlSystemValues->PID_previousIterm_rollAngle, &controlSystemValues->PID_previousError_rollAngle, controlSystemValues->error_rollAngle, CONTROLSYSTEM_KP_ROLL_ANGLE, CONTROLSYSTEM_KI_ROLL_ANGLE, CONTROLSYSTEM_KD_ROLL_ANGLE);
    /* Calculate PID for pitch angle */
    CS_CalculatePID(&controlSystemValues->PID_Output_pitchAngle, &controlSystemValues->PID_previousIterm_pitchAngle, &controlSystemValues->PID_previousError_pitchAngle, controlSystemValues->error_pitchAngle, CONTROLSYSTEM_KP_PITCH_ANGLE, CONTROLSYSTEM_KI_PITCH_ANGLE, CONTROLSYSTEM_KD_PITCH_ANGLE);

    /* Calculate desired rates */
    controlSystemValues->reference_rollRate  = controlSystemValues->PID_Output_rollAngle;
    controlSystemValues->reference_pitchRate = controlSystemValues->PID_Output_pitchAngle;
    controlSystemValues->reference_yawRate   = 0.03 * (controlSystemValues->reference_yawValue - 500);

    /* Calculate rates errors */
    controlSystemValues->error_rollRate      = controlSystemValues->reference_rollRate - controlSystemValues->gyroMeasurement.rotationRateRoll;
    controlSystemValues->error_pitchRate     = controlSystemValues->reference_pitchRate - controlSystemValues->gyroMeasurement.rotationRatePitch;
    controlSystemValues->error_yawRate       = controlSystemValues->reference_yawRate - controlSystemValues->gyroMeasurement.rotationRateYaw;

    /* Calculate PID for roll rate */
    CS_CalculatePID(&controlSystemValues->PID_Output_rollRate, &controlSystemValues->PID_previousIterm_rollRate, &controlSystemValues->PID_previousError_rollRate, controlSystemValues->error_rollRate, CONTROLSYSTEM_KP_ROLL_RATE, CONTROLSYSTEM_KI_ROLL_RATE, CONTROLSYSTEM_KD_ROLL_RATE);
    /* Calculate PID for pitch rate */
    CS_CalculatePID(&controlSystemValues->PID_Output_pitchRate, &controlSystemValues->PID_previousIterm_pitchRate, &controlSystemValues->PID_previousError_pitchRate, controlSystemValues->error_pitchRate, CONTROLSYSTEM_KP_PITCH_RATE, CONTROLSYSTEM_KI_PITCH_RATE, CONTROLSYSTEM_KD_PITCH_RATE);
    /* Calculate PID for yaw rate */
    CS_CalculatePID(&controlSystemValues->PID_Output_yawRate, &controlSystemValues->PID_previousIterm_yawRate, &controlSystemValues->PID_previousError_yawRate, controlSystemValues->error_yawRate, CONTROLSYSTEM_KP_YAW_RATE, CONTROLSYSTEM_KI_YAW_RATE, CONTROLSYSTEM_KD_YAW_RATE);

    /* Calculate motors speed */
    controlSystemValues->motor1_speed = (controlSystemValues->reference_throttle - controlSystemValues->PID_Output_rollRate - controlSystemValues->PID_Output_pitchRate - controlSystemValues->PID_Output_yawRate) / 10;
    controlSystemValues->motor2_speed = (controlSystemValues->reference_throttle + controlSystemValues->PID_Output_rollRate + controlSystemValues->PID_Output_pitchRate - controlSystemValues->PID_Output_yawRate) / 10;
    controlSystemValues->motor3_speed = (controlSystemValues->reference_throttle + controlSystemValues->PID_Output_rollRate - controlSystemValues->PID_Output_pitchRate + controlSystemValues->PID_Output_yawRate) / 10;
    controlSystemValues->motor4_speed = (controlSystemValues->reference_throttle - controlSystemValues->PID_Output_rollRate + controlSystemValues->PID_Output_pitchRate + controlSystemValues->PID_Output_yawRate) / 10;

    /* Limit motors speed */
    CS_CheckForMotorsSpeedLimits(controlSystemValues);

    /* Save motors speed */
    controlSystemValues->ESC1_speed = controlSystemValues->motor1_speed;
    controlSystemValues->ESC2_speed = controlSystemValues->motor2_speed;
    controlSystemValues->ESC3_speed = controlSystemValues->motor3_speed;
    controlSystemValues->ESC4_speed = controlSystemValues->motor4_speed;

    /* Set motors speed */
    ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues->ESC4_speed);
    ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues->ESC2_speed);
    ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues->ESC3_speed);
    ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues->ESC1_speed);
}

void CS_StateMachine_Restart(ControlSystemValues_t *controlSystemValues) {

    /* Reset control system variables */
    /* References (Values) */
    controlSystemValues->reference_throttle           = 0;
    controlSystemValues->reference_rollValue          = 0;
    controlSystemValues->reference_pitchValue         = 0;
    controlSystemValues->reference_yawValue           = 0;
    /* References (Angles) */
    controlSystemValues->reference_rollAngle          = 0;
    controlSystemValues->reference_pitchAngle         = 0;
    /* IMU Correction */
    controlSystemValues->correctedLinearAccelerationX = 0;
    controlSystemValues->correctedLinearAccelerationY = 0;
    controlSystemValues->correctedLinearAccelerationZ = 0;
    controlSystemValues->correctedRollAngle           = 0;
    controlSystemValues->correctedPitchAngle          = 0;
    /* Kalman Filter Variables */
    controlSystemValues->KalmanPrediction_rollAngle   = 0;
    controlSystemValues->KalmanPrediction_pitchAngle  = 0;
    controlSystemValues->KalmanUncertainty_rollAngle  = 0;
    controlSystemValues->KalmanUncertainty_pitchAngle = 0;
    controlSystemValues->KalmanGain_rollAngle         = 0;
    controlSystemValues->KalmanGain_pitchAngle        = 0;
    /* Errors: Angles */
    controlSystemValues->error_rollAngle              = 0;
    controlSystemValues->error_pitchAngle             = 0;
    /* References: Rates */
    controlSystemValues->reference_rollRate           = 0;
    controlSystemValues->reference_pitchRate          = 0;
    controlSystemValues->reference_yawRate            = 0;
    /* Errors: Rates */
    controlSystemValues->error_rollRate               = 0;
    controlSystemValues->error_pitchRate              = 0;
    controlSystemValues->error_yawRate                = 0;
    /* Motors Speeds */
    controlSystemValues->motor1_speed                 = 0;
    controlSystemValues->motor2_speed                 = 0;
    controlSystemValues->motor3_speed                 = 0;
    controlSystemValues->motor4_speed                 = 0;
    /* ESCs Values*/
    controlSystemValues->ESC1_speed                   = 0;
    controlSystemValues->ESC2_speed                   = 0;
    controlSystemValues->ESC3_speed                   = 0;
    controlSystemValues->ESC4_speed                   = 0;

    /* Turn motors off */
    ESC_SetSpeed(hesc, hesc->esc1, controlSystemValues->ESC4_speed);
    ESC_SetSpeed(hesc, hesc->esc2, controlSystemValues->ESC2_speed);
    ESC_SetSpeed(hesc, hesc->esc3, controlSystemValues->ESC3_speed);
    ESC_SetSpeed(hesc, hesc->esc4, controlSystemValues->ESC1_speed);

    /* Reset PID variables */
    CS_ResetPID(controlSystemValues);
}

void CS_StateMachine_SafeRestartCheck(ControlSystemValues_t *controlSystemValues) {

    /* Check if the throttle stick is started down */
    controlSystemValues->throttleStick_startedDown = (controlSystemValues->radioController_channelValues[RC_CHANNEL_MOVEMENT_THROTTLE] <= 15);
    controlSystemValues->safeRestart               = controlSystemValues->throttleStick_startedDown;
}

/* --- End of file ----------------------------------------------------------------------------- */
