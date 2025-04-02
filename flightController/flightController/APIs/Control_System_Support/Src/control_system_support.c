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

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Drivers Handles */
extern IBUS_HandleTypeDef_t *rc_controller;
extern GY87_HandleTypeDef_t *hgy87;
extern ESC_HandleTypeDef_t  *hesc;

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

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
    controlSystemValues->ESC_startedOff            = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[5] <= 500);
    controlSystemValues->throttleStick_startedDown = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[2] <= 15);

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

    /* Calculate Kalman angles */
    CS_Kalman_CalculateAngle(&controlSystemValues->KalmanPrediction_rollAngle, &controlSystemValues->KalmanUncertainty_rollAngle, controlSystemValues->gyroMeasurement.rotationRateRoll, controlSystemValues->accMeasurement.angleRoll);
    CS_Kalman_CalculateAngle(&controlSystemValues->KalmanPrediction_pitchAngle, &controlSystemValues->KalmanUncertainty_pitchAngle, controlSystemValues->gyroMeasurement.rotationRatePitch, controlSystemValues->accMeasurement.anglePitch);

    /* Read inputs from radio controller */
    controlSystemValues->reference_throttle   = (float)controlSystemValues->radioController_channelValues[2];
    controlSystemValues->reference_rollValue  = (float)controlSystemValues->radioController_channelValues[0];
    controlSystemValues->reference_pitchValue = (float)controlSystemValues->radioController_channelValues[1];
    controlSystemValues->reference_yawValue   = (float)controlSystemValues->radioController_channelValues[3];

    /* Adjust and limit throttle input */
    if (CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE < controlSystemValues->reference_throttle) {
        controlSystemValues->reference_throttle = CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE;
    }

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

void CS_StateMachine_SafeRestartCheck(ControlSystemValues_t *controlSystemValues) {

    /* Check if the throttle stick is started down */
    controlSystemValues->throttleStick_startedDown = (controlSystemValues->radioController_channelValues[2] <= 15);
    controlSystemValues->safeRestart               = controlSystemValues->throttleStick_startedDown;
}

void CS_Kalman_CalculateAngle(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {

    // float kalmanGain;

    // *kalmanState = *kalmanState + CONTROLSYSTEM_LOOP_PERIOD_S * kalmanInput;
    // *kalmanUncertainty = *kalmanUncertainty + CONTROLSYSTEM_LOOP_PERIOD_S * CONTROLSYSTEM_LOOP_PERIOD_S * 4 * 4;
    // kalmanGain = *kalmanUncertainty * 1 / (1 * *kalmanUncertainty + 3 * 3);
    // *kalmanState = *kalmanState + kalmanGain * (kalmanMeasurement - *kalmanState);
    // *kalmanUncertainty = (1 - kalmanGain) * *kalmanUncertainty;

    /* Process noise variance (Q) - how much we trust the gyro */
    const float processNoise     = 16.0f; // 4^2
    /* Measurement noise variance (R) - how much we trust the accelerometer */
    const float measurementNoise = 9.0f; // 3^2
    float       kalmanGain;

    /* Prediction step - Project the state ahead using gyro data */
    *kalmanState       = *kalmanState + CONTROLSYSTEM_LOOP_PERIOD_S * kalmanInput;

    /* Project the error covariance ahead */
    *kalmanUncertainty = *kalmanUncertainty + (CONTROLSYSTEM_LOOP_PERIOD_S * CONTROLSYSTEM_LOOP_PERIOD_S * processNoise);

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
    *kalmanState       = *kalmanState + kalmanGain * (kalmanMeasurement - *kalmanState);

    /* Update the error covariance */
    *kalmanUncertainty = (1.0f - kalmanGain) * *kalmanUncertainty;
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
    if (ESC_MINIMUM_SPEED > controlSystemValues->motor1_speed)
        controlSystemValues->motor1_speed = ESC_MINIMUM_SPEED;
    if (ESC_MINIMUM_SPEED > controlSystemValues->motor2_speed)
        controlSystemValues->motor2_speed = ESC_MINIMUM_SPEED;
    if (ESC_MINIMUM_SPEED > controlSystemValues->motor3_speed)
        controlSystemValues->motor3_speed = ESC_MINIMUM_SPEED;
    if (ESC_MINIMUM_SPEED > controlSystemValues->motor4_speed)
        controlSystemValues->motor4_speed = ESC_MINIMUM_SPEED;

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

/* --- End of file ----------------------------------------------------------------------------- */
