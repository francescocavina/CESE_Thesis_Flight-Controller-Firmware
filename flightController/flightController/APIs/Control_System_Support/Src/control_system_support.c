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
 * @date:    03/04/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
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

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
/* Drivers Handles */
extern IBUS_HandleTypeDef_t *rc_controller;
extern ESC_HandleTypeDef_t  *hesc;

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
void CS_Reset(ControlSystemValues_t *controlSystemValues) {

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
    CSM_ResetPID();
}

void CS_CheckRadioControllerStatus(ControlSystemValues_t *controlSystemValues) {

    uint16_t channelValue         = 0;

    /* Read radio controller channel */
    bool_t radioController_Status = FSA8S_ReadChannel(rc_controller, CHANNEL_1, &channelValue);

    /* Check if the radio controller started connected */
    if (radioController_Status == false) {
        controlSystemValues->radioController_startedConnected = false;
    } else {
        controlSystemValues->radioController_startedConnected = true;
    }
}

void CS_CheckForUncontrolledMotorsStart(ControlSystemValues_t *controlSystemValues) {

    /* Check if the ESCs are started off and the throttle stick is started down */
    controlSystemValues->ESC_startedOff            = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[5] <= 500);
    controlSystemValues->throttleStick_startedDown = controlSystemValues->radioController_startedConnected && (controlSystemValues->radioController_channelValues[2] <= 15);

    if (controlSystemValues->ESC_startedOff == true && controlSystemValues->throttleStick_startedDown == true) {
        controlSystemValues->safeStart = true;
    } else {
        controlSystemValues->safeStart = false;
    }
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

void Kalman_CalculateAngle(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {

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

void CSM_CalculatePID(float *PID_Output, float *previousIterm, float *previousErrorValue, float errorValue, float kP, float kI, float kD) {

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

void CSM_ResetPID(void) {

    // /* Reset previously stored PID errors and terms values: Angles */
    // previousErrorValue_rollAngle = 0;
    // previousErrorValue_pitchAngle = 0;
    // previousIterm_rollAngle = 0;
    // previousIterm_pitchAngle = 0;

    // /* Reset previously stored PID errors and terms values: Rates */
    // previousErrorValue_rollRate = 0;
    // previousErrorValue_pitchRate = 0;
    // previousErrorValue_yawRate = 0;
    // previousIterm_rollRate = 0;
    // previousIterm_pitchRate = 0;
    // previousIterm_yawRate = 0;
}

/* --- End of file ----------------------------------------------------------------------------- */
