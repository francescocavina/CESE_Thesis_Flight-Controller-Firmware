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

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
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
