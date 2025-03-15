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
 * @date:    03/04/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:
 *
 * @details:
 */

#ifndef CONTROL_SYSTEM_SUPPORT_H
#define CONTROL_SYSTEM_SUPPORT_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */
#define FSA8S_CHANNELS    (10) // Number of remote control channels to read
#define ESC_MAXIMUM_SPEED (90)
#define ESC_MINIMUM_SPEED (10)

/* --- Public data type declarations ----------------------------------------------------------- */
/**
 * @brief bool_t type declaration.
 */
typedef bool bool_t;

/*
 * @brief Control System Values structure.
 */
typedef struct {
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
    bool_t gyroCalibration_calibrationDone;
    bool_t gyroCalibration_fixedCalibration_en;
    float  gyroCalibration_rotationRateRoll;
    float  gyroCalibration_rotationRatePitch;
    float  gyroCalibration_rotationRateYaw;
    /* IMU Measurements (Gyroscope) */
    float gyroMeasurement_rotationRateRoll;
    float gyroMeasurement_rotationRatePitch;
    float gyroMeasurement_rotationRateYaw;
    /* IMU Calibration (Accelerometer) */
    bool_t accCalibration_calibrationDone;
    bool_t accCalibration_fixedCalibration_en;
    float  accCalibration_linearAccelerationX;
    float  accCalibration_linearAccelerationY;
    float  accCalibration_linearAccelerationZ;
    /* IMU Measurements (Accelerometer) */
    float accMeasurement_linearAccelerationX;
    float accMeasurement_linearAccelerationY;
    float accMeasurement_linearAccelerationZ;
    float accMeasurement_angleRoll;
    float accMeasurement_anglePitch;
    /* IMU Measurements (Magnetometer) */
    float magMeasurement_magneticFieldX;
    float magMeasurement_magneticFieldY;
    float magMeasurement_magneticFieldZ;
    float magMeasurement_magneticHeading;
    /* IMU Measurements (Temperature Sensor) */
    float temperature;
    /* Kalman Filter Variables */
    float KalmanPrediction_rollAngle;
    float KalmanPrediction_pitchAngle;
    float KalmanUncertainty_rollAngle;
    float KalmanUncertainty_pitchAngle;
    /* Errors: Angles */
    float error_rollAngle;
    float error_pitchAngle;
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
} ControlSystemValues_t;
/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Calculates an angle using a Kalman filter.
 * @param  TODO
 * @retval None
 */
void Kalman_CalculateAngle(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement);

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
void CSM_CalculatePID(float *PID_Output, float *previousIterm, float *previousErrorValue, float errorValue, float kP, float kI, float kD);

/*
 * @brief  Resets the PID controller errors and integral terms values.
 * @param  None
 * @retval None
 */
void CSM_ResetPID(void);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEM_SUPPORT_H */

/* --- End of file ----------------------------------------------------------------------------- */
