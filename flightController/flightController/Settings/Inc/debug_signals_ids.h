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
 * @file:    debug_signals_ids.h
 * @date:    03/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This file contains the IDs for the debug signals.
 */

#ifndef DEBUG_SIGNALS_IDS_H
#define DEBUG_SIGNALS_IDS_H

/* --- Headers files inclusions ---------------------------------------------------------------- */

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */
typedef enum {
    DEBUG_GY87_TEMPERATURE                            = 1,  /* 1; General; Flight Controller; Temperature;   [°C] */
    DEBUG_FLIGHT_CONTROLLER_BATTERY_LEVEL             = 2,  /* 1; General; Flight Controller; Battery Level; [V] */

    DEBUG_FSA8S_CHANNEL_VALUES_1                      = 3,  /* 2; Radio Control; Movement;      (ch01)    Roll;        [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_2                      = 4,  /* 2; Radio Control; Movement;      (ch02)    Pitch;       [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_3                      = 5,  /* 2; Radio Control; Movement;      (ch03)    Throttle;    [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_4                      = 6,  /* 2; Radio Control; Movement;      (ch04)    Yaw;         [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_5                      = 7,  /* 2; Radio Control; TBD;           (ch05)    TBD;         [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_6                      = 8,  /* 2; Radio Control; Movement;      (ch06)    ESCs On/Off; [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_7                      = 9,  /* 2; Radio Control; TBD;           (ch07)    TBD;         [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_8                      = 10, /* 2; Radio Control; Flight Lights; (ch08)    Speed;       [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_9                      = 11, /* 2; Radio Control; Flight Lights; (ch09)    Type;        [-] */
    DEBUG_FSA8S_CHANNEL_VALUES_10                     = 12, /* 2; Radio Control; Flight Lights; (ch10)    On/Off;      [-] */

    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL  = 13, /* 3; IMU; Gyroscope Cal.;     Rotation Rate Roll;     [°/s] */
    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH = 14, /* 3; IMU; Gyroscope Cal.;     Rotation Rate Pitch;    [°/s] */
    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW   = 15, /* 3; IMU; Gyroscope Cal.;     Rotation Rate Yaw;      [°/s] */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_X        = 16, /* 3; IMU; Accelerometer Cal.; Linear Acceleration X;  [g] */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Y        = 17, /* 3; IMU; Accelerometer Cal.; Linear Acceleration Y;  [g] */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Z        = 18, /* 3; IMU; Accelerometer Cal.; Linear Acceleration Z;  [g] */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_ROLL              = 19, /* 3; IMU; Gyroscope Meas.;     Rotation Rate Roll;    [°/s] */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_PITCH             = 20, /* 3; IMU; Gyroscope Meas.;     Rotation Rate Pitch;   [°/s] */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_YAW               = 21, /* 3; IMU; Gyroscope Meas.;     Rotation Rate Yaw;     [°/s] */
    DEBUG_GY87_ACC_VALUES_LINEAR_X                    = 22, /* 3; IMU; Accelerometer Meas.; Linear Acceleration X; [g] */
    DEBUG_GY87_ACC_VALUES_LINEAR_Y                    = 23, /* 3; IMU; Accelerometer Meas.; Linear Acceleration Y; [g] */
    DEBUG_GY87_ACC_VALUES_LINEAR_Z                    = 24, /* 3; IMU; Accelerometer Meas.; Linear Acceleration Z; [g] */
    DEBUG_GY87_ACC_VALUES_ANGLE_ROLL                  = 25, /* 3; IMU; Accelerometer Meas.; Rotation Angle Roll;   [°] */
    DEBUG_GY87_ACC_VALUES_ANGLE_PITCH                 = 26, /* 3; IMU; Accelerometer Meas.; Rotation Angle Pitch;  [°] */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_X                 = 27, /* 3; IMU; Magnetometer Meas.;  Magnetic Field X;      [G] */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_Y                 = 28, /* 3; IMU; Magnetometer Meas.;  Magnetic Field Y;      [G] */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_Z                 = 29, /* 3; IMU; Magnetometer Meas.;  Magnetic Field Z;      [G] */
    DEBUG_GY87_MAG_HEADING                            = 30, /* 3; IMU; Magnetometer Meas.;  Magnetic Heading;      [°] */

    DEBUG_CONST_CONTROLSYSTEM_KP_ROLL_ANGLE           = 31, /* 4; Control System PID Constants; PID Gains (Angle); kP Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KI_ROLL_ANGLE           = 32, /* 4; Control System PID Constants; PID Gains (Angle); kI Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KD_ROLL_ANGLE           = 33, /* 4; Control System PID Constants; PID Gains (Angle); kD Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KP_PITCH_ANGLE          = 34, /* 4; Control System PID Constants; PID Gains (Angle); kP Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KI_PITCH_ANGLE          = 35, /* 4; Control System PID Constants; PID Gains (Angle); kI Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KD_PITCH_ANGLE          = 36, /* 4; Control System PID Constants; PID Gains (Angle); kD Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KP_ROLL_RATE            = 37, /* 4; Control System PID Constants; PID Gains (Rate);  kP Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KI_ROLL_RATE            = 38, /* 4; Control System PID Constants; PID Gains (Rate);  kI Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KD_ROLL_RATE            = 39, /* 4; Control System PID Constants; PID Gains (Rate);  kD Roll;  [-] */
    DEBUG_CONST_CONTROLSYSTEM_KP_PITCH_RATE           = 40, /* 4; Control System PID Constants; PID Gains (Rate);  kP Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KI_PITCH_RATE           = 41, /* 4; Control System PID Constants; PID Gains (Rate);  kI Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KD_PITCH_RATE           = 42, /* 4; Control System PID Constants; PID Gains (Rate);  kD Pitch; [-] */
    DEBUG_CONST_CONTROLSYSTEM_KP_YAW_RATE             = 43, /* 4; Control System PID Constants; PID Gains (Rate);  kP Yaw;   [-] */
    DEBUG_CONST_CONTROLSYSTEM_KI_YAW_RATE             = 44, /* 4; Control System PID Constants; PID Gains (Rate);  kI Yaw;   [-] */
    DEBUG_CONST_CONTROLSYSTEM_KD_YAW_RATE             = 45, /* 4; Control System PID Constants; PID Gains (Rate);  kD Yaw;   [-] */

    DEBUG_CONTROLSYSTEM_REFERENCE_THROTTLE            = 46, /* 5; Control System Variables; Reference;     Throttle;     [-] */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_VALUE          = 47, /* 5; Control System Variables; Reference;     Roll Value;   [-] */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_VALUE         = 48, /* 5; Control System Variables; Reference;     Pitch Value;  [-] */
    DEBUG_CONTROLSYSTEM_REFERENCE_YAW_VALUE           = 49, /* 5; Control System Variables; Reference;     Yaw Value;    [-] */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_ANGLE          = 50, /* 5; Control System Variables; Reference;     Roll  Angle;  [°] */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_ANGLE         = 51, /* 5; Control System Variables; Reference;     Pitch Angle;  [°] */
    DEBUG_CONTROLSYSTEM_KALMAN_ROLL_ANGLE             = 52, /* 5; Control System Variables; Kalman;        Roll  Angle;  [°] */
    DEBUG_CONTROLSYSTEM_KALMAN_PITCH_ANGLE            = 53, /* 5; Control System Variables; Kalman;        Pitch Angle;  [°] */
    DEBUG_CONTROLSYSTEM_ERROR_ROLL_ANGLE              = 54, /* 5; Control System Variables; Error;         Roll  Angle;  [°] */
    DEBUG_CONTROLSYSTEM_ERROR_PITCH_ANGLE             = 55, /* 5; Control System Variables; Error;         Pitch Angle;  [°] */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_ANGLE         = 56, /* 5; Control System Variables; PID Output;    Roll Angle;   [°] */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_ANGLE        = 57, /* 5; Control System Variables; PID Output;    Pitch Angle;  [°] */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_RATE           = 58, /* 5; Control System Variables; Reference;     Roll Rate;    [°/s] */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_RATE          = 59, /* 5; Control System Variables; Reference;     Pitch Rate;   [°/s] */
    DEBUG_CONTROLSYSTEM_REFERENCE_YAW_RATE            = 60, /* 5; Control System Variables; Reference;     Yaw Rate;     [°/s] */
    DEBUG_CONTROLSYSTEM_ERROR_ROLL_RATE               = 61, /* 5; Control System Variables; Error;         Roll Rate;    [°/s] */
    DEBUG_CONTROLSYSTEM_ERROR_PITCH_RATE              = 62, /* 5; Control System Variables; Error;         Pitch Rate;   [°/s] */
    DEBUG_CONTROLSYSTEM_ERROR_YAW_RATE                = 63, /* 5; Control System Variables; Error;         Yaw Rate;     [°/s] */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_RATE          = 64, /* 5; Control System Variables; PID Output;    Roll Rate;    [°/s] */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_RATE         = 65, /* 5; Control System Variables; PID Output;    Pitch Rate;   [°/s] */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_YAW_RATE           = 66, /* 5; Control System Variables; PID Output;    Yaw Rate;     [°/s] */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_1                 = 67, /* 5; Control System Variables; Motors Speeds; Motor 1;      [-] */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_2                 = 68, /* 5; Control System Variables; Motors Speeds; Motor 2;      [-] */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_3                 = 69, /* 5; Control System Variables; Motors Speeds; Motor 3;      [-] */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_4                 = 70, /* 5; Control System Variables; Motors Speeds; Motor 4;      [-] */

    DEBUG_CONST_CONTROLSYSTEM_LOOP_PERIOD_MS          = 71, /* 6; Control System Auxiliar; Control Loop; Period (Configured); [ms] */
    DEBUG_CONTROLSYSTEM_LOOP_PERIOD_MEASURED          = 72, /* 6; Control System Auxiliar; Control Loop; Period (Measured);   [ms] */
    DEBUG_CONTROLSYSTEM_TASK_EXECUTION_TIME           = 73, /* 6; Control System Auxiliar; Task;         Execution Time;      [ms] */

    DEBUG_CONST_ESC_MINIMUM_SPEED                     = 74, /* 7; ESCs; Speed Limit; Minimum; [-] */
    DEBUG_CONST_ESC_MAXIMUM_SPEED                     = 75, /* 7; ESCs; Speed Limit; Maximum; [-] */
    DEBUG_ESC_1                                       = 76, /* 7; ESCs; ESC; Motor 1; [-] */
    DEBUG_ESC_2                                       = 77, /* 7; ESCs; ESC; Motor 2; [-] */
    DEBUG_ESC_3                                       = 78, /* 7; ESCs; ESC; Motor 3; [-] */
    DEBUG_ESC_4                                       = 79, /* 7; ESCs; ESC; Motor 4; [-] */

    DEBUG_TASK_STACK_WATERMARK_ONOFFBUTTON            = 80, /* 8; System Stack; Stack High Watermark; Task: OnOffButton      ; [words] */
    DEBUG_TASK_STACK_WATERMARK_CONTROLSYSTEM          = 81, /* 8; System Stack; Stack High Watermark; Task: ControlSystem    ; [words] */
    DEBUG_TASK_STACK_WATERMARK_USBCOMMUNICATION       = 82, /* 8; System Stack; Stack High Watermark; Task: USB_Communication; [words] */
    DEBUG_TASK_STACK_WATERMARK_DEBUGGING              = 83, /* 8; System Stack; Stack High Watermark; Task: Debugging        ; [words] */
    DEBUG_TASK_STACK_WATERMARK_BATTERYLEVEL           = 84, /* 8; System Stack; Stack High Watermark; Task: BatteryLevel     ; [words] */
    DEBUG_TASK_STACK_WATERMARK_BATTERYALARM           = 85, /* 8; System Stack; Stack High Watermark; Task: BatteryAlarm     ; [words] */
    DEBUG_TASK_STACK_WATERMARK_HEARTBEATLIGHT         = 86, /* 8; System Stack; Stack High Watermark; Task: HeartbeatLight   ; [words] */
    DEBUG_TASK_STACK_WATERMARK_FLIGHTLIGHTS           = 87, /* 8; System Stack; Stack High Watermark; Task: FlightLights     ; [words] */
} DebugSignalsIds_t;

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* DEBUG_SIGNALS_IDS_H */

/* --- End of file ----------------------------------------------------------------------------- */
