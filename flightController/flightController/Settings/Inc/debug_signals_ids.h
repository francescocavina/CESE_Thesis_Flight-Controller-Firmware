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
    DEBUG_GY87_TEMPERATURE                                = 1,   /* 1; General; Flight Controller; Temperature;   [°C]; */
    DEBUG_FLIGHT_CONTROLLER_BATTERY_LEVEL                 = 2,   /* 1; General; Flight Controller; Battery Level; [V]; */

    DEBUG_FSA8S_CHANNEL_VALUES_1                          = 3,   /* 3; Radio Control; Movement;      (ch01)    Roll;        [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_2                          = 4,   /* 3; Radio Control; Movement;      (ch02)    Pitch;       [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_3                          = 5,   /* 3; Radio Control; Movement;      (ch03)    Throttle;    [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_4                          = 6,   /* 3; Radio Control; Movement;      (ch04)    Yaw;         [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_5                          = 7,   /* 3; Radio Control; TBD;           (ch05)    TBD;         [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_6                          = 8,   /* 3; Radio Control; Movement;      (ch06)    ESCs On/Off; [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_7                          = 9,   /* 3; Radio Control; TBD;           (ch07)    TBD;         [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_8                          = 10,  /* 3; Radio Control; Flight Lights; (ch08)    Speed;       [%]; */
    DEBUG_FSA8S_CHANNEL_VALUES_9                          = 11,  /* 3; Radio Control; Flight Lights; (ch09)    Type;        [-]; */
    DEBUG_FSA8S_CHANNEL_VALUES_10                         = 12,  /* 3; Radio Control; Flight Lights; (ch10)    On/Off;      [-]; */

    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL      = 13,  /* 6; IMU; Gyroscope Cal.;      Offset   Rotation Rate Roll;     [°/s]; */
    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH     = 14,  /* 6; IMU; Gyroscope Cal.;      Offset   Rotation Rate Pitch;    [°/s]; */
    DEBUG_GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW       = 15,  /* 6; IMU; Gyroscope Cal.;      Offset   Rotation Rate Yaw;      [°/s]; */
    DEBUG_GY87_GYRO_CALIBRATION_STD_DEV_ROT_RATE_ROLL     = 16,  /* 6; IMU; Gyroscope Cal.;      Std Dev. Rotation Rate Roll;     [°/s]; */
    DEBUG_GY87_GYRO_CALIBRATION_STD_DEV_ROT_RATE_PITCH    = 17,  /* 6; IMU; Gyroscope Cal.;      Std Dev. Rotation Rate Pitch;    [°/s]; */
    DEBUG_GY87_GYRO_CALIBRATION_STD_DEV_ROT_RATE_YAW      = 18,  /* 6; IMU; Gyroscope Cal.;      Std Dev. Rotation Rate Yaw;      [°/s]; */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_X            = 19,  /* 6; IMU; Accelerometer Cal.;  Offset   Linear Acceleration X;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Y            = 20,  /* 6; IMU; Accelerometer Cal.;  Offset   Linear Acceleration Y;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_VALUES_LINEAR_Z            = 21,  /* 6; IMU; Accelerometer Cal.;  Offset   Linear Acceleration Z;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_STD_DEV_LINEAR_X           = 22,  /* 6; IMU; Accelerometer Cal.;  Std Dev. Linear Acceleration X;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_STD_DEV_LINEAR_Y           = 23,  /* 6; IMU; Accelerometer Cal.;  Std Dev. Linear Acceleration Y;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_STD_DEV_LINEAR_Z           = 24,  /* 6; IMU; Accelerometer Cal.;  Std Dev. Linear Acceleration Z;  [g]; */
    DEBUG_GY87_ACC_CALIBRATION_STD_DEV_ANGLE_ROLL         = 25,  /* 6; IMU; Accelerometer Cal.;  Std Dev. Rotation Angle Roll;    [°]; */
    DEBUG_GY87_ACC_CALIBRATION_STD_DEV_ANGLE_PITCH        = 26,  /* 6; IMU; Accelerometer Cal.;  Std Dev. Rotation Angle Pitch;   [°]; */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_ROLL                  = 27,  /* 6; IMU; Gyroscope Meas.;     Rotation Rate Roll;              [°/s]; PLOT [0,0] */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_PITCH                 = 28,  /* 6; IMU; Gyroscope Meas.;     Rotation Rate Pitch;             [°/s]; PLOT [0,1] */
    DEBUG_GY87_GYRO_VALUES_ROT_RATE_YAW                   = 29,  /* 6; IMU; Gyroscope Meas.;     Rotation Rate Yaw;               [°/s]; PLOT [0,2] */
    DEBUG_GY87_ACC_VALUES_LINEAR_X                        = 30,  /* 6; IMU; Accelerometer Meas.; Linear Acceleration X;           [g]; */
    DEBUG_GY87_ACC_VALUES_LINEAR_Y                        = 31,  /* 6; IMU; Accelerometer Meas.; Linear Acceleration Y;           [g]; */
    DEBUG_GY87_ACC_VALUES_LINEAR_Z                        = 32,  /* 6; IMU; Accelerometer Meas.; Linear Acceleration Z;           [g]; */
    DEBUG_GY87_ACC_CORRECTED_VALUES_LINEAR_X              = 33,  /* 6; IMU; Accelerometer Meas.; Corrected Linear Acceleration X; [g];   PLOT [1,0] */
    DEBUG_GY87_ACC_CORRECTED_VALUES_LINEAR_Y              = 34,  /* 6; IMU; Accelerometer Meas.; Corrected Linear Acceleration Y; [g];   PLOT [1,1] */
    DEBUG_GY87_ACC_CORRECTED_VALUES_LINEAR_Z              = 35,  /* 6; IMU; Accelerometer Meas.; Corrected Linear Acceleration Z; [g];   PLOT [1,2] */
    DEBUG_GY87_ACC_VALUES_ANGLE_ROLL                      = 36,  /* 6; IMU; Accelerometer Meas.; Rotation Angle Roll;             [°]; */
    DEBUG_GY87_ACC_VALUES_ANGLE_PITCH                     = 37,  /* 6; IMU; Accelerometer Meas.; Rotation Angle Pitch;            [°]; */
    DEBUG_GY87_ACC_CORRECTED_VALUES_ANGLE_ROLL            = 38,  /* 6; IMU; Accelerometer Meas.; Corrected Rotation Angle Roll;   [°];   PLOT [2,0] */
    DEBUG_GY87_ACC_CORRECTED_VALUES_ANGLE_PITCH           = 39,  /* 6; IMU; Accelerometer Meas.; Corrected Rotation Angle Pitch;  [°];   PLOT [2,1] */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_X                     = 40,  /* 6; IMU; Magnetometer Meas.;  Magnetic Field X;                [G]; */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_Y                     = 41,  /* 6; IMU; Magnetometer Meas.;  Magnetic Field Y;                [G]; */
    DEBUG_GY87_MAG_VALUES_MAG_FIELD_Z                     = 42,  /* 6; IMU; Magnetometer Meas.;  Magnetic Field Z;                [G]; */
    DEBUG_GY87_MAG_HEADING                                = 43,  /* 6; IMU; Magnetometer Meas.;  Magnetic Heading;                [°]; */

    DEBUG_CONTROLSYSTEM_KP_ROLL_ANGLE                     = 44,  /* 4; Control System PID Gains; PID Gains (Angle); kP Roll;  [-]; LIVETUNING [0,0] */
    DEBUG_CONTROLSYSTEM_KI_ROLL_ANGLE                     = 45,  /* 4; Control System PID Gains; PID Gains (Angle); kI Roll;  [-]; LIVETUNING [1,0] */
    DEBUG_CONTROLSYSTEM_KD_ROLL_ANGLE                     = 46,  /* 4; Control System PID Gains; PID Gains (Angle); kD Roll;  [-]; LIVETUNING [2,0] */
    DEBUG_CONTROLSYSTEM_KP_PITCH_ANGLE                    = 47,  /* 4; Control System PID Gains; PID Gains (Angle); kP Pitch; [-]; LIVETUNING [0,1] */
    DEBUG_CONTROLSYSTEM_KI_PITCH_ANGLE                    = 48,  /* 4; Control System PID Gains; PID Gains (Angle); kI Pitch; [-]; LIVETUNING [1,1] */
    DEBUG_CONTROLSYSTEM_KD_PITCH_ANGLE                    = 49,  /* 4; Control System PID Gains; PID Gains (Angle); kD Pitch; [-]; LIVETUNING [2,1] */
    DEBUG_CONTROLSYSTEM_KP_ROLL_RATE                      = 50,  /* 4; Control System PID Gains; PID Gains (Rate);  kP Roll;  [-]; LIVETUNING [0,2] */
    DEBUG_CONTROLSYSTEM_KI_ROLL_RATE                      = 51,  /* 4; Control System PID Gains; PID Gains (Rate);  kI Roll;  [-]; LIVETUNING [1,2] */
    DEBUG_CONTROLSYSTEM_KD_ROLL_RATE                      = 52,  /* 4; Control System PID Gains; PID Gains (Rate);  kD Roll;  [-]; LIVETUNING [2,2] */
    DEBUG_CONTROLSYSTEM_KP_PITCH_RATE                     = 53,  /* 4; Control System PID Gains; PID Gains (Rate);  kP Pitch; [-]; LIVETUNING [0,3] */
    DEBUG_CONTROLSYSTEM_KI_PITCH_RATE                     = 54,  /* 4; Control System PID Gains; PID Gains (Rate);  kI Pitch; [-]; LIVETUNING [1,3] */
    DEBUG_CONTROLSYSTEM_KD_PITCH_RATE                     = 55,  /* 4; Control System PID Gains; PID Gains (Rate);  kD Pitch; [-]; LIVETUNING [2,3] */
    DEBUG_CONTROLSYSTEM_KP_YAW_RATE                       = 56,  /* 4; Control System PID Gains; PID Gains (Rate);  kP Yaw;   [-]; LIVETUNING [0,4] */
    DEBUG_CONTROLSYSTEM_KI_YAW_RATE                       = 57,  /* 4; Control System PID Gains; PID Gains (Rate);  kI Yaw;   [-]; LIVETUNING [1,4] */
    DEBUG_CONTROLSYSTEM_KD_YAW_RATE                       = 58,  /* 4; Control System PID Gains; PID Gains (Rate);  kD Yaw;   [-]; LIVETUNING [2,4] */

    DEBUG_CONTROLSYSTEM_REFERENCE_THROTTLE                = 59,  /* 5; Control System Variables; References (Values);       Throttle; [-]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_VALUE              = 60,  /* 5; Control System Variables; References (Values);       Roll;     [-]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_VALUE             = 61,  /* 5; Control System Variables; References (Values);       Pitch;    [-]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_YAW_VALUE               = 62,  /* 5; Control System Variables; References (Values);       Yaw;      [-]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_ANGLE              = 63,  /* 5; Control System Variables; References (Angles);       Roll;     [°]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_ANGLE             = 64,  /* 5; Control System Variables; References (Angles);       Pitch;    [°]; */
    DEBUG_CONTROLSYSTEM_KALMAN_ROLL_ANGLE                 = 65,  /* 5; Control System Variables; Kalman Angles Prediction;  Roll;     [°]; PLOT [3,0] */
    DEBUG_CONTROLSYSTEM_KALMAN_PITCH_ANGLE                = 66,  /* 5; Control System Variables; Kalman Angles Prediction;  Pitch;    [°]; PLOT [3,1] */
    DEBUG_CONTROLSYSTEM_KALMAN_UNCERTAINTY_ROLL_ANGLE     = 67,  /* 5; Control System Variables; Kalman Angles Uncertainty; Roll;     [°]; */
    DEBUG_CONTROLSYSTEM_KALMAN_UNCERTAINTY_PITCH_ANGLE    = 68,  /* 5; Control System Variables; Kalman Angles Uncertainty; Pitch;    [°]; */
    DEBUG_CONTROLSYSTEM_KALMAN_GAIN_ROLL_ANGLE            = 69,  /* 5; Control System Variables; Kalman Gains;              Roll;     [-]; */
    DEBUG_CONTROLSYSTEM_KALMAN_GAIN_PITCH_ANGLE           = 70,  /* 5; Control System Variables; Kalman Gains;              Pitch;    [-]; */
    DEBUG_CONTROLSYSTEM_ERROR_ROLL_ANGLE                  = 71,  /* 5; Control System Variables; Errors (Angles);           Roll;     [°]; */
    DEBUG_CONTROLSYSTEM_ERROR_PITCH_ANGLE                 = 72,  /* 5; Control System Variables; Errors (Angles);           Pitch;    [°]; */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_ANGLE             = 73,  /* 5; Control System Variables; PID Outputs (Angles);      Roll;     [°]; */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_ANGLE            = 74,  /* 5; Control System Variables; PID Outputs (Angles);      Pitch;    [°]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_ROLL_RATE               = 75,  /* 5; Control System Variables; References (Rates);        Roll;     [°/s]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_PITCH_RATE              = 76,  /* 5; Control System Variables; References (Rates);        Pitch;    [°/s]; */
    DEBUG_CONTROLSYSTEM_REFERENCE_YAW_RATE                = 77,  /* 5; Control System Variables; References (Rates);        Yaw;      [°/s]; */
    DEBUG_CONTROLSYSTEM_ERROR_ROLL_RATE                   = 78,  /* 5; Control System Variables; Errors (Rates);            Roll;     [°/s]; */
    DEBUG_CONTROLSYSTEM_ERROR_PITCH_RATE                  = 79,  /* 5; Control System Variables; Errors (Rates);            Pitch;    [°/s]; */
    DEBUG_CONTROLSYSTEM_ERROR_YAW_RATE                    = 80,  /* 5; Control System Variables; Errors (Rates);            Yaw;      [°/s]; */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_ROLL_RATE              = 81,  /* 5; Control System Variables; PID Outputs (Rates);       Roll;     [°/s]; */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_PITCH_RATE             = 82,  /* 5; Control System Variables; PID Outputs (Rates);       Pitch;    [°/s]; */
    DEBUG_CONTROLSYSTEM_PID_OUTPUT_YAW_RATE               = 83,  /* 5; Control System Variables; PID Outputs (Rates);       Yaw;      [°/s]; */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_1                     = 84,  /* 5; Control System Variables; Motors Speeds;             Motor 1;  [-]; */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_2                     = 85,  /* 5; Control System Variables; Motors Speeds;             Motor 2;  [-]; */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_3                     = 86,  /* 5; Control System Variables; Motors Speeds;             Motor 3;  [-]; */
    DEBUG_CONTROLSYSTEM_MOTOR_SPEED_4                     = 87,  /* 5; Control System Variables; Motors Speeds;             Motor 4;  [-]; */

    DEBUG_CONST_CONTROLSYSTEM_MODE                        = 88,  /* 7; Control System Auxiliar; Mode;          Mode (0: Inputs Reading, 1: Full);  [-]; */
    DEBUG_CONST_CONTROLSYSTEM_LOOP_PERIOD_MS              = 89,  /* 7; Control System Auxiliar; Control Loop;  Period (Configured);                [ms]; */
    DEBUG_CONTROLSYSTEM_LOOP_PERIOD_MEASURED              = 90,  /* 7; Control System Auxiliar; Control Loop;  Period (Measured);                  [ms]; */
    DEBUG_CONTROLSYSTEM_TASK_EXECUTION_TIME               = 91,  /* 7; Control System Auxiliar; Task;          Execution Time;                     [ms]; */
    DEBUG_CONTROLSYSTEM_ESCS_ENABLED                      = 92,  /* 7; Control System Auxiliar; Safety;        ESCs State;                         [-]; */
    DEBUG_CONTROLSYSTEM_ESCS_STARTED_OFF                  = 93,  /* 7; Control System Auxiliar; Safety;        ESCs Started Off;                   [-]; */
    DEBUG_CONTROLSYSTEM_RADIOCONTROLLER_STARTED_CONNECTED = 94,  /* 7; Control System Auxiliar; Safety;        Radio Controller Started Connected; [-]; */
    DEBUG_CONTROLSYSTEM_THROTTLE_STICK_STARTED_DOWN       = 95,  /* 7; Control System Auxiliar; Safety;        Throttle Stick Started Down;        [-]; */
    DEBUG_CONTROLSYSTEM_SAFE_START                        = 96,  /* 7; Control System Auxiliar; Safety;        Safe Start;                         [-]; */
    DEBUG_CONTROLSYSTEM_SAFE_RESTART                      = 97,  /* 7; Control System Auxiliar; Safety;        Safe Restart;                       [-]; */
    DEBUG_CONTROLSYSTEM_STATE_MACHINE_STATE               = 98,  /* 7; Control System Auxiliar; State Machine; State;                              [-]; */

    DEBUG_CONST_ESC_MINIMUM_SPEED                         = 99,  /* 7; ESCs; Speed Limit; Minimum; [-]; */
    DEBUG_CONST_ESC_MAXIMUM_SPEED                         = 100, /* 7; ESCs; Speed Limit; Maximum; [-]; */
    DEBUG_ESC_1                                           = 101, /* 7; ESCs; ESC; Motor 1; [-]; */
    DEBUG_ESC_2                                           = 102, /* 7; ESCs; ESC; Motor 2; [-]; */
    DEBUG_ESC_3                                           = 103, /* 7; ESCs; ESC; Motor 3; [-]; */
    DEBUG_ESC_4                                           = 104, /* 7; ESCs; ESC; Motor 4; [-]; */

    DEBUG_TASK_STACK_WATERMARK_ONOFFBUTTON                = 105, /* 2; System Stack; Stack High Watermark; Task: OnOffButton      ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_STARTUP                    = 106, /* 2; System Stack; Stack High Watermark; Task: Startup          ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_IMU_CALIBRATION            = 107, /* 2; System Stack; Stack High Watermark; Task: IMU_Calibration  ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_CONTROLSYSTEM              = 108, /* 2; System Stack; Stack High Watermark; Task: ControlSystem    ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_USBCOMMUNICATION           = 109, /* 2; System Stack; Stack High Watermark; Task: USB_Communication; [words]; */
    DEBUG_TASK_STACK_WATERMARK_DEBUGGING                  = 110, /* 2; System Stack; Stack High Watermark; Task: Debugging        ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_BATTERYLEVEL               = 111, /* 2; System Stack; Stack High Watermark; Task: BatteryLevel     ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_BATTERYALARM               = 112, /* 2; System Stack; Stack High Watermark; Task: BatteryAlarm     ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_HEARTBEATLIGHT             = 113, /* 2; System Stack; Stack High Watermark; Task: HeartbeatLight   ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_FLIGHTLIGHTS               = 114, /* 2; System Stack; Stack High Watermark; Task: FlightLights     ; [words]; */
    DEBUG_TASK_STACK_WATERMARK_LIVETUNINGSYSTEM           = 115, /* 2; System Stack; Stack High Watermark; Task: LiveTuningSystem ; [words]; */
} DebugSignalsIds_t;

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* DEBUG_SIGNALS_IDS_H */

/* --- End of file ----------------------------------------------------------------------------- */
