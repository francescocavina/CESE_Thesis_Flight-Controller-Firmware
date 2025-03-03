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
    GY87_TEMPERATURE = 1,                /* 1; General; Flight Controller; Temperature;   [°C] */
    FLIGHT_CONTROLLER_BATTERY_LEVEL = 2, /* 1; General; Flight Controller; Battery Level; [V] */

    FSA8S_CHANNEL_VALUES_1 = 3,   /* 2; Radio Control; Movement;      Roll            (CH01); [-] */
    FSA8S_CHANNEL_VALUES_2 = 4,   /* 2; Radio Control; Movement;      Pitch           (CH02); [-] */
    FSA8S_CHANNEL_VALUES_3 = 5,   /* 2; Radio Control; Movement;      Throttle        (CH03); [-] */
    FSA8S_CHANNEL_VALUES_4 = 6,   /* 2; Radio Control; Movement;      Yaw             (CH04); [-] */
    FSA8S_CHANNEL_VALUES_5 = 7,   /* 2; Radio Control; TBD;           not_defined     (CH05); [-] */
    FSA8S_CHANNEL_VALUES_6 = 8,   /* 2; Radio Control; Movement;      ESCs On/Off     (CH06); [-] */
    FSA8S_CHANNEL_VALUES_7 = 9,   /* 2; Radio Control; TBD;           not_defined     (CH07); [-] */
    FSA8S_CHANNEL_VALUES_8 = 10,  /* 2; Radio Control; Flight Lights; Speed           (CH08); [-] */
    FSA8S_CHANNEL_VALUES_9 = 11,  /* 2; Radio Control; Flight Lights; Type            (CH09); [-] */
    FSA8S_CHANNEL_VALUES_10 = 12, /* 2; Radio Control; Flight Lights; On/Off          (CH10); [-] */

    GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_ROLL = 13,  /* 3; IMU; Gyroscope;     Rotation Rate Roll  Calibration;   [°/s] */
    GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_PITCH = 14, /* 3; IMU; Gyroscope;     Rotation Rate Pitch Calibration;   [°/s] */
    GY87_GYRO_CALIBRATION_VALUES_ROT_RATE_YAW = 15,   /* 3; IMU; Gyroscope;     Rotation Rate Yaw   Calibration;   [°/s] */
    GY87_ACC_CALIBRATION_VALUES_LINEAR_X = 16,        /* 3; IMU; Accelerometer; Linear Acceleration X Calibration; [g] */
    GY87_ACC_CALIBRATION_VALUES_LINEAR_Y = 17,        /* 3; IMU; Accelerometer; Linear Acceleration Y Calibration; [g] */
    GY87_ACC_CALIBRATION_VALUES_LINEAR_Z = 18,        /* 3; IMU; Accelerometer; Linear Acceleration Z Calibration; [g] */
    GY87_GYRO_VALUES_ROT_RATE_ROLL = 19,              /* 3; IMU; Gyroscope;     Rotation Rate Roll;                [°/s] */
    GY87_GYRO_VALUES_ROT_RATE_PITCH = 20,             /* 3; IMU; Gyroscope;     Rotation Rate Pitch;               [°/s] */
    GY87_GYRO_VALUES_ROT_RATE_YAW = 21,               /* 3; IMU; Gyroscope;     Rotation Rate Yaw;                 [°/s] */
    GY87_ACC_VALUES_LINEAR_X = 22,                    /* 3; IMU; Accelerometer; Linear Acceleration X;             [g] */
    GY87_ACC_VALUES_LINEAR_Y = 23,                    /* 3; IMU; Accelerometer; Linear Acceleration Y;             [g] */
    GY87_ACC_VALUES_LINEAR_Z = 24,                    /* 3; IMU; Accelerometer; Linear Acceleration Z;             [g] */
    GY87_ACC_VALUES_ANGLE_ROLL = 25,                  /* 3; IMU; Accelerometer; Rotation Angle Roll;               [°] */
    GY87_ACC_VALUES_ANGLE_PITCH = 26,                 /* 3; IMU; Accelerometer; Rotation Angle Pitch;              [°] */
    GY87_MAG_VALUES_MAG_FIELD_X = 27,                 /* 3; IMU; Magnetometer;  Magnetic Field X;                  [G] */
    GY87_MAG_VALUES_MAG_FIELD_Y = 28,                 /* 3; IMU; Magnetometer;  Magnetic Field Y;                  [G] */
    GY87_MAG_VALUES_MAG_FIELD_Z = 29,                 /* 3; IMU; Magnetometer;  Magnetic Field Z;                  [G] */
    GY87_MAG_HEADING = 30,                            /* 3; IMU; Magnetometer;  Magnetic Heading;                  [°] */

    CONST_KP_ROLL_ANGLE = 31,  /* 4; Control System Constants; PID Gains; kP Roll  Angle; [-] */
    CONST_KI_ROLL_ANGLE = 32,  /* 4; Control System Constants; PID Gains; kI Roll  Angle; [-] */
    CONST_KD_ROLL_ANGLE = 33,  /* 4; Control System Constants; PID Gains; kD Roll  Angle; [-] */
    CONST_KP_PITCH_ANGLE = 34, /* 4; Control System Constants; PID Gains; kP Pitch Angle; [-] */
    CONST_KI_PITCH_ANGLE = 35, /* 4; Control System Constants; PID Gains; kI Pitch Angle; [-] */
    CONST_KD_PITCH_ANGLE = 36, /* 4; Control System Constants; PID Gains; kD Pitch Angle; [-] */
    CONST_KP_ROLL_RATE = 37,   /* 4; Control System Constants; PID Gains; kP Roll  Rate; [-] */
    CONST_KI_ROLL_RATE = 38,   /* 4; Control System Constants; PID Gains; kI Roll  Rate; [-] */
    CONST_KD_ROLL_RATE = 39,   /* 4; Control System Constants; PID Gains; kD Roll  Rate; [-] */
    CONST_KP_PITCH_RATE = 40,  /* 4; Control System Constants; PID Gains; kP Pitch Rate; [-] */
    CONST_KI_PITCH_RATE = 41,  /* 4; Control System Constants; PID Gains; kI Pitch Rate; [-] */
    CONST_KD_PITCH_RATE = 42,  /* 4; Control System Constants; PID Gains; kD Pitch Rate; [-] */
    CONST_KP_YAW_RATE = 43,    /* 4; Control System Constants; PID Gains; kP Yaw   Rate; [-] */
    CONST_KI_YAW_RATE = 44,    /* 4; Control System Constants; PID Gains; kI Yaw   Rate; [-] */
    CONST_KD_YAW_RATE = 45,    /* 4; Control System Constants; PID Gains; kD Yaw   Rate; [-] */

    CONTROLSYSTEM_REFERENCE_THROTTLE = 46,     /* 5; Control System Variables; Reference;  Throttle;     [-] */
    CONTROLSYSTEM_REFERENCE_ROLL_ANGLE = 47,   /* 5; Control System Variables; Reference;  Roll  Angle;  [°] */
    CONTROLSYSTEM_REFERENCE_PITCH_ANGLE = 48,  /* 5; Control System Variables; Reference;  Pitch Angle;  [°] */
    CONTROLSYSTEM_KALMAN_ROLL_ANGLE = 49,      /* 5; Control System Variables; Kalman;     Roll  Angle;  [°] */
    CONTROLSYSTEM_KALMAN_PITCH_ANGLE = 50,     /* 5; Control System Variables; Kalman;     Pitch Angle;  [°] */
    CONTROLSYSTEM_ERROR_ROLL_ANGLE = 51,       /* 5; Control System Variables; Error;      Roll  Angle;  [°] */
    CONTROLSYSTEM_ERROR_PITCH_ANGLE = 52,      /* 5; Control System Variables; Error;      Pitch Angle;  [°] */
    CONTROLSYSTEM_PID_OUTPUT_ROLL_ANGLE = 53,  /* 5; Control System Variables; PID Output; Roll Angle;   [°] */
    CONTROLSYSTEM_PID_OUTPUT_PITCH_ANGLE = 54, /* 5; Control System Variables; PID Output; Pitch Angle;  [°] */
    CONTROLSYSTEM_REFERENCE_ROLL_RATE = 55,    /* 5; Control System Variables; Reference;  Roll Rate;    [°/s] */
    CONTROLSYSTEM_REFERENCE_PITCH_RATE = 56,   /* 5; Control System Variables; Reference;  Pitch Rate;   [°/s] */
    CONTROLSYSTEM_REFERENCE_YAW_RATE = 57,     /* 5; Control System Variables; Reference;  Yaw Rate;     [°/s] */
    CONTROLSYSTEM_ERROR_ROLL_RATE = 58,        /* 5; Control System Variables; Error;      Roll Rate;    [°/s] */
    CONTROLSYSTEM_ERROR_PITCH_RATE = 59,       /* 5; Control System Variables; Error;      Pitch Rate;   [°/s] */
    CONTROLSYSTEM_ERROR_YAW_RATE = 60,         /* 5; Control System Variables; Error;      Yaw Rate;     [°/s] */
    CONTROLSYSTEM_PID_OUTPUT_ROLL_RATE = 61,   /* 5; Control System Variables; PID Output; Roll Rate;    [°/s] */
    CONTROLSYSTEM_PID_OUTPUT_PITCH_RATE = 62,  /* 5; Control System Variables; PID Output; Pitch Rate;   [°/s] */
    CONTROLSYSTEM_PID_OUTPUT_YAW_RATE = 63,    /* 5; Control System Variables; PID Output; Yaw Rate;     [°/s] */

    ESC_1 = 64, /* 6; ESCs; ESC; Motor 1; [-] */
    ESC_2 = 65, /* 6; ESCs; ESC; Motor 2; [-] */
    ESC_3 = 66, /* 6; ESCs; ESC; Motor 3; [-] */
    ESC_4 = 67, /* 6; ESCs; ESC; Motor 4; [-] */

    TASK_STACK_WATERMARK_ONOFFBUTTON = 68,      /* 7; System Stack; Stack High Watermark; Task: OnOffButton      ; [words] */
    TASK_STACK_WATERMARK_CONTROLSYSTEM = 69,    /* 7; System Stack; Stack High Watermark; Task: ControlSystem    ; [words] */
    TASK_STACK_WATERMARK_USBCOMMUNICATION = 70, /* 7; System Stack; Stack High Watermark; Task: USB_Communication; [words] */
    TASK_STACK_WATERMARK_DEBUGGING = 71,        /* 7; System Stack; Stack High Watermark; Task: Debugging        ; [words] */
    TASK_STACK_WATERMARK_BATTERYLEVEL = 72,     /* 7; System Stack; Stack High Watermark; Task: BatteryLevel     ; [words] */
    TASK_STACK_WATERMARK_BATTERYALARM = 73,     /* 7; System Stack; Stack High Watermark; Task: BatteryAlarm     ; [words] */
    TASK_STACK_WATERMARK_HEARTBEATLIGHT = 74,   /* 7; System Stack; Stack High Watermark; Task: HeartbeatLight   ; [words] */
    TASK_STACK_WATERMARK_FLIGHTLIGHTS = 75,     /* 7; System Stack; Stack High Watermark; Task: FlightLights     ; [words] */
} DebugSignalsIds_t;

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* DEBUG_SIGNALS_IDS_H */

/* --- End of file ----------------------------------------------------------------------------- */
