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
 * @file:    LiveTuningSystem_UAI.c
 * @date:    05/04/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a driver for live-tuning the flight controller control system via USB.
 *           It is divided in two parts: One high level abstraction layer
 *           (LiveTuningSystem_UAI.c and LiveTuningSystem_UAI.h) for interface with the
 *           user application and one low level abstraction layer
 *           (LiveTuningSystem_HWI.c and LiveTuningSystem_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver
 *           to another platform, please only modify the low layer abstraction
 *           layer files where the labels indicate it.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "LiveTuningSystem_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
bool_t LiveTune_CS_PID_Gains(ControlSystem_PID_Gains_t *PID_Gains) {

    uint8_t  buffer[160];
    uint32_t buffer_length = sizeof(buffer);

    if (USB_Read(buffer, &buffer_length)) {
        /* Ensure buffer is null-terminated */
        if (buffer_length < sizeof(buffer)) {
            buffer[buffer_length] = '\0';
        } else {
            buffer[sizeof(buffer) - 1] = '\0';
        }

        /* Pointers to decode input string */
        char *token;
        char *saveptr1;
        char *saveptr2;

        /* Convert buffer to char* */
        char *str = (char *)buffer;

        /* Split string by slashes */
        token     = strtok_r(str, "/", &saveptr1);
        while (token != NULL) {
            /* Split string by underscores */
            char *id_str    = strtok_r(token, "_", &saveptr2);
            char *value_str = strtok_r(NULL, "_", &saveptr2);

            if (id_str && value_str) {
                /* Convert extracted id and value */
                int   signal_id    = atoi(id_str);
                float signal_value = atof(value_str);

                /* Update variables */
                switch (signal_id) {
                case 1:
                    PID_Gains->kP_rollAngle = signal_value;
                    break;
                case 2:
                    PID_Gains->kI_rollAngle = signal_value;
                    break;
                case 3:
                    PID_Gains->kD_rollAngle = signal_value;
                    break;
                case 4:
                    PID_Gains->kP_pitchAngle = signal_value;
                    break;
                case 5:
                    PID_Gains->kI_pitchAngle = signal_value;
                    break;
                case 6:
                    PID_Gains->kD_pitchAngle = signal_value;
                    break;
                case 7:
                    PID_Gains->kP_rollRate = signal_value;
                    break;
                case 8:
                    PID_Gains->kI_rollRate = signal_value;
                    break;
                case 9:
                    PID_Gains->kD_rollRate = signal_value;
                    break;
                case 10:
                    PID_Gains->kP_pitchRate = signal_value;
                    break;
                case 11:
                    PID_Gains->kI_pitchRate = signal_value;
                    break;
                case 12:
                    PID_Gains->kD_pitchRate = signal_value;
                    break;
                case 13:
                    PID_Gains->kP_yawRate = signal_value;
                    break;
                case 14:
                    PID_Gains->kI_yawRate = signal_value;
                    break;
                case 15:
                    PID_Gains->kD_yawRate = signal_value;
                    break;
                default:
                    break;
                }
            }

            /* Get next token */
            token = strtok_r(NULL, "/", &saveptr1);
        }

        return true;
    } else {
        return false;
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
