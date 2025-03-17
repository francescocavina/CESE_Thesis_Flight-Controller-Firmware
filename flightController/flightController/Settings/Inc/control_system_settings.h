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
 * @file:    controlSystem_settings.h
 * @date:    03/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This file contains the control system settings.
 */

#ifndef CONTROLSYSTEM_SETTINGS_H
#define CONTROLSYSTEM_SETTINGS_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
/* Drivers Settings */
#define FSA8S_CHANNELS (10)
/* Control System Settings */
#define CONTROLSYSTEM_MODE                   (0)
#define CONTROLSYSTEM_LOOP_PERIOD_S          (0.004) // Loop period in [s]
#define CONTROLSYSTEM_LOOP_PERIOD_MS         (4)     // Loop period in [ms]
#define CONTROLSYSTEM_MINIMUM_INPUT_THROTTLE (25)
#define CONTROLSYSTEM_MAXIMUM_INPUT_THROTTLE (1800)
#define CONTROLSYSTEM_PID_OUTPUT_LIMIT       (200)
#define CONTROLSYSTEM_PID_ITERM_LIMIT        (200)
#define ESC_MAXIMUM_SPEED                    (90)
#define ESC_MINIMUM_SPEED                    (10)
/* PID gains: Angles */
#define CONTROLSYSTEM_KP_ROLL_ANGLE  (2.0)
#define CONTROLSYSTEM_KP_PITCH_ANGLE (2.0)
#define CONTROLSYSTEM_KI_ROLL_ANGLE  (0.0)
#define CONTROLSYSTEM_KI_PITCH_ANGLE (0.0)
#define CONTROLSYSTEM_KD_ROLL_ANGLE  (0.0)
#define CONTROLSYSTEM_KD_PITCH_ANGLE (0.0)
/* PID gains: Rates */
#define CONTROLSYSTEM_KP_ROLL_RATE  (1.404)
#define CONTROLSYSTEM_KP_PITCH_RATE (0.0)
#define CONTROLSYSTEM_KP_YAW_RATE   (0.0)
#define CONTROLSYSTEM_KI_ROLL_RATE  (0.596)
#define CONTROLSYSTEM_KI_PITCH_RATE (0.0)
#define CONTROLSYSTEM_KI_YAW_RATE   (0.0)
#define CONTROLSYSTEM_KD_ROLL_RATE  (0.0)
#define CONTROLSYSTEM_KD_PITCH_RATE (0.0)
#define CONTROLSYSTEM_KD_YAW_RATE   (0.0)

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* CONTROLSYSTEM_SETTINGS_H */

/* --- End of file ----------------------------------------------------------------------------- */
