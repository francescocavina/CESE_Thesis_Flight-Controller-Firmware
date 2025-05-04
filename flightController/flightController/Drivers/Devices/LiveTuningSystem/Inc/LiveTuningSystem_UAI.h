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
 * @file:    LiveTuningSystem_UAI.h
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

#ifndef INC_LIVE_TUNING_SYSTEM_UAI_H
#define INC_LIVE_TUNING_SYSTEM_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "LiveTuningSystem_HWI.h"
#include "control_system_support.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Live-tunes the PID gains of the flight controller control system.
 * @param  PID_Gains: Pointer to the structure containing the PID gains to be tuned.
 * @retval true if the PID gains were updated, false otherwise.
 */
bool_t LiveTune_CS_PID_Gains(ControlSystem_PID_Gains_t *PID_Gains);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_LIVE_TUNING_SYSTEM_UAI_H */

       /* --- End of file ----------------------------------------------------------------------------- */
