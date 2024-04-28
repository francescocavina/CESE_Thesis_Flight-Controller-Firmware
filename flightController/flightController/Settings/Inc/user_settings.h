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
 * @file:    user_settings.h
 * @date:    26/08/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for header files.
 */

#ifndef USER_SETTINGS_H
#define USER_SETTINGS_H

/* --- Headers files inclusions ---------------------------------------------------------------- */

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* ############################################################################################# */
/* ####################################### USER SETTINGS ####################################### */
/* ############################################################################################# */

/* --- POWER ON/OFF DRIVER --------------------------------------------------------------------- */
#define PW_ON_OFF_DRIVER_TIME                                                                                                                                                                                                                            \
    (3000) // Time in [ms] that the on-board power on/off button needs
           // to be pressed in order to turn on/off the flight
           // controller.

/* --- BATTERY ALARM --------------------------------------------------------------------------- */
#define BATTERY_ALARM_THRESHOLD                                                                                                                                                                                                                          \
    (9.5) // Voltage level in [V] at which the battery alarm starts
          // beeping, indicating that the flight controller has low
          // battery.

/* --- ON-BOARD HEARTEAT LIGHT ----------------------------------------------------------------- */
#define HEARTBEAT_PERIOD (500) // Heartbeat period in [ms] (duty cycle is 50%)

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* USER_SETTINGS_H */

/* --- End of file ----------------------------------------------------------------------------- */
