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
 * @file:    FS-A8S_driver_calibration.h
 * @date:    23/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.6.0
 *
 * @brief:   This is a driver for the radio control receiver FlySky FS-A8S.
 *           It is divided in two parts: One high level abstraction layer
 *           (FS-A8S_driver_UAI.c and FS-A8S_driver_UAI.h) for interface with the
 *           user application and one low level abstraction layer
 *           (FS-A8S_driver_HWI.c and FS-A8S_driver_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver
 *           to another platform, please only modify the low layer abstraction
 *           layer files where the labels indicate it.
 *
 * @details: The radio control device must be calibrated in order to have the right
 *           minimum and maximum values. As the left with horizontal movement
 *           joystick is the one that can permanently stay in a position without
 *           returning to the center, it might have some deviation. Therefore, it
 *           might be necessary to calibrate it.
 */

#ifndef INC_FS_A8S_DRIVER_CALIBRATION_H_
#define INC_FS_A8S_DRIVER_CALIBRATION_H_

/* --- Public macros definitions --------------------------------------------------------------- */
#define CHANNEL_01_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_02_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_03_CALIBRATION_VALUE (9) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_04_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_05_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_06_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_07_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_08_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_09_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_10_CALIBRATION_VALUE (0) // Calibrated with IBUS_CHANNEL_MAX_VALUE = 1000
#define CHANNEL_11_CALIBRATION_VALUE                                                               \
    (0) // FlySkyfly FS-i6X model only supports 10 channel using iBus
#define CHANNEL_12_CALIBRATION_VALUE                                                               \
    (0) // FlySkyfly FS-i6X model only supports 10 channel using iBus
#define CHANNEL_13_CALIBRATION_VALUE                                                               \
    (0) // FlySkyfly FS-i6X model only supports 10 channel using iBus
#define CHANNEL_14_CALIBRATION_VALUE                                                               \
    (0) // FlySkyfly FS-i6X model only supports 10 channel using iBus

#endif /* INC_FS_A8S_DRIVER_CALIBRATION_H_ */

/* --- End of file ----------------------------------------------------------------------------- */
