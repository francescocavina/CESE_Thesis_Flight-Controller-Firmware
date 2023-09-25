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
 * @file:    FS-A8S_driver_UAI.h
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
 */

#ifndef INC_FS_A8S_DRIVER_UAI_H
#define INC_FS_A8S_DRIVER_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "FS-A8S_driver_HWI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */

/*
 * @brief  Enumeration for channel number.
 */
typedef enum {
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_6 = 6,
    CHANNEL_7 = 7,
    CHANNEL_8 = 8,
    CHANNEL_9 = 9,
    CHANNEL_10 = 10,
    CHANNEL_11 = 11,
    CHANNEL_12 = 12,
    CHANNEL_13 = 13,
    CHANNEL_14 = 14,
} FSA8S_RC_CHANNEL_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Initializes the radio control receiver driver. The device can be initialized
 *         only once.
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the UART communication.
 * @retval hibus: Pointer to a IBUS_HandleTypeDef structure that contains
 *                the configuration information for the iBus communication
 *                if the initialization was successful.
 *         NULL:  If initialization was not successful. This may happen because
 *                of an initialization error or because the driver had already
 *                been initialized before.
 */
IBUS_HandleTypeDef_t * FSA8S_RC_Init(UART_HandleTypeDef * huart);

/**
 * @brief  Reads a radio control receiver channel (14 available).
 * @param  hibus:   Pointer to a IBUS_HandleTypeDef structure that contains
 *                  the configuration information for the iBus communication.
 *         channel: Channel number to be read (CHANNEL_1 to CHANNEL_14).
 * @retval value:   Channel value from 0 to a defined maximum number.
 *         0:       If hibus param is NULL or channel number is not between
 *                  CHANNEL_1 to CHANNEL_14.
 */
uint16_t FSA8S_RC_ReadChannel(IBUS_HandleTypeDef_t * hibus, FSA8S_RC_CHANNEL_t channel);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_FS_A8S_DRIVER_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
