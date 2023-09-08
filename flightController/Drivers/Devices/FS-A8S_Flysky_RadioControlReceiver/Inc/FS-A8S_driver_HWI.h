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
 * @file:    FS-A8S_driver_HWI.h
 * @date:    08/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.3.0
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
 * @details: This driver uses UART for the communication with the radio control.
 *           The configuration of the UART peripheral MUST be BAUDRATE = 115200,
 *           WORDLENGTH = 8, STOPBITS = 1, PARITY = NONE and MODE = RX (optional)
 *           to be able to communicate with the radio control receiver.
 *           Moreover, this driver uses DMA and it is optional. But it is worth
 *           mentioning that the type of driver (polling, interrupt or DMA) will
 *           have an effect on the final application.
 *           As mentioned before, the configuration here set uses DMA, in circular
 *           mode with data width of a byte and it is associated to the UART RX.
 */

#ifndef INC_FS_A8S_DRIVER_HWI_H
#define INC_FS_A8S_DRIVER_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "dev_settings.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
/**
 * @brief iBus handle structure definition
 */
typedef struct {
    UART_HandleTypeDef * huart; /* Pointer to a iBus_HandleTypeDef structure */
    uint8_t * buffer;           /* Buffer in which UART DMA will put the received data */
    uint8_t bufferSize;         /* Buffer size */
    uint16_t * data;            /* Channels data */
    uint8_t channels;           /* Number of channels */
} iBus_HandleTypeDef_t;

/**
 * @brief boolt_t type definition
 */
typedef bool bool_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Initializes the iBus communication with the RC receiver.
 * @param  hibus: Pointer to a iBus_HandleTypeDef structure that contains
 *                the configuration information for the iBus communication.
 * @retval true:  If communication could be initialized
 *         false: If communication couldn't be initialized
 */
bool_t iBus_Init(iBus_HandleTypeDef_t * hibus);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_FS_A8S_DRIVER_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
