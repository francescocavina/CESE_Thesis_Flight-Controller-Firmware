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
 * @date:    20/08/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for header files.
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
 * @brief
 * @param
 * @retval
 */
iBus_HandleTypeDef_t * FSA8S_RC_Init(UART_HandleTypeDef * huart, uint8_t * buffer);

/**
 * @brief
 * @param
 * @retval
 */
uint16_t FSA8S_RC_ReadChannel(iBus_HandleTypeDef_t * hibus, FSA8S_RC_CHANNEL_t channel);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_FS_A8S_DRIVER_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
