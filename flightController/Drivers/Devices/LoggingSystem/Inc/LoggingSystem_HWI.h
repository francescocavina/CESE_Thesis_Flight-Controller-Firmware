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
 * @file:    LoggingSystem_HWI.h
 * @date:    21/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   TODO
 */

#ifndef INC_LOGGING_SYSTEM_HWI_H
#define INC_LOGGING_SYSTEM_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Sends a string through the USB port.
 * @param  string: characters to send.
 * @retval None
 */
void USB_Write(uint8_t * string);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_LOGGING_SYSTEM_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
