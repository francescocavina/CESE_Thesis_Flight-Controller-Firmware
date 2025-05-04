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
 * @file:    LiveTuningSystem_HWI.c
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
#include "LiveTuningSystem_HWI.h"
#include "FreeRTOS.h"
#include "task.h"

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
bool USB_Read(uint8_t *buffer, uint32_t *buffer_length) {

    /* Check if USB is ready and buffer is not NULL */
    if (!usb_rx_ready || buffer == NULL || buffer_length == NULL) {
        return false;
    }

    uint32_t copy_length = 0;

    /* BEGIN MODIFY 1 */
    taskENTER_CRITICAL();
    /* END MODIFY 1*/
    {
        /* Copy data up to the end of message character */
        for (uint32_t i = 0; i < usb_rx_length; i++) {
            if (usb_rx_buffer[i] == '\n') {
                copy_length = i + 1;
                break;
            }
        }

        /* Copy data to the output buffer */
        if (copy_length > 0) {
            memcpy(buffer, usb_rx_buffer, copy_length);
            *buffer_length = copy_length;

            /* Shift remaining data in the buffer that has arrived after the end of message character */
            memmove(usb_rx_buffer, &usb_rx_buffer[copy_length], usb_rx_length - copy_length);
            usb_rx_length -= copy_length;

            /* Set buffer state as not ready */
            usb_rx_ready = (usb_rx_length > 0);
        } else {
            *buffer_length = 0;
        }
    }
    /* BEGIN MODIFY 2 */
    taskEXIT_CRITICAL();
    /* END MODIFY 2 */

    return (copy_length > 0);
}

/* --- Public function implementation ---------------------------------------------------------- */

/* --- End of file ----------------------------------------------------------------------------- */
