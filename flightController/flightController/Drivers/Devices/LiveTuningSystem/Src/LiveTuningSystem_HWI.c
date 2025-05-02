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
 * @date:    05/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   TODO.
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

    if (!usb_rx_ready || buffer == NULL || buffer_length == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    // Copy data to user buffer
    uint32_t copy_length = (*buffer_length < usb_rx_length) ? *buffer_length : usb_rx_length;
    memcpy(buffer, usb_rx_buffer, copy_length);
    *buffer_length = copy_length;

    // Mark buffer as read
    usb_rx_ready   = false;
    usb_rx_length  = 0;
    taskEXIT_CRITICAL();
    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */

/* --- End of file ----------------------------------------------------------------------------- */
