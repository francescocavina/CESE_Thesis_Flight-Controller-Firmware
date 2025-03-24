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
 * @file:    FSA8S_driver_HWI.C
 * @date:    03/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:   This is a driver for the radio control receiver FlySky FS-A8S.
 *           It is divided in two parts: One high level abstraction layer
 *           (FSA8S_driver_UAI.c and FSA8S_driver_UAI.h) for interface with the
 *           user application and one low level abstraction layer
 *           (FSA8S_driver_HWI.c and FSA8S_driver_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver
 *           to another platform, please only modify the low layer abstraction
 *           layer files where the labels indicate it.
 *
 * @details: This driver uses UART for the communication with the radio control.
 *           The configuration of the UART peripheral MUST be BAUDRATE = 115200,
 *           WORDLENGTH = 8, STOPBITS = 1, PARITY = NONE and MODE = RX (TX is
 *           optional) to be able to communicate with the radio control receiver.
 *           Moreover, this driver uses DMA and it is optional. But it is worth
 *           mentioning that the type of driver (polling, interrupt or DMA) will
 *           have an effect on the final performance.
 *           As mentioned before, the configuration here set uses DMA, in circular
 *           mode with data width of a byte and it is associated to the UART RX.
 *           This driver was developed and intended to be used with a
 *           STM32F401CCU6 (STM Cortex M4) microcontroller, but it could be easily
 *           ported to another platform by changing the code between the labels
 *           that indicate "MODIFY".
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "FSA8S_driver_HWI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
bool_t IBUS_Init(IBUS_HandleTypeDef_t *hibus) {

    /* Check parameter */
    /* BEGIN MODIFY 1 */
    if (NULL == hibus || NULL == hibus->buffer || 0 == hibus->bufferSize) {
        /* END MODIFY 1 */
        return false;
    }

    /* Check if UART is initialized */
    /* BEGIN MODIFY 2 */
    if (NULL == hibus->huart || hibus->huart->gState == HAL_UART_STATE_RESET) {
        /* END MODIFY 2 */
        return false;
    }

    /* Initialize DMA reception */
    /* BEGIN MODIFY 3 */
    if (HAL_OK != HAL_UART_Receive_DMA(hibus->huart, hibus->buffer, hibus->bufferSize)) {
        /* END MODIFY 3 */

        /* DMA initialization was unsuccessful */
        return false;
    }

    /* iBus initialization was successful */
    return true;
}

bool_t IBUS_CheckAndResetDMA(IBUS_HandleTypeDef_t *hibus) {
    if (hibus == NULL || hibus->huart == NULL) {
        return false;
    }

    /* Check if DMA is in error state */
    if (hibus->huart->RxState == HAL_UART_STATE_ERROR || hibus->huart->hdmarx->State == HAL_DMA_STATE_ERROR || hibus->huart->hdmarx->State == HAL_DMA_STATE_TIMEOUT) {

        /* Abort current DMA */
        HAL_UART_DMAStop(hibus->huart);

        /* Reset error flags */
        hibus->huart->ErrorCode         = HAL_UART_ERROR_NONE;
        hibus->huart->hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Restart DMA reception */
        HAL_UART_Receive_DMA(hibus->huart, hibus->buffer, hibus->bufferSize);

        return true;
    }

    return false;
}

/* --- End of file ----------------------------------------------------------------------------- */
