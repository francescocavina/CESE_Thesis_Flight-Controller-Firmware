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
 * @file:    FS-A8S_driver_HWI.C
 * @date:    16/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.5.0
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

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "FS-A8S_driver_HWI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define FSA8S_RC_UART_INSTANCE (USART2)
#define UART_BAUD_RATE         (115200)

/* --- Private data type declarations ---------------------------------------------------------- */
static UART_HandleTypeDef huart;

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  UART Initialization Function
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval true:  If UART could be initialized.
 *         false: If UART couldn't be initialized.
 */
static bool_t MX_UART_Init(UART_HandleTypeDef * huart);

/**
 * @brief  Enable DMA controller clock
 * @param  None
 * @retval None
 */
static void MX_DMA_Init(void);

/* --- Public variable definitions ------------------------------------------------------------- */
DMA_HandleTypeDef hdma_usart2_rx;

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t MX_UART_Init(UART_HandleTypeDef * huart) {
    /* BEGIN MODIFY 1*/
    huart->Instance = FSA8S_RC_UART_INSTANCE;
    huart->Init.BaudRate = UART_BAUD_RATE;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    /* END MODIFY 1 */

    /* Initialize UART peripheral with function located on the "stm32f4xx_hal_uart.c"
     * file, taking the previously defined UART handle as argument.
     */

    /* BEGIN MODIFY 2 */
    if (HAL_OK != HAL_UART_Init(huart)) {
        /* END MODIFY 2 */
        return false;
    }

    return true;
}

static void MX_DMA_Init(void) {

    /* The DMA initialization happens on the background when HAL_UART_MspInit() is called.
     * This function is located on the "stm32f4xx_hal_msp.c" file and calls the function
     * HAL_DMA_Init() on the "stm32f4xx_hal_dma.c" file which sets all the configuration.
     */

    /* DMA controller clock enable */
    /* BEGIN MODIFY 3 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* END MODIFY 3 */
}

/* --- Public function implementation ---------------------------------------------------------- */
bool_t iBus_Init(iBus_HandleTypeDef_t * hibus) {

    hibus->huart = &huart;

    MX_DMA_Init();

    /* Initialize UART */
    if (!MX_UART_Init(hibus->huart)) {
        /* UART initialization was unsuccessful */
        return false;
    }

    /* Initialize DMA reception */
    /* BEGIN MODIFY 4 */
    if (HAL_OK != HAL_UART_Receive_DMA(hibus->huart, hibus->buffer, hibus->bufferSize)) {
        /* END MODIFY 4 */

        /* DMA initialization was unsuccessful */
        return false;
    }

    /* iBus initialization was successful */
    return true;
}

/* --- End of file ----------------------------------------------------------------------------- */
