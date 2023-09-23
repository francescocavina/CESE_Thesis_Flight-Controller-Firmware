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
 * @file:    main_app.c
 * @date:    23/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is the main application.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "main_app.h"
#include "usbd_cdc_if.h"
// #include "MPU-6050_driver_UAI.h"
// #include "MPU-6050_driver_HWI.h"
#include "FS-A8S_driver_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
void flightController_App(void) {
    static IBUS_HandleTypeDef_t * rc_controller;
    uint8_t channel = CHANNEL_1;
    static uint16_t channelValue;
    uint8_t str[20];

    rc_controller = FSA8S_RC_Init(&huart2);

    while (1) {
        channelValue = FSA8S_RC_ReadChannel(rc_controller, channel);
        sprintf((char *)str, (const char *)"Channel %d: %d\r\n", channel, channelValue);
        CDC_Transmit_FS(str, strlen((const char *)str));
    }

    //        MPU6050_HandleTypeDef_t * hmpu6050 = MPU6050_IMU_Init();
    //        // str[12] = hmpu6050->instance + 48;
    //        HAL_Delay(1000);
    //
    //        if (hmpu6050->instance == 1 || hmpu6050->instance == 2) {
    //            CDC_Transmit_FS(str, 17);
    //        } else {
    //            CDC_Transmit_FS((char *)"Not initialized\r\n", 18);
    //        }
}

/* --- End of file ----------------------------------------------------------------------------- */
