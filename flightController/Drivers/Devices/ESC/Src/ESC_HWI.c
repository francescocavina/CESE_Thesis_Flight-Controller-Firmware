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
 * @file:    ESC_HWI.c
 * @date:    24/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a driver for a generic ESC device. It is divided in two parts: One high level
 *           abstraction layer (ESC_UAI.c and ESC_UAI.h) for interface with the user application
 *           and one low level abstraction layer (ESC_HWI.c and ESC_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver to another
 *           platform, please only modify the low layer abstraction layer files where the labels
 *           indicate it.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "ESC_HWI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
bool_t PWM_Init(
    /* BEGIN MODIFY 1 */
    TIM_HandleTypeDef * htim,
    /* END MODIFY 1 */
    uint32_t channel) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    /* BEGIN MODIFY 2 */
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        /* END MODIFY 2 */
        return false;
    }

    /* Initialize timer peripheral */
    /* BEGIN MODIFY 3 */
    if (HAL_OK != HAL_TIM_PWM_Init(htim)) {
        /* END MODIFY 3 */
        return false;
    }

    /* Start PWM signal generation */
    /* BEGIN MODIFY 4 */
    if (HAL_OK != HAL_TIM_PWM_Start(htim, channel)) {
        /* END MODIFY 4 */
        return false;
    }

    return true;
}

bool_t PWM_Deinit(
    /* BEGIN MODIFY 5 */
    TIM_HandleTypeDef * htim,
    /* END MODIFY 5 */
    uint32_t channel) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    /* BEGIN MODIFY 6 */
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        /* END MODIFY 6 */
        return false;
    }

    /* Stop PWM signal generation */
    /* BEGIN MODIFY 7 */
    if (HAL_OK != HAL_TIM_PWM_Stop(htim, channel)) {
        /* END MODIFY 7 */
        return false;
    }

    /* Deinitialize timer peripheral */
    /* BEGIN MODIFY 8 */
    if (HAL_OK != HAL_TIM_PWM_DeInit(htim)) {
        /* END MODIFY 8 */
        return false;
    }

    return true;
}

bool_t PWM_SetDutyCycle(
    /* BEGIN MODIFY 9 */
    TIM_HandleTypeDef * htim,
    /* END MODIFY 9 */
    uint32_t channel, uint16_t dutyCycle) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    /* BEGIN MODIFY 10 */
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        /* END MODIFY 10 */
        return false;
    }
    if (dutyCycle > 16383) {
        return false;
    }

    /* Set duty cycle */
    switch (channel) {

    case TIM_CHANNEL_1:
        /* BEGIN MODIFY 11 */
        TIM3->CCR1 = dutyCycle;
        /* END MODIFY 11 */
        break;

    case TIM_CHANNEL_2:
        /* BEGIN MODIFY 12 */
        TIM3->CCR2 = dutyCycle;
        /* END MODIFY 12 */
        break;

    case TIM_CHANNEL_3:
        /* BEGIN MODIFY 13 */
        TIM3->CCR3 = dutyCycle;
        /* END MODIFY 13 */
        break;

    case TIM_CHANNEL_4:
        /* BEGIN MODIFY 14 */
        TIM3->CCR4 = dutyCycle;
        /* END MODIFY 14 */
        break;

    default:
        return false;
        break;
    }

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */

/* --- End of file ----------------------------------------------------------------------------- */
