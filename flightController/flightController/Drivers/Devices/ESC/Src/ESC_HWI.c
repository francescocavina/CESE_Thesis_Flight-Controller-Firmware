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
 * @date:    27/02/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
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

/* --- Public function implementation ---------------------------------------------------------- */
bool_t PWM_Init(ESC_HandleTypeDef_t * hesc) {

    /* Check first parameter */
    if (NULL == hesc) {
        return false;
    }

    /* Start PWM signal generation */
    /* BEGIN MODIFY 1 */
    if (HAL_OK != HAL_TIM_PWM_Start(hesc->htim, TIM_CHANNEL_1)) {
        /* END MODIFY 1 */
        return false;
    }
    /* BEGIN MODIFY 2 */
    if (HAL_OK != HAL_TIM_PWM_Start(hesc->htim, TIM_CHANNEL_2)) {
        /* END MODIFY 2 */
        return false;
    }
    /* BEGIN MODIFY 3 */
    if (HAL_OK != HAL_TIM_PWM_Start(hesc->htim, TIM_CHANNEL_3)) {
        /* END MODIFY 3 */
        return false;
    }
    /* BEGIN MODIFY 4 */
    if (HAL_OK != HAL_TIM_PWM_Start(hesc->htim, TIM_CHANNEL_4)) {
        /* END MODIFY 4 */
        return false;
    }

    return true;
}

bool_t PWM_Deinit(ESC_HandleTypeDef_t * hesc) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }

    /* Stop PWM signal generation */
    /* BEGIN MODIFY 5 */
    if (HAL_OK != HAL_TIM_PWM_Stop(hesc->htim, TIM_CHANNEL_1)) {
        /* END MODIFY 5 */
        return false;
    }
    /* BEGIN MODIFY 6 */
    if (HAL_OK != HAL_TIM_PWM_Stop(hesc->htim, TIM_CHANNEL_2)) {
        /* END MODIFY 6 */
        return false;
    }
    /* BEGIN MODIFY 7 */
    if (HAL_OK != HAL_TIM_PWM_Stop(hesc->htim, TIM_CHANNEL_3)) {
        /* END MODIFY 7 */
        return false;
    }
    /* BEGIN MODIFY 8 */
    if (HAL_OK != HAL_TIM_PWM_Stop(hesc->htim, TIM_CHANNEL_4)) {
        /* END MODIFY 8 */
        return false;
    }

    /* De-initialize timer peripheral */
    /* BEGIN MODIFY 9 */
    if (HAL_OK != HAL_TIM_PWM_DeInit(hesc->htim)) {
        /* END MODIFY 9 */
        return false;
    }

    return true;
}

bool_t PWM_SetDutyCycle(ESC_HandleTypeDef_t * hesc, uint8_t channel, uint32_t dutyCycle) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }
    if (PWM_CHANNEL_1 != channel && PWM_CHANNEL_2 != channel && PWM_CHANNEL_3 != channel && PWM_CHANNEL_4 != channel) {
        return false;
    }

    /* Set duty cycle */
    if (PWM_CHANNEL_1 == channel) {
        /* BEGIN MODIFY 10 */
        TIM3->CCR1 = (uint16_t)dutyCycle;
        /* END MODIFY 10 */
    } else if (PWM_CHANNEL_2 == channel) {
        /* BEGIN MODIFY 11 */
        TIM3->CCR2 = (uint16_t)dutyCycle;
        /* END MODIFY 11 */
    } else if (PWM_CHANNEL_3 == channel) {
        /* BEGIN MODIFY 12 */
        TIM3->CCR3 = (uint16_t)dutyCycle;
        /* END MODIFY 12 */
    } else if (PWM_CHANNEL_4 == channel) {
        /* BEGIN MODIFY 13 */
        TIM3->CCR4 = (uint16_t)dutyCycle;
        /* END MODIFY 13 */
    }

    return true;
}

/* --- End of file ----------------------------------------------------------------------------- */
