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
 * @brief:   TODO
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
bool_t PWM_Init(TIM_HandleTypeDef * htim, uint32_t channel) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        return false;
    }

    /* Initialize timer peripheral */
    HAL_TIM_PWM_Init(htim);

    /* Start PWM signal generation */
    HAL_TIM_PWM_Start(htim, channel);

    return true;
}

bool_t PWM_Deinit(TIM_HandleTypeDef * htim, uint32_t channel) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        return false;
    }

    /* Stop PWM signal generation */
    HAL_TIM_PWM_Stop(htim, channel);

    /* Deinitialize timer peripheral */
    HAL_TIM_PWM_DeInit(htim);

    return true;
}

bool_t PWM_SetDutyCycle(TIM_HandleTypeDef * htim, uint32_t channel, uint16_t dutyCycle) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        return false;
    }
    if (dutyCycle > 16383) {
        return false;
    }

    /* Set duty cycle */
    switch (channel) {

    case TIM_CHANNEL_1:
        TIM3->CCR1 = dutyCycle;
        break;

    case TIM_CHANNEL_2:
        TIM3->CCR2 = dutyCycle;
        break;

    case TIM_CHANNEL_3:
        TIM3->CCR3 = dutyCycle;
        break;

    case TIM_CHANNEL_4:
        TIM3->CCR4 = dutyCycle;
        break;

    default:
        return false;
        break;
    }

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */

/* --- End of file ----------------------------------------------------------------------------- */
