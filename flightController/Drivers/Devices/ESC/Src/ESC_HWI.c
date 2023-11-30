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
 * @date:    29/11/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.1.0
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
bool_t PWM_Init(ESC_HandleTypeDef_t * hesc, uint32_t channel) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }
    if (channel != hesc->channel1 && channel != hesc->channel2 && channel != hesc->channel3 && channel != hesc->channel4) {
        return false;
    }

    /* Initialize ESC_HandleTypeDef structure */
    /* BEGIN MODIFY 1 */
    hesc->channel1 = TIM_CHANNEL_1;
    hesc->channel2 = TIM_CHANNEL_2;
    hesc->channel3 = TIM_CHANNEL_3;
    hesc->channel4 = TIM_CHANNEL_4;
    hesc->CCR1 = (uint32_t *)&(TIM3->CCR1);
    hesc->CCR2 = (uint32_t *)&(TIM3->CCR2);
    hesc->CCR3 = (uint32_t *)&(TIM3->CCR3);
    hesc->CCR4 = (uint32_t *)&(TIM3->CCR4);
    /* END MODIFY 1 */

    /* Initialize timer peripheral */
    /* BEGIN MODIFY 2 */
    if (HAL_OK != HAL_TIM_PWM_Init(hesc->htim)) {
        /* END MODIFY 2 */
        return false;
    }

    /* Start PWM signal generation */
    /* BEGIN MODIFY 3 */
    if (HAL_OK != HAL_TIM_PWM_Start(hesc->htim, channel)) {
        /* END MODIFY 3 */
        return false;
    }

    return true;
}

bool_t PWM_Deinit(ESC_HandleTypeDef_t * hesc, uint32_t channel) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }
    if (channel != hesc->channel1 && channel != hesc->channel2 && channel != hesc->channel3 && channel != hesc->channel4) {
        return false;
    }

    /* Stop PWM signal generation */
    /* BEGIN MODIFY 4 */
    if (HAL_OK != HAL_TIM_PWM_Stop(hesc->htim, channel)) {
        /* END MODIFY 4 */
        return false;
    }

    /* De-initialize timer peripheral */
    /* BEGIN MODIFY 5 */
    if (HAL_OK != HAL_TIM_PWM_DeInit(hesc->htim)) {
        /* END MODIFY 5 */
        return false;
    }

    return true;
}

bool_t PWM_SetDutyCycle(ESC_HandleTypeDef_t * hesc, uint32_t channel, uint16_t dutyCycle) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }
    if (channel != hesc->channel1 && channel != hesc->channel2 && channel != hesc->channel3 && channel != hesc->channel4) {
        return false;
    }
    if (dutyCycle > MAX_PWM_VALUE) {
        return false;
    }

    /* Set duty cycle */
    if (hesc->channel1 == channel) {

        *(hesc->CCR1) = dutyCycle;
    } else if (hesc->channel2 == channel) {

        *(hesc->CCR2) = dutyCycle;
    } else if (hesc->channel3 == channel) {

        *(hesc->CCR3) = dutyCycle;
    } else if (hesc->channel4 == channel) {

        *(hesc->CCR4) = dutyCycle;
    } else {

        return false;
    }

    return true;
}

void ESC_SetTimeDelay(uint32_t delay) {

/* No need to check parameters */

/* Set delay */
#ifdef USE_FREERTOS
    const TickType_t xDelay = pdMS_TO_TICKS(delay);
    vTaskDelay(xDelay);
#else
    HAL_Delay(delay);
#endif
}

/* --- Public function implementation ---------------------------------------------------------- */

/* --- End of file ----------------------------------------------------------------------------- */
