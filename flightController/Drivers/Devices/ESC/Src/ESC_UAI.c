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
 * @file:    ESC_UAI.c
 * @date:    24/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   TODO
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "ESC_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MAX_PWM_VALUE (16384 - 1)            // 14 bits
#define MIN_ESC_SPEED (MAX_PWM_VALUE * 0.05) // 1ms
#define MAX_ESC_SPEED (MAX_PWM_VALUE * 0.10) // 2ms

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static bool_t ESC_CalculateSpeed(float speed, uint16_t * pwmValue);

/**
 * @brief  TODO
 * @param  TODO
 * @retval TODO
 */
static bool_t ESC_Calibrate(TIM_HandleTypeDef * htim, uint32_t channel);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t ESC_CalculateSpeed(float speed, uint16_t * pwmValue) {

    /* Check parameters */
    if (speed < 0 || speed > 100) {
        return false;
    }

    *pwmValue = (MAX_ESC_SPEED - MIN_ESC_SPEED) * (speed / 100) + MIN_ESC_SPEED;

    return true;
}

static bool_t ESC_Calibrate(TIM_HandleTypeDef * htim, uint32_t channel) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        return false;
    }

    /* Set ESC to maximum throttle */
    if (false == PWM_SetDutyCycle(htim, TIM_CHANNEL_ALL, MAX_ESC_SPEED)) {
        return false;
    }

    /* Wait 2 seconds */
    HAL_Delay(3000); // TODO

    /* Set ESC to minimum throttle */
    if (false == PWM_SetDutyCycle(htim, TIM_CHANNEL_ALL, MIN_ESC_SPEED)) {
        return false;
    }

    /* Wait 1 second */
    HAL_Delay(2000); // TODO

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */
bool_t ESC_Init(TIM_HandleTypeDef * htim) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }

    /* Start PWM signal generation */
    if (false == PWM_Init(htim, TIM_CHANNEL_ALL)) {
        return false;
    }

    /* Calibrate ESC */
    if (false == ESC_Calibrate(htim, TIM_CHANNEL_ALL)) {
        return false;
    }

    return true;
}

bool_t ESC_Deinit(TIM_HandleTypeDef * htim) {

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }

    /* Stop PWM signal generation */
    if (false == PWM_Deinit(htim, TIM_CHANNEL_ALL)) {
        return false;
    }

    return true;
}

bool_t ESC_SetSpeed(TIM_HandleTypeDef * htim, uint32_t channel, float speed) {

    uint16_t pwmValue;
    uint16_t * pwmValuePtr = &pwmValue;

    /* Check parameters */
    if (NULL == htim) {
        return false;
    }
    if (TIM_CHANNEL_1 != channel && TIM_CHANNEL_2 != channel && TIM_CHANNEL_3 != channel && TIM_CHANNEL_4 != channel && TIM_CHANNEL_ALL != channel) {
        return false;
    }
    if (speed < 0 || speed > 100) {
        return false;
    }

    /* Calculate PWM value */
    if (false == ESC_CalculateSpeed(speed, pwmValuePtr)) {
        return false;
    }

    /* Set PWM duty cycle */
    if (false == PWM_SetDutyCycle(htim, channel, *pwmValuePtr)) {
        return false;
    };

    return true;
}

/* --- End of file ----------------------------------------------------------------------------- */
