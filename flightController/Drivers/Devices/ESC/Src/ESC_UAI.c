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
#include "ESC_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define USE_FREERTOS // Remove comment when using FreeRTOS
// #define ESC_USE_LOGGING                 // Remove comment to allow driver info logging

#define MIN_ESC_SPEED                   (MAX_PWM_VALUE * 0.05) // 1ms
#define MAX_ESC_SPEED                   (MAX_PWM_VALUE * 0.10) // 2ms

#define ESC_AUTOCALIBRATION_WAIT_TIME_1 (2000) // Time to wait after ESC was set to maximum throttle
#define ESC_AUTOCALIBRATION_WAIT_TIME_2 (1000) // Time to wait after ESC was set to minimum throttle

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  Calculates PWM duty cycle.
 * @param  speed:    Speed percentage (from 0.00 to 100.00).
 * 		   pwmValue: Pointer to variable to store the result.
 * @retval true:     If PWM value could be calculated.
 *         false:    If PWM value couldn't be calculated.
 */
static bool_t ESC_CalculatePWMDutyCycle(float speed, uint32_t * pwmValue);

/**
 * @brief  Auto-calibrates ESC. This is necessary after powering on the ESC device.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 			        information for the communication with the ESC device.
 * 		   channel: Channel to auto-calibrate.
 * @retval true:    If ESC device could be calibrated.
 *         false:   If ESC device couldn't be calibrated.
 */
static bool_t ESC_AutoCalibrate(ESC_HandleTypeDef_t * hesc);

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t ESC_CalculatePWMDutyCycle(float speed, uint32_t * pwmValue) {

    /* Check parameters */
    if (0 > speed || 100 < speed) {
        return false;
    }

    /* Calculate PWM value */
    *pwmValue = (uint32_t)((MAX_ESC_SPEED - MIN_ESC_SPEED) * (float)(speed / 100) + MIN_ESC_SPEED);

    return true;
}

static bool_t ESC_AutoCalibrate(ESC_HandleTypeDef_t * hesc) {

    /* Check parameters */
    if (NULL == hesc) {
        return false;
    }

#ifdef ESC_USE_LOGGING
    LOG((uint8_t *)"Auto-calibrating ESCs...\r\n\n", LOG_INFORMATION);
#endif

    /* Set ESC to maximum throttle */
    if (false == PWM_SetDutyCycle(hesc, hesc->esc1, MAX_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc2, MAX_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc3, MAX_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc4, MAX_ESC_SPEED)) {
        return false;
    }

    /* Set ESC to minimum throttle */
    if (false == PWM_SetDutyCycle(hesc, hesc->esc1, MIN_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc2, MIN_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc3, MIN_ESC_SPEED)) {
        return false;
    }
    if (false == PWM_SetDutyCycle(hesc, hesc->esc4, MIN_ESC_SPEED)) {
        return false;
    }

#ifdef ESC_USE_LOGGING
    LOG((uint8_t *)"ESCs auto-calibrated.\r\n\n", LOG_INFORMATION);
#endif

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */
ESC_HandleTypeDef_t * ESC_Init(TIM_HandleTypeDef * htim) {

    /* Check parameters */
    if (NULL == htim) {
        return NULL;
    }

#ifdef ESC_USE_LOGGING
    LOG((uint8_t *)"Initializing ESCs...\r\n\n", LOG_INFORMATION);
#endif

    /* Allocate dynamic memory for the ESC_HandleTypeDef_t structure */
#ifdef USE_FREERTOS
    ESC_HandleTypeDef_t * hesc = pvPortMalloc(sizeof(ESC_HandleTypeDef_t));
#else
    ESC_HandleTypeDef_t * hesc = malloc(sizeof(ESC_HandleTypeDef_t));
#endif

    /* Initialize ESC_HandleTypeDef structure */
    if (hesc) {
        hesc->htim = htim;
        hesc->esc1 = PWM_CHANNEL_4;
        hesc->esc2 = PWM_CHANNEL_2;
        hesc->esc3 = PWM_CHANNEL_3;
        hesc->esc4 = PWM_CHANNEL_1;
    } else {
        /* Dynamic memory allocation was not successful */
        /* Free up dynamic allocated memory */
#ifdef USE_FREERTOS
        vPortFree(hesc);
#else
        free(hesc);
#endif
    }

    /* Start PWM signal generation */
    if (false == PWM_Init(hesc)) {
#ifdef ESC_USE_LOGGING
        LOG((uint8_t *)"ESCs  couldn't be initialized.\r\n\n", LOG_ERROR);
#endif

/* Free up dynamic allocated memory */
#ifdef USE_FREERTOS
        vPortFree(hesc);
#else
        free(hesc);
#endif

        return NULL;
    }

    /* Calibrate ESC */
    if (false == ESC_AutoCalibrate(hesc)) {
#ifdef ESC_USE_LOGGING
        LOG((uint8_t *)"ESCs couldn't be calibrated.\r\n\n", LOG_ERROR);
#endif

/* Free up dynamic allocated memory */
#ifdef USE_FREERTOS
        vPortFree(hesc);
#else
        free(hesc);
#endif

        return NULL;
    }

#ifdef ESC_USE_LOGGING
    LOG((uint8_t *)"ESCs initialized.\r\n\n", LOG_INFORMATION);
#endif

    return hesc;
}

bool_t ESC_Deinit(ESC_HandleTypeDef_t * hesc) {

    /* Check parameters */
    if (NULL == hesc->htim) {
        return false;
    }

    /* Stop PWM signal generation */
    if (false == PWM_Deinit(hesc)) {
        return false;
    }

    /* Free up dynamic allocated memory */
#ifdef USE_FREERTOS
    vPortFree(hesc);
#else
    free(hesc);
#endif

    return true;
}

bool_t ESC_SetSpeed(ESC_HandleTypeDef_t * hesc, uint8_t channel, float speed) {

    uint32_t pwmValue;

    /* Check parameters */
    if (NULL == hesc->htim) {
        return false;
    }
    if (PWM_CHANNEL_1 != channel && PWM_CHANNEL_2 != channel && PWM_CHANNEL_3 != channel && PWM_CHANNEL_4 != channel) {
        return false;
    }
    if (speed < 0 || speed > 100) {
        return false;
    }

    /* Calculate PWM duty cycle */
    if (false == ESC_CalculatePWMDutyCycle(speed, &pwmValue)) {
        return false;
    }

    /* Set PWM duty cycle */
    if (false == PWM_SetDutyCycle(hesc, channel, pwmValue)) {
        return false;
    };

    return true;
}

/* --- End of file ----------------------------------------------------------------------------- */
