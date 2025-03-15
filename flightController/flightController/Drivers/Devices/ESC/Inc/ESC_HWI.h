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
 * @file:    ESC_HWI.h
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

#ifndef INC_ESC_HWI_H
#define INC_ESC_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* BEGIN MODIFY 1 */
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
/* END MODIFY 1 */
#include "LoggingSystem_UAI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */
#define MAX_PWM_VALUE (65536 - 1)

/* --- Public data type declarations ----------------------------------------------------------- */
/**
 * @brief bool_t type declaration.
 */
typedef bool bool_t;

/**
 * @brief ESC handle structure declaration.
 */
typedef struct {
    /* BEGIN MODIFY 2 */
    TIM_HandleTypeDef *htim; /* Pointer to TIM_HandleTypeDef structure */
    /* END MODIFY 2 */
    uint8_t esc1;
    uint8_t esc2;
    uint8_t esc3;
    uint8_t esc4;
} ESC_HandleTypeDef_t;

/**
 * @brief Enumeration for PWM channel number.
 */
typedef enum { PWM_CHANNEL_1 = 1,
               PWM_CHANNEL_2 = 2,
               PWM_CHANNEL_3 = 3,
               PWM_CHANNEL_4 = 4 } PWM_CHANNEL_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Initializes the PWM Timer peripheral.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					information for the communication with the ESC device.
 * @retval true:    If PWM could be initialized.
 *         false:   If PWM couldn't be initialized.
 */
bool_t PWM_Init(ESC_HandleTypeDef_t *hesc);

/**
 * @brief  De-initializes the PWM Timer peripheral.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					information for the communication with the ESC device.
 * @retval true:    If PWM could be de-initialized.
 *         false:   If PWM couldn't be de-initialized.
 */
bool_t PWM_Deinit(ESC_HandleTypeDef_t *hesc);

/**
 * @brief  Sets PWM duty cycle.
 * @param  hesc:     Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					 information for the communication with the ESC device.
 * 		   channel:  Channel to set duty cycle.
 * 		   dutyCyle: Percentage of the total PWM period where the signal is on an ON state.
 * 		             The duty cycle can be calculated as Ton / (Ton + Toff). In this driver,
 * 		             duty cycle goes from 0 to 65535 (16 bit PWM).
 * @retval true:     If duty cycle could be set.
 *         false:    If duty cycle couldn't be set.
 */
bool_t PWM_SetDutyCycle(ESC_HandleTypeDef_t *hesc, uint8_t channel, uint32_t dutyCycle);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_ESC_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
