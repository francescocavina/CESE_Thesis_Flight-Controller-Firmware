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

#ifndef INC_ESC_HWI_H
#define INC_ESC_HWI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* BEGIN MODIFY 1 */
#include "stm32f4xx_hal.h"
/* END MODIFY 1 */
#include "LoggingSystem_UAI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */
#define MAX_PWM_VALUE (16384 - 1)            // 14 bits
#define MIN_ESC_SPEED (MAX_PWM_VALUE * 0.05) // 1ms
#define MAX_ESC_SPEED (MAX_PWM_VALUE * 0.10) // 2ms

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
    TIM_HandleTypeDef * htim; /* Pointer to TIM_HandleTypeDef structure */
    /* END MODIFY 2 */
    uint32_t channel1; // Value of channel 1 identifier
    uint32_t channel2; // Value of channel 2 identifier
    uint32_t channel3; // Value of channel 3 identifier
    uint32_t channel4; // Value of channel 4 identifier
    uint32_t * CCR1;   // Capture/Compare register for channel 1
    uint32_t * CCR2;   // Capture/Compare register for channel 2
    uint32_t * CCR3;   // Capture/Compare register for channel 3
    uint32_t * CCR4;   // Capture/Compare register for channel 4
} ESC_HandleTypeDef_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Initializes the PWM Timer peripheral.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					information for the communication with the ESC device.
 * 		   channel: Channel to initialize.
 * @retval true:    If PWM could be initialized.
 *         false:   If PWM couldn't be initialized.
 */
bool_t PWM_Init(ESC_HandleTypeDef_t * hesc, uint32_t channel);

/**
 * @brief  De-initializes the PWM Timer peripheral.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					information for the communication with the ESC device.
 * 		   channel: Channel to initialize.
 * @retval true:    If PWM could be de-initialized.
 *         false:   If PWM couldn't be de-initialized.
 */
bool_t PWM_Deinit(ESC_HandleTypeDef_t * hesc, uint32_t channel);

/**
 * @brief  Sets PWM duty cycle.
 * @param  hesc:     Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 					 information for the communication with the ESC device.
 * 		   channel:  Channel to initialize.
 * 		   dutyCyle: Percentage of the total PWM period where the signal is on an ON state.
 * 		             The duty cycle can be calculated as Ton / (Ton + Toff). In this driver,
 * 		             duty cycle goes from 0 to 16.383 (14 bit PWM).
 * @retval true:     If duty cycle could be set.
 *         false:    If duty cycle couldn't be set.
 */
bool_t PWM_SetDutyCycle(ESC_HandleTypeDef_t * hesc, uint32_t channel, uint16_t dutyCycle);

/**
 * @brief  Provides a time delay.
 * @param  Time in mili-seconds.
 * @retval None
 */
void ESC_SetTimeDelay(uint32_t delay);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_ESC_HWI_H */

/* --- End of file ----------------------------------------------------------------------------- */
