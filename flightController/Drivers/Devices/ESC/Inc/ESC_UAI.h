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
 * @file:    ESC_UAI.h
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

#ifndef INC_ESC_UAI_H
#define INC_ESC_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "ESC_HWI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Initializes the ESC device.
 * @param  htim:  Pointer to a TIM_HandleTypeDef structure that contains the configuration
 * 				  information for the Timer as well as for the PWM Channels.
 * @retval hesc:  Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 			      information for the communication with the ESC device.
 */
ESC_HandleTypeDef_t * ESC_Init(TIM_HandleTypeDef * htim);

/**
 * @brief  Deinitializes the ESC device.
 * @param  hesc:  Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 			      information for the communication with the ESC device.
 * @retval true:  If ESC device could be deinitialized.
 *         false: If ESC device couldn't be deinitialized.
 */
bool_t ESC_Deinit(ESC_HandleTypeDef_t * hesc);

/**
 * @brief  Sets the speed to a certain ESC device.
 * @param  hesc:    Pointer to a ESC_HandleTypeDef_t structure that contains the configuration
 * 			        information for the communication with the ESC device.
 * 		   channel: Channel to set the speed to.
 * 		   speed:   Speed to set (from 0 to 100).
 * @retval true:    Speed could be set.
 *         false:   Speed couldn't be set.
 */
bool_t ESC_SetSpeed(ESC_HandleTypeDef_t * hesc, uint8_t channel, float speed);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_ESC_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
