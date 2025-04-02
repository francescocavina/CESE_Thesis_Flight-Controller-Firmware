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
 * @file:    error_led.h
 * @date:    03/03/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:
 *
 * @details:
 */

#ifndef ERROR_LED_H
#define ERROR_LED_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "main.h"
#include <stdint.h>

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
/**
 * @brief Error codes enumeration.
 */
typedef enum {
    FSA8S_INITIALIZATION_ERROR,
    GY87_INITIALIZATION_ERROR,
    ESC_INITIALIZATION_ERROR,
    FREERTOS_TIMERS_CREATION_ERROR,
    FREERTOS_QUEUES_CREATION_ERROR,
    FREERTOS_SEMAPHORES_CREATION_ERROR,
    FREERTOS_TASKS_CREATION_ERROR,
    CONTROLSYSTEM_STATE_MACHINE_ERROR,
} ErrorLED_types_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/**
 * @brief  Starts the LED Error Signal.
 * @param  None
 * @retval None
 */
void ErrorLED_Start(ErrorLED_types_t errorType);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* ERROR_LED_H */

/* --- End of file ----------------------------------------------------------------------------- */
