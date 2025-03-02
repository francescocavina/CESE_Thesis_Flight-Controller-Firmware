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
 * @file:    LoggingSystem_UAI.h
 * @date:    03/02/2025
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v2.0.0
 *
 * @brief:   This is a driver for logging messages for the user via USB.
 *           It is divided in two parts: One high level abstraction layer
 *           (LoggingSystem_UAI.c and LoggingSystem_UAI.h) for interface with the
 *           user application and one low level abstraction layer
 *           (LoggingSystem_HWI.c and LoggingSystem_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver
 *           to another platform, please only modify the low layer abstraction
 *           layer files where the labels indicate it.
 */

#ifndef INC_LOGGING_SYSTEM_UAI_H
#define INC_LOGGING_SYSTEM_UAI_H

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "LoggingSystem_HWI.h"

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */

/* --- Public data type declarations ----------------------------------------------------------- */
/*
 * @brief Enumeration for logging messages types.
 */
typedef enum {
    LOG_INFORMATION = 0,
    LOG_DEBUGGING = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,
    LOG_RAW = 4,
} LOGGING_TYPE_t;

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */
/*
 * @brief  Sends a string through the USB port with Logging Type as prefix.
 * @param  message: string to send (must be null-terminated). The caller retains ownership.
 *         logType: type of message = LOG_INFORMATION, LOG_DEBUGGING, LOG_WARNING, LOG_ERROR, or LOG_RAW.
 * @retval true:    message was successfully sent to the USB buffer.
 *         false:   message couldn't be sent to the USB buffer.
 * @note   Maximum message length is defined by LOG_MESSAGE_MAX_LENGTH.
 * @note   This function is not thread-safe and should be protected if called from multiple tasks.
 * @example LOG((uint8_t*)"Motor initialized", LOG_INFORMATION);
 */
bool_t LOG(const uint8_t * message, LOGGING_TYPE_t logType);

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_LOGGING_SYSTEM_UAI_H */

/* --- End of file ----------------------------------------------------------------------------- */
