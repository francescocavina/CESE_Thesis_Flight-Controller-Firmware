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
 * @file:    LoggingSystem_UAI.c
 * @date:    03/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
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

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "LoggingSystem_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
#define LOG_MESSAGE_MAX_LENGTH 1024

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static const uint8_t * informationTypeLabel = (uint8_t *)"LOG:INFO:    ";
static const uint8_t * debuggingTypeLabel = (uint8_t *)"LOG:DEBUG:   ";
static const uint8_t * warningTypeLabel = (uint8_t *)"LOG:WARNING: ";
static const uint8_t * errorTypeLabel = (uint8_t *)"LOG:ERROR:   ";

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
bool_t LOG(uint8_t * message, LOGGING_TYPE_t logType) {

    /* Check parameters */
    if (NULL == message) {
        return false;
    }

    if (logType > LOG_RAW) { // Remove unnecessary check
        return false;
    }

    /* Build log message */
    uint8_t logMessage[LOG_MESSAGE_MAX_LENGTH] = {0};

    // Calculate required buffer size to prevent overflow
    size_t prefixLen = 0;
    const uint8_t * prefix = NULL;

    if (logType != LOG_RAW) {
        if (LOG_INFORMATION == logType) {
            prefix = informationTypeLabel;
        } else if (LOG_DEBUGGING == logType) {
            prefix = debuggingTypeLabel;
        } else if (LOG_WARNING == logType) {
            prefix = warningTypeLabel;
        } else if (LOG_ERROR == logType) {
            prefix = errorTypeLabel;
        }
        prefixLen = strlen((const char *)prefix);
    }

    size_t messageLen = strlen((const char *)message);

    // Check for potential buffer overflow
    if ((prefixLen + messageLen) >= LOG_MESSAGE_MAX_LENGTH) {
        // Message too long - either truncate or return error
        return false;
    }

    // Safe concatenation
    if (prefix) {
        strcat((char *)logMessage, (const char *)prefix);
    }
    strcat((char *)logMessage, (const char *)message);

    /* Send message through USB port */
    return USB_Write(logMessage);
}

/* --- End of file ----------------------------------------------------------------------------- */
