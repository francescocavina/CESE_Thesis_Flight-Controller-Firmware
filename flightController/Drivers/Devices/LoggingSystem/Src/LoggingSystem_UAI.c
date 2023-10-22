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
 * @date:    21/10/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   TODO
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "LoggingSystem_UAI.h"

#include <string.h>

/* --- Macros definitions ---------------------------------------------------------------------- */
#define LOG_MESSAGE_MAX_LENGTH 50

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t * informationTypeLabel = (uint8_t *)"LOG:INFO:    ";
static uint8_t * debuggingTypeLabel = (uint8_t *)"LOG:DEBUG:   ";
static uint8_t * warningTypeLabel = (uint8_t *)"LOG:WARNING: ";
static uint8_t * errorTypeLabel = (uint8_t *)"LOG:ERROR:   ";

/* --- Private function declarations ----------------------------------------------------------- */

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */

/* --- Public function implementation ---------------------------------------------------------- */
int8_t LOG(uint8_t * message, LOGGING_TYPE_t logType) {

    /* Check parameters */
    if (NULL == message) {
        return -1;
    }

    if (LOG_INFORMATION < 0 || logType > LOG_ERROR) {
        return -1;
    }

    /* Build log message */
    uint8_t logMessage[LOG_MESSAGE_MAX_LENGTH] = {0};

    if (LOG_INFORMATION == logType) {

        strcat((char *)logMessage, (char *)informationTypeLabel);
    } else if (LOG_DEBUGGING == logType) {

        strcat((char *)logMessage, (char *)debuggingTypeLabel);
    } else if (LOG_WARNING == logType) {

        strcat((char *)logMessage, (char *)warningTypeLabel);
    } else if (LOG_ERROR == logType) {

        strcat((char *)logMessage, (char *)errorTypeLabel);
    }

    strcat((char *)logMessage, (char *)message);

    /* Send message through USB port */
    USB_Write(logMessage);

    return 0;
}

/* --- End of file ----------------------------------------------------------------------------- */
