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
 * @file:    FS-A8S_driver_UAI.c
 * @date:    23/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.6.0
 *
 * @brief:   This is a driver for the radio control receiver FlySky FS-A8S.
 *           It is divided in two parts: One high level abstraction layer
 *           (FS-A8S_driver_UAI.c and FS-A8S_driver_UAI.h) for interface with the
 *           user application and one low level abstraction layer
 *           (FS-A8S_driver_HWI.c and FS-A8S_driver_HWI.h) for interface with the
 *           hardware (also known as port). In case of need to port this driver
 *           to another platform, please only modify the low layer abstraction
 *           layer files where the labels indicate it.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "FS-A8S_driver_UAI.h"
#include "FS-A8S_driver_calibration.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
// #define USE_FREERTOS
#define IBUS_BUFFER_LENGTH         (0X20) // 32 BYTES
#define IBUS_COMMAND               (0x40) // 40
#define IBUS_CHANNELS              (0x0E) // 14
#define IBUS_CHANNEL_MAX_VALUE     (1000) // This value MUST be between 100 and 65535
#define IBUS_CHANNEL_MIN_RAW_VALUE (1000) // Minimum raw value received from receiver
#define IBUS_CHANNEL_MAX_RAW_VALUE (2000) // Maximum raw value received from receiver
#define IBUS_CHANNEL_NUM_OFFSET    (1)
#define IBUS_CHANNEL_VALUE_NULL    (0) // Null channel value

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t calibrationValues[IBUS_CHANNELS] = {
    CHANNEL_01_CALIBRATION_VALUE, CHANNEL_02_CALIBRATION_VALUE, CHANNEL_03_CALIBRATION_VALUE,
    CHANNEL_04_CALIBRATION_VALUE, CHANNEL_05_CALIBRATION_VALUE, CHANNEL_06_CALIBRATION_VALUE,
    CHANNEL_07_CALIBRATION_VALUE, CHANNEL_08_CALIBRATION_VALUE, CHANNEL_09_CALIBRATION_VALUE,
    CHANNEL_10_CALIBRATION_VALUE, CHANNEL_11_CALIBRATION_VALUE, CHANNEL_12_CALIBRATION_VALUE,
    CHANNEL_13_CALIBRATION_VALUE, CHANNEL_14_CALIBRATION_VALUE};

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  Checks if first two bytes are correct.
 * @param  None
 * @retval true:  If first two bytes are correct.
 *         false: If first two bytes are not correct.
 */
static bool_t FSA8S_RC_CheckFirstBytes();

/**
 * @brief  Checks if data received is not corrupted calculating a checksum.
 * @param  None
 * @retval true:  If data is is correct.
 *         false: If data is corrupted.
 */
static bool_t FSA8S_RC_Checksum();

/**
 * @brief  Swaps bytes for each channel as the data is sent in big-endian system and stores it
 *         in another buffer.
 * @param  None
 * @retval None
 */
static void FSA8S_RC_AmendData();

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t FSA8S_RC_CheckFirstBytes(IBUS_HandleTypeDef_t * hibus) {

    /* Check parameter */
    if (NULL == hibus) {
        return false;
    }

    /* Check first bytes*/
    if (IBUS_BUFFER_LENGTH == hibus->buffer[0] && IBUS_COMMAND == hibus->buffer[1]) {
        /* First two bytes are correct */
        return true;
    } else {
        /* First two bytes are not correct */
        return false;
    }
}

static bool_t FSA8S_RC_Checksum(IBUS_HandleTypeDef_t * hibus) {

    uint16_t sentChecksum;
    uint16_t receivedChecksum = 0xFFFF;

    /* Check parameter */
    if (NULL == hibus) {
        return false;
    }

    /* Get received checksum value */
    sentChecksum =
        (hibus->buffer[hibus->bufferSize - 1] << 8) | (hibus->buffer[hibus->bufferSize - 2]);

    /* Calculate checksum */
    for (uint8_t i = 0; i < 30; i++) {
        receivedChecksum -= hibus->buffer[i];
    }

    /* Compare received checksum value with calculated one */
    if (sentChecksum == receivedChecksum) {
        /* Received data is correct */
        return true;
    } else {
        /* Received data is corrupted */
        return false;
    }
}

static void FSA8S_RC_AmendData(IBUS_HandleTypeDef_t * hibus) {

    uint16_t channelValue;

    /* Check parameter */
    if (NULL != hibus) {

        /* Ammend data */
        for (uint8_t i = 2; i <= (hibus->bufferSize - 2); i += 2) {

            channelValue = IBUS_CHANNEL_VALUE_NULL;

            /* Swap channel bytes */
            channelValue =
                ((hibus->buffer[i + 1] << 8) | (hibus->buffer[i])) - calibrationValues[(i - 2) / 2];

            /* Map channel value from 0 to IBUS_CHANNEL_MAX_VALUE */
            if ((IBUS_CHANNEL_MIN_RAW_VALUE <= channelValue) &&
                (IBUS_CHANNEL_MAX_RAW_VALUE >= channelValue)) {
                channelValue -= IBUS_CHANNEL_MIN_RAW_VALUE;
            } else {
                channelValue = IBUS_CHANNEL_VALUE_NULL;
            }

            hibus->data[(i - 2) / 2] =
                channelValue * ((float)(IBUS_CHANNEL_MAX_VALUE + (calibrationValues[(i - 2) / 2] *
                                                                  ((float)IBUS_CHANNEL_MAX_VALUE /
                                                                   IBUS_CHANNEL_MIN_RAW_VALUE))) /
                                IBUS_CHANNEL_MIN_RAW_VALUE);
        }
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
IBUS_HandleTypeDef_t * FSA8S_RC_Init(UART_HandleTypeDef * huart) {

    static uint8_t alreadyInitialized = false;

    /* Check parameter */
    if (NULL == huart) {
        return NULL;
    }

    /* Check if driver was already initialized */
    if (alreadyInitialized) {
        return NULL;
    }

    /* Allocate dynamic memory for the IBUS_HandleTypeDef_t structure and for the buffer to receive
     * data */
#ifdef USE_FREERTOS
    IBUS_HandleTypeDef_t * hibus = pvPortmalloc(sizeof(IBUS_HandleTypeDef_t));
    uint8_t * buffer = pvortMalloc(sizeof(IBUS_BUFFER_LENGTH));
    uint16_t * data = pvortMalloc(sizeof(uint16_t) * IBUS_CHANNELS);
#else
    IBUS_HandleTypeDef_t * hibus = malloc(sizeof(IBUS_HandleTypeDef_t));
    uint8_t * buffer = malloc(sizeof(IBUS_BUFFER_LENGTH));
    uint16_t * data = malloc(sizeof(uint16_t));
#endif

    /* Initialize iBus_HandleTypeDef structure */
    if (hibus) {
        hibus->huart = huart;
        hibus->buffer = buffer;
        hibus->bufferSize = IBUS_BUFFER_LENGTH;
        hibus->data = data;
        hibus->channels = IBUS_CHANNELS;
    } else {
        /* Dynamic memory allocation was not successful */
#ifdef USE_FREERTOS
        /* Free up dynamic allocated memory */
        vPortFree(hibus->buffer);
        vPortFree(hibus);
#else
        /* Free up dynamic allocated memory */
        hibus->buffer = 0;
        free(hibus->buffer);
        free(hibus);
#endif
    }

    /* Initialize iBus communication */
    if (IBUS_Init(hibus)) {
        /* Initialization was successful */
        alreadyInitialized = true;
        return hibus;
    } else {
        /* Initialization was unsuccessful */
#ifdef USE_FREERTOS
        /* Free up dynamic allocated memory */
        vPortFree(hibus->buffer);
        vPortFree(hibus);
#else
        /* Free up dynamic allocated memory */
        hibus->buffer = 0;
        free(hibus->buffer);
        free(hibus);
#endif
        return NULL;
    }
}

uint16_t FSA8S_RC_ReadChannel(IBUS_HandleTypeDef_t * hibus, FSA8S_RC_CHANNEL_t channel) {

    /* Check parameter */
    if (NULL == hibus) {
        return IBUS_CHANNEL_VALUE_NULL;
    }
    /* Check parameter */
    if (!(channel > 0 && channel <= IBUS_CHANNELS)) {
        return IBUS_CHANNEL_VALUE_NULL;
    }

    /* Check if first two bytes are IBUS_LENGTH and IBUS_COMMAND */
    while (1) {
        while (!FSA8S_RC_CheckFirstBytes(hibus)) {
            /* Wait until a data frame with the right format is received */
        }

        /* Perform a checksum */
        if (!FSA8S_RC_Checksum(hibus)) {
            /* Received data is corrupted */
            continue;
        } else {
            /* Received data is correct */
            break;
        }
    }

    /* Get channels data in little-endian */
    FSA8S_RC_AmendData(hibus);

    return hibus->data[channel - IBUS_CHANNEL_NUM_OFFSET];
}
/* --- End of file ----------------------------------------------------------------------------- */
