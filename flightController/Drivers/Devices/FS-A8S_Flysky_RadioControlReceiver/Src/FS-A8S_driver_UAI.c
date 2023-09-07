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
 * @date:    07/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.2.0
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

/* --- Macros definitions ---------------------------------------------------------------------- */
// #define USE_FREERTOS

#define IBUS_BUFFER_LENGTH     (0X20) // 32 BYTES
#define IBUS_COMMAND           (0x40) // 40
#define IBUS_CHANNELS          (0x0E) // 14
#define IBUS_CHANNEL_MAX_VALUE (10000)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
/**
 * @brief  Checks if first two bytes are correct.
 * @param  None
 * @retval true:  If first two bytes are correct.
 *         false: If first two bytes are not correct.
 */
static bool_t FSA8S_RC_CheckFirstByte();

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
static bool_t FSA8S_RC_CheckFirstByte(iBus_HandleTypeDef_t * hibus) {
    if (IBUS_BUFFER_LENGTH == hibus->buffer[0] && IBUS_COMMAND == hibus->buffer[1]) {
        return true;
    } else {
        return false;
    }
}

static bool_t FSA8S_RC_Checksum(iBus_HandleTypeDef_t * hibus) {
    uint16_t sentChecksum;
    uint16_t receivedChecksum = 0xFFFF;

    sentChecksum =
        (hibus->buffer[hibus->bufferSize - 1] << 8) | (hibus->buffer[hibus->bufferSize - 2]);

    for (uint8_t i = 0; i < 30; i++) {
        receivedChecksum -= hibus->buffer[i];
    }

    if (sentChecksum == receivedChecksum) {
        return true;
    } else {
        return false;
    }
}

static void FSA8S_RC_AmendData(iBus_HandleTypeDef_t * hibus) {

    uint16_t channelValue;

    for (uint8_t i = 2; i <= (hibus->bufferSize - 2); i += 2) {

        channelValue = 0;

        channelValue = ((hibus->buffer[i + 1] << 8) | (hibus->buffer[i]));

        if ((1000 <= channelValue) && (2000 >= channelValue)) {
            channelValue -= 1000;
        } else {
            channelValue = 0;
        }

        hibus->data[(i - 2) / 2] = channelValue * (IBUS_CHANNEL_MAX_VALUE / 1000);
    }
}

/* --- Public function implementation ---------------------------------------------------------- */
iBus_HandleTypeDef_t * FSA8S_RC_Init(UART_HandleTypeDef * huart, uint8_t * buffer) {

#ifdef USE_FREERTOS
    iBus_HandleTypeDef_t * hibus = pvPortmalloc(sizeof(iBus_HandleTypeDef_t));
    uint16_t * data = pvortMalloc(sizeof(uint16_t) * IBUS_CHANNELS);
#else
    iBus_HandleTypeDef_t * hibus = malloc(sizeof(iBus_HandleTypeDef_t));
    uint16_t * data = malloc(sizeof(uint16_t));
#endif

    if (hibus) {
        hibus->huart = huart;
        hibus->buffer = buffer;
        hibus->bufferSize = IBUS_BUFFER_LENGTH;
        hibus->data = data;
        hibus->channels = IBUS_CHANNELS;
    }

    if (iBus_Init(hibus)) {
        return hibus;
    } else {
        return NULL;
    }
}

uint16_t FSA8S_RC_ReadChannel(iBus_HandleTypeDef_t * hibus, FSA8S_RC_CHANNEL_t channel) {

    FSA8S_RC_CheckFirstByte(hibus);
    FSA8S_RC_Checksum(hibus);
    FSA8S_RC_AmendData(hibus);

    return hibus->data[channel - 1];
}
/* --- End of file ----------------------------------------------------------------------------- */