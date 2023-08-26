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
 * @date:    20/08/2023
 * @author:  francescocavina98@gmail.com
 * @version: v1.0.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "FS-A8S_driver_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define IBUS_BUFFER_LENGHT (0X20)
#define IBUS_CHANNELS      (0x40)

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */

/* --- Private function declarations ----------------------------------------------------------- */
static bool_t FSA8S_RC_CheckFirstByte();
static bool_t FSA8S_RC_Checksum();
static bool_t FSA8S_RC_AmendData();

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static bool_t FSA8S_RC_CheckFirstByte(iBus_HandleTypeDef_t * hibus) {
    if (IBUS_BUFFER_LENGHT == hibus->buffer[0] && IBUS_CHANNELS == hibus->buffer[1]) {
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

static bool_t FSA8S_RC_AmendData(iBus_HandleTypeDef_t * hibus) {
    for (uint8_t i = 2; i <= (hibus->bufferSize - 2); i += 2) {
        hibus->buffer[i] = (hibus->buffer[i + 1] << 8) | (hibus->buffer[i]);
    }

    return true;
}

/* --- Public function implementation ---------------------------------------------------------- */
iBus_HandleTypeDef_t * FSA8S_RC_Init(UART_HandleTypeDef * huart, uint8_t * buffer) {
    iBus_HandleTypeDef_t * hibus = malloc(sizeof(iBus_HandleTypeDef_t));

    if (hibus) {
        hibus->huart = huart;
        hibus->buffer = buffer;
        hibus->bufferSize = IBUS_BUFFER_LENGHT;
    }

    if (iBus_Init(hibus)) {
        return hibus;
    } else {
        return NULL;
    }
}

uint16_t FSA8S_RC_ReadChannel(iBus_HandleTypeDef_t * hibus, FSA8S_RC_CHANNEL_t channel) {
    int16_t channelValue;

    FSA8S_RC_CheckFirstByte(hibus);
    FSA8S_RC_Checksum(hibus);
    FSA8S_RC_AmendData(hibus);

    channelValue = (hibus->buffer[(channel * 2) + 1] << 8) | (hibus->buffer[channel * 2]);

    return channelValue;
}
/* --- End of file ----------------------------------------------------------------------------- */
