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
 * @file:    MPU-6050_driver_UAI.c
 * @date:    16/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.0.0
 *
 * @brief:   This is a template for source files.
 */

/* --- Headers files inclusions ---------------------------------------------------------------- */
#include "MPU-6050_driver_UAI.h"

/* --- Macros definitions ---------------------------------------------------------------------- */
#define MPU6050_ADDR1                (0xD0) // First address for first MPU-6050 device
#define MPU6050_ADDR2                (0xD1) // Second address for second MPU-6050 device
#define MPU6050_MAX_NUMBER_INSTANCES (2) // Maximum number of possible IMUs connected to the i2c bus

/* --- Private data type declarations ---------------------------------------------------------- */

/* --- Private variable declarations ----------------------------------------------------------- */
static uint8_t instancesNumber = 0;

/* --- Private function declarations ----------------------------------------------------------- */
/*
 * @brief
 * @param
 * @retval
 */
static MPU6050_HandleTypeDef_t * MPU6050_IMU_InstanceInit();

/* --- Public variable definitions ------------------------------------------------------------- */

/* --- Private variable definitions ------------------------------------------------------------ */

/* --- Private function implementation --------------------------------------------------------- */
static MPU6050_HandleTypeDef_t * MPU6050_IMU_InstanceInit() {

    /* Check if driver was already once or twice initialized */
    if (MPU6050_MAX_NUMBER_INSTANCES == instancesNumber) {
        return NULL;
    }

#ifdef USE_FREERTOS
    /* Allocate dynamic memory for the MPU6050_HandleTypeDef_t structure */
    MPU6050_HandleTypeDef_t * hmpu6050 = pvPortmalloc(sizeof(MPU6050_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = pvortMalloc(sizeof(1));
#else
    /* Allocate dynamic memory for the MPU6050_HandleTypeDef_t structure */
    MPU6050_HandleTypeDef_t * hmpu6050 = malloc(sizeof(MPU6050_HandleTypeDef_t));

    /* Allocate dynamic memory for data buffer */
    uint8_t * buffer = malloc(sizeof(1));
#endif

    /* Check if dynamic memory allocation was successful */
    if (NULL == hmpu6050 || NULL == buffer) {
        /* Dynamic memory allocation was not successful */
#ifdef USE_FREERTOS
        /* Free up dynamic allocated memory */
        vPortFree(hmpu6050->buffer);
        vPortFree(hmpu6050);
#else
        /* Free up dynamic allocated memory */
        free(hmpu6050->buffer);
        free(hmpu6050);
#endif
        return NULL;
    } else {
        /* Dynamic memory allocation was successful */

        /* Initialize MPU6050_HandleTypeDef_t structure */
        if (instancesNumber == 0) {
            hmpu6050->instance = 1;
            hmpu6050->address = MPU6050_ADDR1;
        } else if (instancesNumber == 1) {
            hmpu6050->instance = 2;
            hmpu6050->address = MPU6050_ADDR2;
        }
        hmpu6050->buffer = buffer;
    }

    /* Return created instance */
    return hmpu6050;
}

/* --- Public function implementation ---------------------------------------------------------- */
MPU6050_HandleTypeDef_t * MPU6050_IMU_Init() {

    /* Create an instance of the MPU6050_IMU device */
    MPU6050_HandleTypeDef_t * hmpu6050 = MPU6050_IMU_InstanceInit();

    /* Check if instance was successfully created */
    if (NULL != hmpu6050) {
        /* Instance was successfully created */

        /* Initialize I2C communication */
        if (i2c_Init(hmpu6050)) {

            /* Initialization was successful */
            instancesNumber++;

            return hmpu6050;
        } else {

            /* Initialization was unsuccessful */
#ifdef USE_FREERTOS
            /* Free up dynamic allocated memory */
            vPortFree(hmpu6050->buffer);
            vPortFree(hmpu6050);
#else
            /* Free up dynamic allocated memory */
            free(hmpu6050->buffer);
            free(hmpu6050);
#endif

            return NULL;
        }
    } else {

        /* Instance couldn't be created */
        return NULL;
    }
}

/* --- End of file ----------------------------------------------------------------------------- */
