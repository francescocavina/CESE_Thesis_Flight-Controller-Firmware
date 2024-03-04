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
 * @file:    QMC5883L_driver_register_map.h
 * @date:    03/03/2024
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.4.0
 *
 * @brief:   This file contains the register map for the QMC5883L magnetometer.
 */

#ifndef INC_QMC5883L_DRIVER_REGISTER_MAP_H
#define INC_QMC5883L_DRIVER_REGISTER_MAP_H

/* --- Public macros definitions --------------------------------------------------------------- */
/*
 * @brief QMC5883L Register Map and Descriptions
 */
#define QMC5883L_AUX_VAL_I2C_ADDR (0x0D)
#define QMC5883L_REG_X_LSB        (0x00)
#define QMC5883L_REG_X_MSB        (0x01)
#define QMC5883L_REG_Y_LS         (0x02)
#define QMC5883L_REG_Y_MSB        (0x03)
#define QMC5883L_REG_Z_LSB        (0x04)
#define QMC5883L_REG_Z_MSB        (0x05)
#define QMC5883L_REG_STATUS       (0x06)
#define QMC5883L_REG_TEMP_LSB     (0x07)
#define QMC5883L_REG_TEMP_MSB     (0x08)
#define QMC5883L_REG_CONFIG1      (0x09)
#define QMC5883L_REG_CONFIG2      (0x0A)
#define QMC5883L_REG_RESET        (0x0B)
#define QMC5883L_REG_RESERVED     (0x0C)
#define QMC5883L_REG_CHIP_ID      (0x0D)
#define QMC5883L_BIT_CHIP_ID      (0xFF)

#endif /* INC_QMC5883L_DRIVER_REGISTER_MAP_H */

/* --- End of file ----------------------------------------------------------------------------- */
