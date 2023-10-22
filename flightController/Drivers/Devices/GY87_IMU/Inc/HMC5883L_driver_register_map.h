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
 * @file:    HMC5883L_driver_register_map.h
 * @date:    29/09/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.3.0
 *
 * @brief:   This file contains the register map for the HMC5883L magnetometer.
 */

#ifndef INC_HMC5883L_DRIVER_REGISTER_MAP_H
#define INC_HMC5883L_DRIVER_REGISTER_MAP_H

/* --- Public macros definitions --------------------------------------------------------------- */

/*
 * @brief HMC5883L Register Map and Descriptions
 */
#define HMC5883L_AUX_VAL_I2C_ADDR (0x1E)
#define HMC5883L_REG_CONFIG_A     (0x00)
#define HMC5883L_REG_CONFIG_B     (0x01)
#define HMC5883L_REG_MODE         (0x02)
#define HMC5883L_REG_OUT_X_M      (0x03)
#define HMC5883L_REG_OUT_X_L      (0x04)
#define HMC5883L_REG_OUT_Z_M      (0x05)
#define HMC5883L_REG_OUT_Z_L      (0x06)
#define HMC5883L_REG_OUT_Y_M      (0x07)
#define HMC5883L_REG_OUT_Y_L      (0x08)
#define HMC5883L_REG_STATUS       (0x09)

#define HMC5883L_REG_ID_A         (0x0A)
#define HMC5883L_BIT_ID_A         (0X48)
#define HMC5883L_REG_ID_B         (0x0B)
#define HMC5883L_BIT_ID_B         (0X34)
#define HMC5883L_REG_ID_C         (0x0C)
#define HMC5883L_BIT_ID_C         (0X33)

#endif /* INC_HMC5883L_DRIVER_REGISTER_MAP_H */

/* --- End of file ----------------------------------------------------------------------------- */
