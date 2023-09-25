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
 * @file:    MPU-6050_driver_register_map.h
 * @date:    25/08/2023
 * @author:  Francesco Cavina <francescocavina98@gmail.com>
 * @version: v1.1.0
 *
 * @brief:   This is a template for header files.
 */

#ifndef INC_MPU_6050_DRIVER_REGISTER_MAP_H
#define INC_MPU_6050_DRIVER_REGISTER_MAP_H

/* --- Headers files inclusions ---------------------------------------------------------------- */

/* --- C++ guard ------------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* --- Public macros definitions --------------------------------------------------------------- */
/*
 * @brief MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.0
 */
/* --- GYROSCOPE OFFSET - READ/WRITE ----------------------------------------------------------- */
#define MPU_6050_REG_XGOFFS_TC (0x00) // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define MPU_6050_REG_YGOFFS_TC (0x01)
#define MPU_6050_REG_ZGOFFS_TC (0x02)

/* --- FINE GAIN - READ/WRITE ------------------------------------------------------------------ */
#define MPU_6050_REG_X_FINE_GAIN (0x03)
#define MPU_6050_REG_Y_FINE_GAIN (0x04)
#define MPU_6050_REG_Z_FINE_GAIN (0x05)

/* --- USER-DEFINED TRIM VALUES FOR ACCELEROMETER - READ/WRITE --------------------------------- */
#define MPU_6050_REG_XA_OFFSET_H    (0x06) // User-defined trim values for accelerometer in X
#define MPU_6050_REG_XA_OFFSET_L_TC (0x07) // User-defined trim values for accelerometer in X
#define MPU_6050_REG_YA_OFFSET_H    (0x08) // User-defined trim values for accelerometer in Y
#define MPU_6050_REG_YA_OFFSET_L_TC (0x09) // User-defined trim values for accelerometer in Y
#define MPU_6050_REG_ZA_OFFSET_H    (0x0A) // User-defined trim values for accelerometer in Z
#define MPU_6050_REG_ZA_OFFSET_L_TC (0x0B) // User-defined trim values for accelerometer in Z

/* --- SELF-TEST REGISTERS - READ/WRITE -------------------------------------------------------- */
#define MPU_6050_REG_SELF_TEST_X (0x0D) // Self-Test Register X
#define MPU_6050_REG_SELF_TEST_Y (0x0E) // Self-Test Register Y
#define MPU_6050_REG_SELF_TEST_Z (0x0F) // Self-Test Register Z
#define MPU_6050_REG_SELF_TEST_A (0x10) // Self-Test Register A

/* USER-DEFINED TRIM VALUES FOR GYROSCOPES - READ/WRITE ---------------------------------------- */
#define MPU_6050_REG_XG_OFFS_USRH (0x13) // User-defined trim values for gyroscope in X
#define MPU_6050_REG_XG_OFFS_USRL (0x14) // User-defined trim values for gyroscope in X
#define MPU_6050_REG_YG_OFFS_USRH (0x15) // User-defined trim values for gyroscope in Y
#define MPU_6050_REG_YG_OFFS_USRL (0x16) // User-defined trim values for gyroscope in Y
#define MPU_6050_REG_ZG_OFFS_USRH (0x17) // User-defined trim values for gyroscope in Z
#define MPU_6050_REG_ZG_OFFS_USRL (0x18) // User-defined trim values for gyroscope in Z

/* --- CONFIGURATION - READ/WRITE -------------------------------------------------------------- */
#define MPU_6050_REG_SMPLRT_DIV   (0x19) // Sample Rate Divider
#define MPU_6050_REG_CONFIG       (0x1A) // Configuration
#define MPU_6050_REG_GYRO_CONFIG  (0x1B) // Gyroscope Configuration
#define MPU_6050_REG_ACCEL_CONFIG (0x1C) // Accelerometer Configuration

/* --- FREE FALL - READ/WRITE ------------------------------------------------------------------ */
#define MPU_6050_REG_FF_THR (0x1D) // Free Fall Acceleration Threshold
#define MPU_6050_REG_FF_DUR (0x1E) // Free Fall Duration

/* --- MOTION DETECCTION - READ/WRITE ---------------------------------------------------------- */
#define MPU_6050_REG_MOT_THR   (0x1F) // Motion Detection Threshold
#define MPU_6050_REG_MOT_DUR   (0x20) // Motion Detection Duration
#define MPU_6050_REG_ZMOT_THR  (0x21) // Zero-Motion Detection Threshold
#define MPU_6050_REG_ZRMOT_DUR (0x22) // Zero-Motion Detection Duration

/* --- FIFO ENABLE AND CONFIGURATION - READ/WRITE ---------------------------------------------- */
#define MPU_6050_REG_FIFO_EN (0x23) // FIFO Enable

/* --- AUXILIARY I2C BUS - READ/WRITE ---------------------------------------------------------- */
#define MPU_6050_REG_I2C_MST_CTRL  (0x24) // I2C Master Control

#define MPU_6050_REG_I2C_SLV0_ADDR (0x25) // I2C Slave 0 Control
#define MPU_6050_REG_I2C_SLV0_REG  (0x26) // I2C Slave 0 Control
#define MPU_6050_REG_I2C_SLV0_CTRL (0x27) // I2C Slave 0 Control

#define MPU_6050_REG_I2C_SLV1_ADDR (0x28) // I2C Slave 1 Control
#define MPU_6050_REG_I2C_SLV1_REG  (0x29) // I2C Slave 1 Control
#define MPU_6050_REG_I2C_SLV1_CTRL (0x2A) // I2C Slave 1 Control

#define MPU_6050_REG_I2C_SLV2_ADDR (0x2B) // I2C Slave 2 Control
#define MPU_6050_REG_I2C_SLV2_REG  (0x2C) // I2C Salve 2 Control
#define MPU_6050_REG_I2C_SLV2_CTRL (0x2D) // I2C Slave 2 Control

#define MPU_6050_REG_I2C_SLV3_ADDR (0x2E) // I2C Slave 3 Control
#define MPU_6050_REG_I2C_SLV3_REG  (0x2F) // I2C Slave 3 Control
#define MPU_6050_REG_I2C_SLV3_CTRL (0x30) // I2C Slave 3 Control

#define MPU_6050_REG_I2C_SLV4_ADDR (0x31) // I2C Slave 4 Control
#define MPU_6050_REG_I2C_SLV4_REG  (0x32) // I2C Slave 4 Control
#define MPU_6050_REG_I2C_SLV4_DO   (0x33) // I2C Slave 4 Control
#define MPU_6050_REG_I2C_SLV4_CTRL (0x34) // I2C Slave 4 Control
#define MPU_6050_REG_I2C_SLV4_DI   (0x35) // I2C Slave 4 Control

/* --- AUXILIARY I2C BUS - READ ONLY ----------------------------------------------------------- */
#define MPU_6050_REG_I2C_MST_STATUS (0x36) // I2C Master Status

/* --- INTERRUPTS - READ/WRITE ----------------------------------------------------------------- */
#define MPU_6050_REG_INT_PIN_CFG (0x37) // Interrupt Pin/Bypass Enable Configuration
#define MPU_6050_REG_INT_ENABLE  (0x38) // Interrupt Enable

/* --- INTERRUPTS - READ ONLY ------------------------------------------------------------------ */
#define MPU_6050_REG_DMP_INT_STATUS (0x39) // DMP Interrupt Status
#define MPU_6050_REG_INT_STATUS     (0x3A) // Interrupt Status: FIFO, I2C master, data ready

/* --- ACCELEROMETER MEASUREMENTS - READ ONLY -------------------------------------------------- */
#define MPU_6050_REG_ACCEL_XOUT_H (0x3B) // Accelerometer Measurement in X
#define MPU_6050_REG_ACCEL_XOUT_L (0x3C) // Accelerometer Measurement in X

#define MPU_6050_REG_ACCEL_YOUT_H (0x3D) // Accelerometer Measurement in Y
#define MPU_6050_REG_ACCEL_YOUT_L (0x3E) // Accelerometer Measurement in Y

#define MPU_6050_REG_ACCEL_ZOUT_H (0x3F) // Accelerometer Measurement in Z
#define MPU_6050_REG_ACCEL_ZOUT_L (0x40) // Accelerometer Measurement in Z

/* --- TEMPERATURE MEASUREMENTS - READ ONLY ---------------------------------------------------- */
#define MPU_6050_REG_TEMP_OUT_H (0x41) // Temperature Measurement
#define MPU_6050_REG_TEMP_OUT_L (0x42) // Temperature Measurement

/* --- GYROSCOPE MEASUREMENTS - READ ONLY ------------------------------------------------------ */
#define MPU_6050_REG_GYRO_XOUT_H (0x43) // Gyroscope Measurement in X
#define MPU_6050_REG_GYRO_XOUT_L (0x44) // Gyroscope Measurement in X

#define MPU_6050_REG_GYRO_YOUT_H (0x45) // Gyroscope Measurement in Y
#define MPU_6050_REG_GYRO_YOUT_L (0x46) // Gyroscope Measurement in Y

#define MPU_6050_REG_GYRO_ZOUT_H (0x47) // Gyroscope Measurement in Z
#define MPU_6050_REG_GYRO_ZOUT_L (0x48) // Gyroscope Measurement in Z

/* --- EXTERNAL SENSORS DATA - READ ONLY ------------------------------------------------------- */
#define MPU_6050_REG_EXT_SENS_DATA_00 (0x49) // External Sensor Data 00
#define MPU_6050_REG_EXT_SENS_DATA_01 (0x4A) // External Sensor Data 01
#define MPU_6050_REG_EXT_SENS_DATA_02 (0x4B) // External Sensor Data 02
#define MPU_6050_REG_EXT_SENS_DATA_03 (0x4C) // External Sensor Data 03
#define MPU_6050_REG_EXT_SENS_DATA_04 (0x4D) // External Sensor Data 04
#define MPU_6050_REG_EXT_SENS_DATA_05 (0x4E) // External Sensor Data 05
#define MPU_6050_REG_EXT_SENS_DATA_06 (0x4F) // External Sensor Data 06
#define MPU_6050_REG_EXT_SENS_DATA_07 (0x50) // External Sensor Data 07
#define MPU_6050_REG_EXT_SENS_DATA_08 (0x51) // External Sensor Data 08
#define MPU_6050_REG_EXT_SENS_DATA_09 (0x52) // External Sensor Data 09
#define MPU_6050_REG_EXT_SENS_DATA_10 (0x53) // External Sensor Data 10
#define MPU_6050_REG_EXT_SENS_DATA_11 (0x54) // External Sensor Data 11
#define MPU_6050_REG_EXT_SENS_DATA_12 (0x55) // External Sensor Data 12
#define MPU_6050_REG_EXT_SENS_DATA_13 (0x56) // External Sensor Data 13
#define MPU_6050_REG_EXT_SENS_DATA_14 (0x57) // External Sensor Data 14
#define MPU_6050_REG_EXT_SENS_DATA_15 (0x58) // External Sensor Data 15
#define MPU_6050_REG_EXT_SENS_DATA_16 (0x59) // External Sensor Data 16
#define MPU_6050_REG_EXT_SENS_DATA_17 (0x5A) // External Sensor Data 17
#define MPU_6050_REG_EXT_SENS_DATA_18 (0x5B) // External Sensor Data 18
#define MPU_6050_REG_EXT_SENS_DATA_19 (0x5C) // External Sensor Data 19
#define MPU_6050_REG_EXT_SENS_DATA_20 (0x5D) // External Sensor Data 20
#define MPU_6050_REG_EXT_SENS_DATA_21 (0x5E) // External Sensor Data 21
#define MPU_6050_REG_EXT_SENS_DATA_22 (0x5F) // External Sensor Data 22
#define MPU_6050_REG_EXT_SENS_DATA_23 (0x60) // External Sensor Data 23

/* --- MOTION DETECTION STATUS - READ ONLY ----------------------------------------------------- */
#define MPU_6050_REG_MOT_DETECT_STATUS                                                             \
    (0x61) // Motion Detection Status: reports axis and polarity of motion which generate a motion

/* --- I2C SLAVE DATA OUT - READ/WRITE --------------------------------------------------------- */
#define MPU_6050_REG_I2C_SLV0_DO (0x63) // I2C Slave 0 Data Out
#define MPU_6050_REG_I2C_SLV1_DO (0x64) // I2C Slave 1 Data Out
#define MPU_6050_REG_I2C_SLV2_DO (0x65) // I2C Slave 2 Data Out
#define MPU_6050_REG_I2C_SLV3_DO (0x66) // I2C Slave 3 Data Out

/* --- I2C MASTER DELAT CONTROL - READ/WRITE --------------------------------------------------- */
#define MPU_6050_REG_I2C_MST_DELAY_CTRL (0x67) // I2C Master Delay Control

/* --- SIGNAL PATH RESET - READ/WRITE ---------------------------------------------------------- */
#define MPU_6050_REG_SIGNAL_PATH_RESET (0x68) // Signal Path Reset

/* --- MOTION DETECTION CONTROL - READ/WRITE --------------------------------------------------- */
#define MPU_6050_REG_MOT_DETECT_CTRL (0x69) // Motion Detection Control

/* --- USER CONTROL - READ/WRITE --------------------------------------------------------------- */
#define MPU_6050_REG_USER_CTRL (0x6A) // User Control: bit 7 enable DMP, bit 3 reset DMP

/* --- POWER MANAGEMENT - READ/WRITE ----------------------------------------------------------- */
#define MPU_6050_REG_PWR_MGMT_1 (0x6B) // Power Management 1: Device defaults to the SLEEP mode
#define MPU_6050_REG_PWR_MGMT_2 (0x6C) // Power Management 1

/* --- DIGITAL MOTION PROCESSOR - READ/WRITE --------------------------------------------------- */
#define MPU_6050_REG_DMP_BANK_SEL                                                                  \
    (0x6D) // DMP Memory Bank Selection: activates a specific bank in the DMP
#define MPU_6050_REG_DMP_RW_PNT                                                                    \
    (0x6E) // DMP Memory Start Address: set read/write pointer to a specific start address in
           // specified DMP bank
#define MPU_6050_REG_DMP_REG                                                                       \
    (0x6F) // DMP Memory Read/Write: register in DMP from which to read or to which to write
#define MPU_6050_REG_DMP_CONFIG1 (0x70) // DMP Configuration 1
#define MPU_6050_REG_DMP_CONFIG2 (0x71) // DMP Configuration 2

/* --- FIFO BUFFER - READ ONLY ----------------------------------------------------------------- */
#define MPU_6050_REG_FIFO_COUNTH (0x72) // FIFO Count Register
#define MPU_6050_REG_FIFO_COUNTL (0x73) // FIFO Count Register

/* --- FIFO BUFFER - READ/WRITE ---------------------------------------------------------------- */
#define MPU_6050_REG_FIFO_R_W (0x74) // FIFO Read Write

/* --- MPU-6050 ID - READ ONLY ----------------------------------------------------------------- */
#define MPU_6050_REG_WHO_AM_I_MPU6050 (0x75) // Who Am I: Should return 0x68
#define MPU_6050_VALUE_WHO_AM_I       (0x68)

/* --- Public data type declarations ----------------------------------------------------------- */

/* --- Public variable declarations ------------------------------------------------------------ */

/* --- Public function declarations ------------------------------------------------------------ */

/* --- End of C++ guard ------------------------------------------------------------------------ */
#ifdef __cplusplus
}
#endif

#endif /* INC_MPU_6050_DRIVER_REGISTER_MAP_H  */

/* --- End of file ----------------------------------------------------------------------------- */
