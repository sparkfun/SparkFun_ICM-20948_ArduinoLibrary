/*

This file contains a useful c translation of the DMP register map

*/

#ifndef _ICM_20948_DMP_H_
#define _ICM_20948_DMP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

#define DMP_START_ADDRESS   ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE   256
#define DMP_LOAD_START      0x90

#define DMP_CODE_SIZE 14290

// AGB0_REG_DMP_INT_STATUS bit definitions
#define BIT_WAKE_ON_MOTION_INT          0x08
#define BIT_MSG_DMP_INT                 0x0002
#define BIT_MSG_DMP_INT_0               0x0100  // CI Command

#define BIT_MSG_DMP_INT_2               0x0200  // CIM Command - SMD
#define BIT_MSG_DMP_INT_3               0x0400  // CIM Command - Pedometer

#define BIT_MSG_DMP_INT_4               0x1000  // CIM Command - Pedometer binning
#define BIT_MSG_DMP_INT_5               0x2000  // CIM Command - Bring To See Gesture
#define BIT_MSG_DMP_INT_6               0x4000  // CIM Command - Look To See Gesture

// Appendix I: DMP register addresses

// data output control
#define DATA_OUT_CTL1 (4 * 16)
#define DATA_OUT_CTL2 (4 * 16 + 2)
#define DATA_INTR_CTL (4 * 16 + 12)
#define FIFO_WATERMARK (31 * 16 + 14)
// motion event control
#define MOTION_EVENT_CTL (4 * 16 + 14)
// indicates to DMP which sensors are available
#define DATA_RDY_STATUS (8 * 16 + 10)
// batch mode
#define BM_BATCH_CNTR (27 * 16)
#define BM_BATCH_THLD (19 * 16 + 12)
#define BM_BATCH_MASK (21 * 16 + 14)
// sensor output data rate
#define ODR_ACCEL (11 * 16 + 14)
#define ODR_GYRO (11 * 16 + 10)
#define ODR_CPASS (11 * 16 + 6)
#define ODR_ALS (11 * 16 + 2)
#define ODR_QUAT6 (10 * 16 + 12)
#define ODR_QUAT9 (10 * 16 + 8)
#define ODR_PQUAT6 (10 * 16 + 4)
#define ODR_GEOMAG (10 * 16 + 0)
#define ODR_PRESSURE (11 * 16 + 12)
#define ODR_GYRO_CALIBR (11 * 16 + 8)
#define ODR_CPASS_CALIBR (11 * 16 + 4)
// sensor output data rate counter
#define ODR_CNTR_ACCEL (9 * 16 + 14)
#define ODR_CNTR_GYRO (9 * 16 + 10)
#define ODR_CNTR_CPASS (9 * 16 + 6)
#define ODR_CNTR_ALS (9 * 16 + 2)
#define ODR_CNTR_QUAT6 (8 * 16 + 12)
#define ODR_CNTR_QUAT9 (8 * 16 + 8)
#define ODR_CNTR_PQUAT6 (8 * 16 + 4)
#define ODR_CNTR_GEOMAG (8 * 16 + 0)
#define ODR_CNTR_PRESSURE (9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR (9 * 16 + 8)
#define ODR_CNTR_CPASS_CALIBR (9 * 16 + 4)
// mounting matrix
#define CPASS_MTX_00 (23 * 16)
#define CPASS_MTX_01 (23 * 16 + 4)
#define CPASS_MTX_02 (23 * 16 + 8)
#define CPASS_MTX_10 (23 * 16 + 12)
#define CPASS_MTX_11 (24 * 16)
#define CPASS_MTX_12 (24 * 16 + 4)
#define CPASS_MTX_20 (24 * 16 + 8)
#define CPASS_MTX_21 (24 * 16 + 12)
#define CPASS_MTX_22 (25 * 16)
// bias calibration
#define GYRO_BIAS_X (139 * 16 + 4)
#define GYRO_BIAS_Y (139 * 16 + 8)
#define GYRO_BIAS_Z (139 * 16 + 12)
#define ACCEL_BIAS_X (110 * 16 + 4)
#define ACCEL_BIAS_Y (110 * 16 + 8)
#define ACCEL_BIAS_Z (110 * 16 + 12)
#define CPASS_BIAS_X (126 * 16 + 4)
#define CPASS_BIAS_Y (126 * 16 + 8)
#define CPASS_BIAS_Z (126 * 16 + 12)
// Accel FSR
#define ACC_SCALE (30 * 16 + 0)
#define ACC_SCALE2 (79 * 16 + 4)
// pedometer
#define PEDSTD_STEPCTR (54 * 16)
#define PEDSTD_TIMECTR (60 * 16 + 4)
// Activity Recognition
#define BAC_RATE (48 * 16 + 10)
// parameters for accel calibration
#define ACCEL_CAL_RATE (94 * 16 + 4)
#define ACCEL_ALPHA_VAR (91 * 16)
#define ACCEL_A_VAR (92 * 16)
// parameters for compass calibration
#define CPASS_TIME_BUFFER (112 * 16 + 14)
// gains
#define ACCEL_ONLY_GAIN (16 * 16 + 12)
#define GYRO_SF (19 * 16)

#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /* _ICM_20948_REGISTERS_H_ */
