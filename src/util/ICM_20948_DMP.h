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


/** @brief Sensor identifier for control function
 */
enum inv_icm20948_sensor {
	INV_ICM20948_SENSOR_ACCELEROMETER,
	INV_ICM20948_SENSOR_GYROSCOPE,
	INV_ICM20948_SENSOR_RAW_ACCELEROMETER,
	INV_ICM20948_SENSOR_RAW_GYROSCOPE,
	INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
	INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED,
	INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON,
	INV_ICM20948_SENSOR_STEP_DETECTOR,
	INV_ICM20948_SENSOR_STEP_COUNTER,
	INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR,
	INV_ICM20948_SENSOR_ROTATION_VECTOR,
	INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
	INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD,
	INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
	INV_ICM20948_SENSOR_FLIP_PICKUP,
	INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR,
	INV_ICM20948_SENSOR_GRAVITY,
	INV_ICM20948_SENSOR_LINEAR_ACCELERATION,
	INV_ICM20948_SENSOR_ORIENTATION,
	INV_ICM20948_SENSOR_B2S,
	INV_ICM20948_SENSOR_MAX,
};

/* enum for android sensor*/
enum ANDROID_SENSORS {
	ANDROID_SENSOR_META_DATA = 0,
	ANDROID_SENSOR_ACCELEROMETER,
	ANDROID_SENSOR_GEOMAGNETIC_FIELD,
	ANDROID_SENSOR_ORIENTATION,
	ANDROID_SENSOR_GYROSCOPE,
	ANDROID_SENSOR_LIGHT,
	ANDROID_SENSOR_PRESSURE,
	ANDROID_SENSOR_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_PROXIMITY,
	ANDROID_SENSOR_GRAVITY,
	ANDROID_SENSOR_LINEAR_ACCELERATION,
	ANDROID_SENSOR_ROTATION_VECTOR,
	ANDROID_SENSOR_HUMIDITY,
	ANDROID_SENSOR_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
	ANDROID_SENSOR_STEP_DETECTOR,
	ANDROID_SENSOR_STEP_COUNTER,
	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_HEART_RATE,
	ANDROID_SENSOR_PROXIMITY,

	ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
	ANDROID_SENSOR_WAKEUP_ORIENTATION,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE,
	ANDROID_SENSOR_WAKEUP_LIGHT,
	ANDROID_SENSOR_WAKEUP_PRESSURE,
	ANDROID_SENSOR_WAKEUP_GRAVITY,
	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
	ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
	ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_HEART_RATE,
	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
	ANDROID_SENSOR_RAW_ACCELEROMETER,
	ANDROID_SENSOR_RAW_GYROSCOPE,
	ANDROID_SENSOR_NUM_MAX,

  ANDROID_SENSOR_B2S,
	ANDROID_SENSOR_FLIP_PICKUP,
	ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,
	ANDROID_SENSOR_SCREEN_ROTATION,
	SELF_TEST,
	SETUP,
	GENERAL_SENSORS_MAX
};

enum DMP_Data_Output_Control_1_Register_Bits
{
	DMP_Data_Output_Control_1_Compass_Calibr = 0x0020,
	DMP_Data_Output_Control_1_Gyro_Calibr = 0x0040,
	DMP_Data_Output_Control_1_Pressure = 0x0080,
	DMP_Data_Output_Control_1_Geomag = 0x0100,
	DMP_Data_Output_Control_1_PQuat6 = 0x0200,
	DMP_Data_Output_Control_1_Quat9 = 0x0400,
	DMP_Data_Output_Control_1_Quat6 = 0x0800,
	DMP_Data_Output_Control_1_ALS = 0x1000,
	DMP_Data_Output_Control_1_Compass = 0x2000,
	DMP_Data_Output_Control_1_Gyro = 0x4000,
	DMP_Data_Output_Control_1_Accel = 0x8000
};

const uint16_t inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX]=
{
	// Data output control 1 register bit definition
	// 16-bit accel                                0x8000
	// 16-bit gyro                                 0x4000
	// 16-bit compass                              0x2000
	// 16-bit ALS                                  0x1000
	// 32-bit 6-axis quaternion                    0x0800
	// 32-bit 9-axis quaternion + heading accuracy 0x0400
	// 16-bit pedometer quaternion                 0x0200
	// 32-bit Geomag rv + heading accuracy         0x0100
	// 16-bit Pressure                             0x0080
	// 32-bit calibrated gyro                      0x0040
	// 32-bit calibrated compass                   0x0020
	// Pedometer Step Detector                     0x0010
	// Header 2                                    0x0008
	// Pedometer Step Indicator Bit 2              0x0004
	// Pedometer Step Indicator Bit 1              0x0002
	// Pedometer Step Indicator Bit 0              0x0001
	// Unsupported Sensors are 0xFFFF
	0xFFFF, // Meta Data
	0x8008, // Accelerometer
	0x0028, // Magnetic Field
	0x0408, // Orientation
	0x4048, // Gyroscope
	0x1008, // Light
	0x0088, // Pressure
	0xFFFF, // Temperature
	0xFFFF, // Proximity <----------- fixme
	0x0808, // Gravity
	0x8808, // Linear Acceleration
	0x0408, // Rotation Vector
	0xFFFF, // Humidity
	0xFFFF, // Ambient Temperature
	0x2008, // Magnetic Field Uncalibrated
	0x0808, // Game Rotation Vector
	0x4008, // Gyroscope Uncalibrated
	0, // Significant Motion
	0x0018, // Step Detector
	0x0010, // Step Counter <----------- fixme
	0x0108, // Geomagnetic Rotation Vector
	0xFFFF, //ANDROID_SENSOR_HEART_RATE,
	0xFFFF, //ANDROID_SENSOR_PROXIMITY,

	0x8008, // ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
	0x0028, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
	0x0408, // ANDROID_SENSOR_WAKEUP_ORIENTATION,
	0x4048, // ANDROID_SENSOR_WAKEUP_GYROSCOPE,
	0x1008, // ANDROID_SENSOR_WAKEUP_LIGHT,
	0x0088, // ANDROID_SENSOR_WAKEUP_PRESSURE,
	0x0808, // ANDROID_SENSOR_WAKEUP_GRAVITY,
	0x8808, // ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
	0x0408, // ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
	0xFFFF,		// ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
	0xFFFF,		// ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
	0x2008, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
	0x0808, // ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
	0x4008, // ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
	0x0018, // ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
	0x0010, // ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
	0x0108, // ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
	0xFFFF,		// ANDROID_SENSOR_WAKEUP_HEART_RATE,
	0,		// ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
	0x8008, // Raw Acc
	0x4048, // Raw Gyr
};



#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /* _ICM_20948_REGISTERS_H_ */
