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
#define DATA_OUT_CTL1 (4 * 16) // 16-bit: Data output control 1 register : configure DMP to output required data
#define DATA_OUT_CTL2 (4 * 16 + 2) // 16-bit: Data output control 2 register : configure the BM, accel/gyro/compass accuracy and gesture such as Pick-up
#define DATA_INTR_CTL (4 * 16 + 12) // 16-bit: Determines which sensors can generate interrupt according to bit map defined for DATA_OUT_CTL1
#define FIFO_WATERMARK (31 * 16 + 14) // 16-bit: DMP will send FIFO interrupt if FIFO count > FIFO watermark. FIFO watermark is set to 80% of actual FIFO size by default
// motion event control
#define MOTION_EVENT_CTL (4 * 16 + 14) // 16-bit: configure DMP for Android L and Invensense specific features
#define DATA_RDY_STATUS (8 * 16 + 10) // 16-bit: indicates to DMP which sensors are available
// batch mode
#define BM_BATCH_CNTR (27 * 16) // 32-bit: Batch counter
#define BM_BATCH_THLD (19 * 16 + 12) // 32-bit: Batch mode threshold
#define BM_BATCH_MASK (21 * 16 + 14) // 16-bit
// sensor output data rate: all 16-bit
#define ODR_ACCEL (11 * 16 + 14) // ODR_ACCEL Register for accel ODR
#define ODR_GYRO (11 * 16 + 10) // ODR_GYRO Register for gyro ODR
#define ODR_CPASS (11 * 16 + 6) // ODR_CPASS Register for compass ODR
#define ODR_ALS (11 * 16 + 2) // ODR_ALS Register for ALS ODR
#define ODR_QUAT6 (10 * 16 + 12) // ODR_QUAT6 Register for 6-axis quaternion ODR
#define ODR_QUAT9 (10 * 16 + 8) // ODR_QUAT9 Register for 9-axis quaternion ODR
#define ODR_PQUAT6 (10 * 16 + 4) // ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
#define ODR_GEOMAG (10 * 16 + 0) // ODR_GEOMAG Register for Geomag rv ODR
#define ODR_PRESSURE (11 * 16 + 12) // ODR_PRESSURE Register for pressure ODR
#define ODR_GYRO_CALIBR (11 * 16 + 8) // ODR_GYRO_CALIBR Register for calibrated gyro ODR
#define ODR_CPASS_CALIBR (11 * 16 + 4) // ODR_CPASS_CALIBR Register for calibrated compass ODR
// sensor output data rate counter: all 16-bit
#define ODR_CNTR_ACCEL (9 * 16 + 14) // ODR_CNTR_ACCEL Register for accel ODR counter
#define ODR_CNTR_GYRO (9 * 16 + 10) // ODR_CNTR_GYRO Register for gyro ODR counter
#define ODR_CNTR_CPASS (9 * 16 + 6) // ODR_CNTR_CPASS Register for compass ODR counter
#define ODR_CNTR_ALS (9 * 16 + 2) // ODR_CNTR_ALS Register for ALS ODR counter
#define ODR_CNTR_QUAT6 (8 * 16 + 12) // ODR_CNTR_QUAT6 Register for 6-axis quaternion ODR counter
#define ODR_CNTR_QUAT9 (8 * 16 + 8) // ODR_CNTR_QUAT9 Register for 9-axis quaternion ODR counter
#define ODR_CNTR_PQUAT6 (8 * 16 + 4) // ODR_CNTR_PQUAT6 Register for 6-axis pedometer quaternion ODR counter
#define ODR_CNTR_GEOMAG (8 * 16 + 0) // ODR_CNTR_GEOMAG Register for Geomag rv ODR counter
#define ODR_CNTR_PRESSURE (9 * 16 + 12) // ODR_CNTR_PRESSURE Register for pressure ODR counter
#define ODR_CNTR_GYRO_CALIBR (9 * 16 + 8) // ODR_CNTR_GYRO_CALIBR Register for calibrated gyro ODR counter
#define ODR_CNTR_CPASS_CALIBR (9 * 16 + 4) // ODR_CNTR_CPASS_CALIBR Register for calibrated compass ODR counter
// mounting matrix: all 32-bit
#define CPASS_MTX_00 (23 * 16) // Compass mount matrix and scale
#define CPASS_MTX_01 (23 * 16 + 4) // Compass mount matrix and scale
#define CPASS_MTX_02 (23 * 16 + 8) // Compass mount matrix and scale
#define CPASS_MTX_10 (23 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_11 (24 * 16) // Compass mount matrix and scale
#define CPASS_MTX_12 (24 * 16 + 4) // Compass mount matrix and scale
#define CPASS_MTX_20 (24 * 16 + 8) // Compass mount matrix and scale
#define CPASS_MTX_21 (24 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_22 (25 * 16) // Compass mount matrix and scale
// bias calibration: all 32-bit
// The biases are 32-bits in chip frame in hardware unit scaled by:
// 2^12 (FSR 4g) for accel, 2^15 for gyro, in uT scaled by 2^16 for compass.
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
// The DMP scales accel raw data internally to align 1g as 2^25.
// To do this and output hardware unit again as configured FSR, write 0x4000000 to ACC_SCALE DMP register, and write 0x40000 to ACC_SCALE2 DMP register.
#define ACC_SCALE (30 * 16 + 0) // 32-bit: Write accel scaling value for internal use
#define ACC_SCALE2 (79 * 16 + 4) // 32-bit: Write accel scaling down value
// pedometer
#define PEDSTD_STEPCTR (54 * 16) // 32-bit: step count (big endian)
#define PEDSTD_TIMECTR (60 * 16 + 4) // 32-bit: walk time (big endian)
// Activity Recognition (BAC)
#define BAC_RATE (48 * 16 + 10) // 16-bit
// parameters for accel calibration
#define ACCEL_CAL_RATE (94 * 16 + 4) // 16-bit
#define ACCEL_ALPHA_VAR (91 * 16) // 32-bit: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
#define ACCEL_A_VAR (92 * 16) // 32-bit: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
// parameters for compass calibration
#define CPASS_TIME_BUFFER (112 * 16 + 14) // 16-bit: running rate of compass. e.g. 70 (70Hz)
// gains
#define ACCEL_ONLY_GAIN (16 * 16 + 12) // 32-bit: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
#define GYRO_SF (19 * 16) // 32-bit: gyro scaling factor

enum DMP_ODR_Registers {
	DMP_ODR_Reg_Accel = ODR_ACCEL, // ODR_ACCEL Register for accel ODR
	DMP_ODR_Reg_Gyro = ODR_GYRO, // ODR_GYRO Register for gyro ODR
	DMP_ODR_Reg_Cpass = ODR_CPASS, // ODR_CPASS Register for compass ODR
	DMP_ODR_Reg_ALS = ODR_ALS, // ODR_ALS Register for ALS ODR
	DMP_ODR_Reg_Quat6 = ODR_QUAT6, // ODR_QUAT6 Register for 6-axis quaternion ODR
	DMP_ODR_Reg_Quat9 = ODR_QUAT9, // ODR_QUAT9 Register for 9-axis quaternion ODR
	DMP_ODR_Reg_PQuat6 = ODR_PQUAT6, // ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
	DMP_ODR_Reg_Geomag = ODR_GEOMAG, // ODR_GEOMAG Register for Geomag RV ODR
	DMP_ODR_Reg_Pressure = ODR_PRESSURE, // ODR_PRESSURE Register for pressure ODR
	DMP_ODR_Reg_Gyro_Calibr = ODR_GYRO_CALIBR, // ODR_GYRO_CALIBR Register for calibrated gyro ODR
	DMP_ODR_Reg_Cpass_Calibr = ODR_CPASS_CALIBR // ODR_CPASS_CALIBR Register for calibrated compass ODR
};

/** @brief Sensor identifier for control function
 */
enum inv_icm20948_sensor {
	INV_ICM20948_SENSOR_ACCELEROMETER = 0,
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

enum DMP_Data_Ready_Status_Register_Bits
{
	DMP_Data_ready_Gyro = 0x0001, // Gyro samples available
	DMP_Data_ready_Accel = 0x0002, // Accel samples available
	DMP_Data_ready_Secondary = 0x0008 // Secondary samples available
};

enum DMP_Data_Output_Control_1_Register_Bits
{
	DMP_Data_Output_Control_1_Step_Ind_0 = 0x0001, // Pedometer Step Indicator Bit 0
	DMP_Data_Output_Control_1_Step_Ind_1 = 0x0002, // Pedometer Step Indicator Bit 1
	DMP_Data_Output_Control_1_Step_Ind_2 = 0x0004, // Pedometer Step Indicator Bit 2
	DMP_Data_Output_Control_1_Header2 = 0x0008, // Header 2
	DMP_Data_Output_Control_1_Step_Detector = 0x0010, // Pedometer Step Detector
	DMP_Data_Output_Control_1_Compass_Calibr = 0x0020, // 32-bit calibrated compass
	DMP_Data_Output_Control_1_Gyro_Calibr = 0x0040, // 32-bit calibrated gyro
	DMP_Data_Output_Control_1_Pressure = 0x0080, // 16-bit Pressure
	DMP_Data_Output_Control_1_Geomag = 0x0100, // 32-bit Geomag rv + heading accuracy
	DMP_Data_Output_Control_1_PQuat6 = 0x0200, // 16-bit pedometer quaternion
	DMP_Data_Output_Control_1_Quat9 = 0x0400, // 32-bit 9-axis quaternion + heading accuracy
	DMP_Data_Output_Control_1_Quat6 = 0x0800, // 32-bit 6-axis quaternion
	DMP_Data_Output_Control_1_ALS = 0x1000, // 16-bit ALS
	DMP_Data_Output_Control_1_Compass = 0x2000, // 16-bit compass
	DMP_Data_Output_Control_1_Gyro = 0x4000, // 16-bit gyro
	DMP_Data_Output_Control_1_Accel = 0x8000 // 16-bit accel
};

enum DMP_Data_Output_Control_2_Register_Bits
{
	DMP_Data_Output_Control_2_Secondary_On_Off = 0x0040,
	DMP_Data_Output_Control_2_Activity_Recognition_BAC = 0x0080,
	DMP_Data_Output_Control_2_Batch_Mode_Enable = 0x0100,
	DMP_Data_Output_Control_2_Pickup = 0x0400,
	DMP_Data_Output_Control_2_Fsync_Detection = 0x0800,
	DMP_Data_Output_Control_2_Compass_Accuracy = 0x1000,
	DMP_Data_Output_Control_2_Gyro_Accuracy = 0x2000,
	DMP_Data_Output_Control_2_Accel_Accuracy = 0x4000
};

enum DMP_Motion_Event_Control_Register_Bits
{
	DMP_Motion_Event_Control_Activity_Recog_Pedom_Accel = 0x0002, // Activity Recognition / Pedometer accel only
	DMP_Motion_Event_Control_Geomag = 0x0008, // Geomag rv
	DMP_Motion_Event_Control_Pickup = 0x0010,
	DMP_Motion_Event_Control_9axis = 0x0040,
	DMP_Motion_Event_Control_Compass_Calibr = 0x0080,
	DMP_Motion_Event_Control_Gyro_Calibr = 0x0100,
	DMP_Motion_Event_Control_Accel_Calibr = 0x0200,
	DMP_Motion_Event_Control_Significant_Motion_Det = 0x0800,
	DMP_Motion_Event_Control_Tilt_Interrupt = 0x1000,
	DMP_Motion_Event_Control_Pedometer_Interrupt = 0x2000,
	DMP_Motion_Event_Control_Activity_Recog_Pedom = 0x4000
};

enum DMP_Header_Bitmap
{
	DMP_header_bitmap_Header2 = 0x0008,
	DMP_header_bitmap_Step_Detector = 0x0010,
	DMP_header_bitmap_Compass_Calibr = 0x0020,
	DMP_header_bitmap_Gyro_Calibr = 0x0040,
	DMP_header_bitmap_Pressure = 0x0080,
	DMP_header_bitmap_Geomag = 0x0100,
	DMP_header_bitmap_PQuat6 = 0x0200,
	DMP_header_bitmap_Quat9 = 0x0400,
	DMP_header_bitmap_Quat6 = 0x0800,
	DMP_header_bitmap_ALS = 0x1000,
	DMP_header_bitmap_Compass = 0x2000,
	DMP_header_bitmap_Gyro = 0x4000,
	DMP_header_bitmap_Accel = 0x8000
};

enum DMP_Header2_Bitmap
{
	DMP_header2_bitmap_Secondary_On_Off = 0x0040,
	DMP_header2_bitmap_Activity_Recog = 0x0080,
	DMP_header2_bitmap_Pickup = 0x0400,
	DMP_header2_bitmap_Fsync = 0x0800,
	DMP_header2_bitmap_Compass_Accuracy = 0x1000,
	DMP_header2_bitmap_Gyro_Accuracy = 0x2000,
	DMP_header2_bitmap_Accel_Accuracy = 0x4000
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

typedef struct // DMP Activity Recognition data
{
		uint8_t Drive : 1;
		uint8_t Walk : 1;
		uint8_t Run : 1;
		uint8_t Bike : 1;
		uint8_t Tilt : 1;
		uint8_t Still : 1;
		uint8_t reserved : 2;
} icm_20948_DMP_Activity_t;

typedef struct // DMP Secondary On/Off data
{
		uint16_t Gyro_Off : 1;
		uint16_t Gyro_On : 1;
		uint16_t Compass_Off : 1;
		uint16_t Compass_On : 1;
		uint16_t Proximity_Off : 1;
		uint16_t Proximity_On : 1;
		uint16_t reserved : 10;
} icm_20948_DMP_Secondary_On_Off_t;

#define icm_20948_DMP_Header_Bytes 2
#define icm_20948_DMP_Header2_Bytes 2
#define icm_20948_DMP_Raw_Accel_Bytes 6
#define icm_20948_DMP_Raw_Gyro_Bytes 6
#define icm_20948_DMP_Compass_Bytes 6
#define icm_20948_DMP_ALS_Bytes 8
#define icm_20948_DMP_Quat6_Bytes 12
#define icm_20948_DMP_Quat9_Bytes 14
#define icm_20948_DMP_PQuat6_Bytes 6
#define icm_20948_DMP_Geomag_Bytes 14
#define icm_20948_DMP_Pressure_Bytes 6
#define icm_20948_DMP_Gyro_Calibr_Bytes 12
#define icm_20948_DMP_Compass_Calibr_Bytes 12
#define icm_20948_DMP_Step_Detector_Bytes 4
#define icm_20948_DMP_Accel_Accuracy_Bytes 2
#define icm_20948_DMP_Gyro_Accuracy_Bytes 2
#define icm_20948_DMP_Compass_Accuracy_Bytes 2
#define icm_20948_DMP_Fsync_Detection_Bytes 2
#define icm_20948_DMP_Pickup_Bytes 2
#define icm_20948_DMP_Activity_Recognition_Bytes 6
#define icm_20948_DMP_Secondary_On_Off_Bytes 2

// ICM-20948 data is big-endian. We need to make it little-endian when writing into icm_20948_DMP_data_t
const int DMP_Quat9_Byte_Ordering[icm_20948_DMP_Quat9_Bytes] =
{
	3,2,1,0,7,6,5,4,11,10,9,8,13,12 // Also used for Geomag
};
const int DMP_Quat6_Byte_Ordering[icm_20948_DMP_Quat6_Bytes] =
{
	3,2,1,0,7,6,5,4,11,10,9,8 // Also used for Gyro_Calibr, Compass_Calibr
};
const int DMP_PQuat6_Byte_Ordering[icm_20948_DMP_PQuat6_Bytes] =
{
	1,0,3,2,5,4 // Also used for Raw_Accel, Raw_Gyro, Compass
};
const int DMP_Activity_Recognition_Byte_Ordering[icm_20948_DMP_Activity_Recognition_Bytes] =
{
	0,1,5,4,3,2
};
const int DMP_Secondary_On_Off_Byte_Ordering[icm_20948_DMP_Secondary_On_Off_Bytes] =
{
	1,0
};

typedef struct
{
	uint16_t header;
	uint16_t header2;
	union
	{
		uint8_t Bytes[icm_20948_DMP_Raw_Accel_Bytes];
		struct
		{
			int16_t X;
			int16_t Y;
			int16_t Z;
		} Data;
	} Raw_Accel;
	union
	{
		uint8_t Bytes[icm_20948_DMP_Raw_Gyro_Bytes];
		struct
		{
			int16_t X;
			int16_t Y;
			int16_t Z;
		} Data;
	} Raw_Gyro;
	union
	{
		uint8_t Bytes[icm_20948_DMP_Compass_Bytes];
		struct
		{
			int16_t X;
			int16_t Y;
			int16_t Z;
		} Data;
	} Compass;
	uint8_t ALS[icm_20948_DMP_ALS_Bytes]; // Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
	// The 6-Axis and 9-axis Quaternion outputs each consist of 12 bytes of data.
	// These 12 bytes in turn consists of three 4-byte elements.
	// 9-axis quaternion data and Geomag rv is always followed by 2-bytes of heading accuracy, hence the size of Quat9 and Geomag data size in the FIFO is 14 bytes.
	// Quaternion data for both cases is cumulative/integrated values.
	// For a given quaternion Q, the ordering of its elements is {Q1, Q2, Q3}.
	// Each element is represented using Big Endian byte order.
	// Q0 value is computed from this equation: Q20 + Q21 + Q22 + Q23 = 1.
	// In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
	// The quaternion data is scaled by 2^30.
	union
	{
		uint8_t Bytes[icm_20948_DMP_Quat6_Bytes];
		struct
		{
			int32_t Q1;
			int32_t Q2;
			int32_t Q3;
		} Data;
	} Quat6;
	union
	{
		uint8_t Bytes[icm_20948_DMP_Quat9_Bytes];
		struct
		{
			int32_t Q1;
			int32_t Q2;
			int32_t Q3;
			int16_t Accuracy;
		} Data;
	} Quat9;
	union
	{
		uint8_t Bytes[icm_20948_DMP_PQuat6_Bytes];
		struct
		{
			int16_t Q1;
			int16_t Q2;
			int16_t Q3;
		} Data;
	} PQuat6;
	union
	{
		uint8_t Bytes[icm_20948_DMP_Geomag_Bytes];
		struct
		{
			int32_t Q1;
			int32_t Q2;
			int32_t Q3;
			int16_t Accuracy;
		} Data;
	} Geomag;
	uint8_t Pressure[6]; // Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
	union
	{
		uint8_t Bytes[icm_20948_DMP_Gyro_Calibr_Bytes];
		struct
		{
			int32_t X;
			int32_t Y;
			int32_t Z;
		} Data;
	} Gyro_Calibr; // Hardware unit scaled by 2^15
	union
	{
		uint8_t Bytes[icm_20948_DMP_Compass_Calibr_Bytes];
		struct
		{
			int32_t X;
			int32_t Y;
			int32_t Z;
		} Data;
	} Compass_Calibr; // The unit is uT scaled by 2^16
	uint32_t Pedometer_Timestamp; // Timestamp as DMP cycle
	uint16_t Accel_Accuracy; // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
	uint16_t Gyro_Accuracy; // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
	uint16_t Compass_Accuracy; // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
	uint16_t Fsync_Delay_Time; // The data is delay time between Fsync event and the 1st ODR event after Fsync event.
	uint16_t Pickup; // The value “2” indicates pick up is detected.
	// Activity Recognition data
	// The data include Start and End states, and timestamp as DMP cycle.
	// Byte [0]: State-Start, Byte [1]: State-End, Byte [5:2]: timestamp.
	// The states are expressed as below.
	// Drive: 0x01
	// Walk: 0x02
	// Run: 0x04
	// Bike: 0x08
	// Tilt: 0x10
	// Still: 0x20
	union
	{
		uint8_t Bytes[icm_20948_DMP_Activity_Recognition_Bytes];
		struct
		{
			icm_20948_DMP_Activity_t State_Start;
			icm_20948_DMP_Activity_t State_End;
			uint32_t Timestamp;
		} Data;
	} Activity_Recognition;
	// Secondary On/Off data
	// BAC algorithm requires sensors on/off through FIFO data to detect activities effectively and save power.
	// The driver is expected to control sensors accordingly.
	// The data indicates which sensor and on or off as below.
	// Gyro Off: 0x01
	// Gyro On: 0x02
	// Compass Off: 0x04
	// Compass On: 0x08
	// Proximity Off: 0x10
	// Proximity On: 0x20
	union
	{
		uint8_t Bytes[icm_20948_DMP_Secondary_On_Off_Bytes];
		icm_20948_DMP_Secondary_On_Off_t Sensors;
	} Secondary_On_Off;
} icm_20948_DMP_data_t;

#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /* _ICM_20948_REGISTERS_H_ */
