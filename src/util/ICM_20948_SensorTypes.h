/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/** @defgroup SensorTypes Sensor types
 *  @brief    Sensor related types definitions
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_SENSOR_TYPES_H_
#define _INV_SENSOR_TYPES_H_

//#include "Invn/InvExport.h" // Needed for DLL INV_EXPORT

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ICM_20948_InvBool.h" // Defines true and false

/** @brief Sensor type identifier definition
 */
enum inv_sensor_type {
	INV_SENSOR_TYPE_RESERVED                     = 0 ,  /**< Reserved ID: do not use */
	INV_SENSOR_TYPE_ACCELEROMETER                = 1 ,  /**< Accelerometer */
	INV_SENSOR_TYPE_MAGNETOMETER                 = 2 ,  /**< Magnetic field */
	INV_SENSOR_TYPE_ORIENTATION                  = 3 ,  /**< Deprecated orientation */
	INV_SENSOR_TYPE_GYROSCOPE                    = 4 ,  /**< Gyroscope */
	INV_SENSOR_TYPE_LIGHT                        = 5 ,  /**< Ambient light sensor */
	INV_SENSOR_TYPE_PRESSURE                     = 6 ,  /**< Barometer */
	INV_SENSOR_TYPE_TEMPERATURE                  = 7 ,  /**< Temperature */
	INV_SENSOR_TYPE_PROXIMITY                    = 8 ,  /**< Proximity */
	INV_SENSOR_TYPE_GRAVITY                      = 9 ,  /**< Gravity */
	INV_SENSOR_TYPE_LINEAR_ACCELERATION          = 10,  /**< Linear acceleration */
	INV_SENSOR_TYPE_ROTATION_VECTOR              = 11,  /**< Rotation vector */
	INV_SENSOR_TYPE_HUMIDITY                     = 12,  /**< Relative humidity */
	INV_SENSOR_TYPE_AMBIENT_TEMPERATURE          = 13,  /**< Ambient temperature */
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER           = 14,  /**< Uncalibrated magnetic field */
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR         = 15,  /**< Game rotation vector */
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE              = 16,  /**< Uncalibrated gyroscope */
	INV_SENSOR_TYPE_SMD                          = 17,  /**< Significant motion detection */
	INV_SENSOR_TYPE_STEP_DETECTOR                = 18,  /**< Step detector */
	INV_SENSOR_TYPE_STEP_COUNTER                 = 19,  /**< Step counter */
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR       = 20,  /**< Geomagnetic rotation vector */
	INV_SENSOR_TYPE_HEART_RATE                   = 21,  /**< Heart rate */
	INV_SENSOR_TYPE_TILT_DETECTOR                = 22,  /**< Tilt detector */
	INV_SENSOR_TYPE_WAKE_GESTURE                 = 23,  /**< Wake-up gesture  */
	INV_SENSOR_TYPE_GLANCE_GESTURE               = 24,  /**< Glance gesture  */
	INV_SENSOR_TYPE_PICK_UP_GESTURE              = 25,  /**< Pick-up gesture */
	INV_SENSOR_TYPE_BAC                          = 26,  /**< Basic Activity Classifier */
	INV_SENSOR_TYPE_PDR                          = 27,  /**< Pedestrian Dead Reckoning */
	INV_SENSOR_TYPE_B2S                          = 28,  /**< Bring to see */
	INV_SENSOR_TYPE_3AXIS                        = 29,  /**< 3 Axis sensor */
	INV_SENSOR_TYPE_EIS                          = 30,  /**< Electronic Image Stabilization */
	INV_SENSOR_TYPE_OIS                          = 31,  /**< Optical Image Stabilization */
	INV_SENSOR_TYPE_RAW_ACCELEROMETER            = 32,  /**< Raw accelerometer */
	INV_SENSOR_TYPE_RAW_GYROSCOPE                = 33,  /**< Raw gyroscope */
	INV_SENSOR_TYPE_RAW_MAGNETOMETER             = 34,  /**< Raw magnetometer */
	INV_SENSOR_TYPE_RAW_TEMPERATURE              = 35,  /**< Raw temperature */
	INV_SENSOR_TYPE_CUSTOM_PRESSURE              = 36,  /**< Custom Pressure Sensor */
	INV_SENSOR_TYPE_MIC                          = 37,  /**< Stream audio from microphone */
	INV_SENSOR_TYPE_TSIMU                        = 38,  /**< TS-IMU */
	INV_SENSOR_TYPE_RAW_PPG                      = 39,  /**< Raw Photoplethysmogram */
	INV_SENSOR_TYPE_HRV                          = 40,  /**< Heart rate variability */
	INV_SENSOR_TYPE_SLEEP_ANALYSIS               = 41,  /**< Sleep analysis */
	INV_SENSOR_TYPE_BAC_EXTENDED                 = 42,  /**< Basic Activity Classifier Extended */
	INV_SENSOR_TYPE_BAC_STATISTICS               = 43,  /**< Basic Activity Classifier Statistics */
	INV_SENSOR_TYPE_FLOOR_CLIMB_COUNTER          = 44,  /**< Floor Climbed Counter */
	INV_SENSOR_TYPE_ENERGY_EXPENDITURE           = 45,  /**< Energy Expenditure */
	INV_SENSOR_TYPE_DISTANCE                     = 46,  /**< Distance */
	INV_SENSOR_TYPE_SHAKE                        = 47,  /**< Shake Gesture */
	INV_SENSOR_TYPE_DOUBLE_TAP                   = 48,  /**< Double Tap */
	INV_SENSOR_TYPE_CUSTOM0,                            /**< Custom sensor ID 0 */
	INV_SENSOR_TYPE_CUSTOM1,                            /**< Custom sensor ID 1 */
	INV_SENSOR_TYPE_CUSTOM2,                            /**< Custom sensor ID 2 */
	INV_SENSOR_TYPE_CUSTOM3,                            /**< Custom sensor ID 3 */
	INV_SENSOR_TYPE_CUSTOM4,                            /**< Custom sensor ID 4 */
	INV_SENSOR_TYPE_CUSTOM5,                            /**< Custom sensor ID 5 */
	INV_SENSOR_TYPE_CUSTOM6,                            /**< Custom sensor ID 6 */
	INV_SENSOR_TYPE_CUSTOM7,                            /**< Custom sensor ID 7 */
	INV_SENSOR_TYPE_WOM,                                /**< Wake-up on motion */
	INV_SENSOR_TYPE_SEDENTARY_REMIND,                   /**< Sedentary Remind */
	INV_SENSOR_TYPE_DATA_ENCRYPTION,                    /**< Data Encryption */
	INV_SENSOR_TYPE_FSYNC_EVENT,                        /**< FSYNC event */
	INV_SENSOR_TYPE_HIGH_RATE_GYRO,                     /**< High Rate Gyro */
	INV_SENSOR_TYPE_CUSTOM_BSCD,                        /**< Custom BAC StepCounter Calorie counter and Distance counter */
	INV_SENSOR_TYPE_HRM_LOGGER,                         /**< HRM ouput for logger */
	/* Starting from there, the SensorID is coded with more than 6bits so check that communication protocol is adequate */
	INV_SENSOR_TYPE_PREDICTIVE_QUATERNION,              /**< Predictive Quaternion */
	INV_SENSOR_TYPE_MAX                                 /**< sentinel value for sensor type */
};

#define INV_SENSOR_TYPE_CUSTOM_BASE    INV_SENSOR_TYPE_CUSTOM0
#define INV_SENSOR_TYPE_CUSTOM_END     (INV_SENSOR_TYPE_CUSTOM7+1)

#define INV_SENSOR_TYPE_META_DATA       INV_SENSOR_TYPE_RESERVED        /**< @deprecated */
#define INV_SENSOR_TYPE_GYROMETER       INV_SENSOR_TYPE_GYROSCOPE       /**< @deprecated */
#define INV_SENSOR_TYPE_UNCAL_GYROMETER INV_SENSOR_TYPE_UNCAL_GYROSCOPE /**< @deprecated */
#define INV_SENSOR_TYPE_ENERGY_EXPANDITURE INV_SENSOR_TYPE_ENERGY_EXPENDITURE /**< @deprecated */

/** @brief Helper flag to indicate if sensor is a Wale-Up sensor
 */
#define INV_SENSOR_TYPE_WU_FLAG        (unsigned int)(0x80000000)

/** @brief Sensor status definition
 */
enum inv_sensor_status
{
	INV_SENSOR_STATUS_DATA_UPDATED      = 0,    /**< new sensor data */
	INV_SENSOR_STATUS_STATE_CHANGED     = 1,    /**< dummy sensor data indicating
                                                     to a change in sensor state */
	INV_SENSOR_STATUS_FLUSH_COMPLETE    = 2,    /**< dummy sensor data indicating
                                                     a end of batch after a manual flush */
	INV_SENSOR_STATUS_POLLED_DATA       = 3,    /**< sensor data value after manual request */
};

/** @brief Event definition for BAC sensor
 */
enum inv_sensor_bac_event {
	INV_SENSOR_BAC_EVENT_ACT_UNKNOWN             =  0,
	INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN    =  1,
	INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END      = -1,
	INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN 	     =  2,
	INV_SENSOR_BAC_EVENT_ACT_WALKING_END         = -2,
	INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN       =  3,
	INV_SENSOR_BAC_EVENT_ACT_RUNNING_END         = -3,
	INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN    =  4,
	INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END      = -4,
	INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN          =  5,
	INV_SENSOR_BAC_EVENT_ACT_TILT_END            = -5,
	INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN         =  6,
	INV_SENSOR_BAC_EVENT_ACT_STILL_END           = -6,
};

/** @brief Event definition for BAC Ext sensor
 */
enum inv_sensor_bacext_event {
	INV_SENSOR_BACEXT_EVENT_ACT_UNKNOWN                 =  0,
	INV_SENSOR_BACEXT_EVENT_ACT_WALKING_START           =  1,
	INV_SENSOR_BACEXT_EVENT_ACT_WALKING_END             = -1,
	INV_SENSOR_BACEXT_EVENT_ACT_RUNNING_START           =  2,
	INV_SENSOR_BACEXT_EVENT_ACT_RUNNING_END             = -2,
	INV_SENSOR_BACEXT_EVENT_ACT_ON_BICYCLE_START        =  3,
	INV_SENSOR_BACEXT_EVENT_ACT_ON_BICYCLE_END          = -3,
	INV_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_SIT_START    =  4,
	INV_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_SIT_END      = -4,
	INV_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_STAND_START  =  5,
	INV_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_STAND_END    = -5,
	INV_SENSOR_BACEXT_EVENT_ACT_STILL_SIT_START         =  6,
	INV_SENSOR_BACEXT_EVENT_ACT_STILL_SIT_END           = -6,
	INV_SENSOR_BACEXT_EVENT_ACT_STILL_STAND_START       =  7,
	INV_SENSOR_BACEXT_EVENT_ACT_STILL_STAND_END         = -7
};

/** @brief Maximum size of an event data
 */
#define INV_SENSOR_EVENT_DATA_SIZE      64

/** @brief For backward compatibility only - do not use
 */
#define IVN_SENSOR_EVENT_DATA_SIZE INV_SENSOR_EVENT_DATA_SIZE

/** @brief Sensor event definition
 */
typedef struct inv_sensor_event
{
	unsigned int         sensor;           /**< sensor type */
	int                  status;           /**< sensor data status as of
	                                            enum inv_sensor_status */
	uint64_t             timestamp;        /**< sensor data timestamp in us */
	union {
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} acc;                             /**< 3d accelerometer data in g */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} mag;                             /**< 3d magnetometer data in uT */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} gyr;                             /**< 3d gyroscope data in deg/s */
		struct {
			float        quat[4];          /**< w,x,y,z quaternion data */
			float        accuracy;         /**< heading accuracy in deg */
			uint8_t      accuracy_flag;    /**< accuracy flag specific for GRV*/
		} quaternion;                      /**< quaternion data */
		struct {
			float        x,y,z;            /**< x,y,z angles in deg as defined by Google Orientation sensor */
			uint8_t      accuracy_flag;    /**< heading accuracy in deg */
		} orientation;                     /**< orientation data */
		struct {
			float        bpm;              /**< beat per minute */
			uint8_t      confidence;       /**< confidence level */
			uint8_t      sqi;              /**< signal quality as seen by the the HRM engine */
		} hrm;                             /**< heart rate monitor data */                            /**< heart rate monitor data */
		struct {
			int32_t      acc[3];           /**< accel data used by hrm algorithm */
			int32_t      gyr[3];           /**< gyro data used by hrm algorithm */
			uint32_t     ppg_value;        /**< ppg value read from HRM sensor */
			float        ppm;              /**< beat per minute */
			uint8_t      confidence;       /**< confidence level */
			uint8_t      sqi;              /**< signal quality as seen by the the HRM engine */
			uint8_t      touch_status;     /**< touch status, detected or not by the PPG */
			uint8_t      gyrEnable;        /**< 1 gyro is enable else 0 */
		} hrmlogger;                       /**< heart rate monitor logger data */
		struct {
			uint8_t      rr_count;
			int16_t      rr_interval[4];   /**< beat-to-beat(RR) interval */
		} hrv;                             /**< heart rate variability data */
		struct {
			uint32_t     ppg_value;        /**< ppg value read from HRM sensor */
			uint8_t      touch_status;     /**< touch status, detected or not */
		} rawppg;                          /**< raw heart rate monitor data */
		struct {
			uint8_t      sleep_phase;      /**< state of sleep phases: 0 not defined, 1 restless sleep, 2 light sleep, 3 deep sleep */
			uint32_t     timestamp;        /**< time stamp of the sleep phase transition (seconds)*/
			int32_t      sleep_onset;      /**< time until first period of 20 min sleep without more than 1 min wake */
			int32_t      sleep_latency;    /**< time until first sleep phase */
			uint32_t     time_in_bed;      /**< time in bed (seconds) */
			uint32_t     total_sleep_time; /**< total sleep time (seconds) */
			uint8_t      sleep_efficiency; /**< ratio between total sleep time and time in bed */
		} sleepanalysis;                   /**< sleep analysis data */
		struct {
			int          event;            /**< BAC extended data begin/end event as of
			                                    enum inv_sensor_bac_ext_event */
		} bacext;                          /**< activity classifier (BAC) extended data */
		struct {
			uint32_t     durationWalk;          /**< ms */
			uint32_t     durationRun;           /**< ms */
			uint32_t     durationTransportSit;  /**< ms */
			uint32_t     durationTransportStand;/**< ms */
			uint32_t     durationBiking;        /**< ms */
			uint32_t     durationStillSit;      /**< ms */
			uint32_t     durationStillStand;    /**< ms */
			uint32_t     durationTotalSit;      /**< Still-Sit + Transport-Sit + Biking (ms) */
			uint32_t     durationTotalStand;    /**< Still-Stand + Transport-Stand (ms) */
			uint32_t     stepWalk;              /**< walk step count */
			uint32_t     stepRun;               /**< run step count */
		} bacstat;                              /**< activity classifier (BAC) statistics data */
		struct {
			int32_t      floorsUp;         /**< number of floors climbed Up on foot by user. */
			int32_t      floorsDown;       /**< number of floors climbed Down on foot by user. */
		} floorclimb;                      /**< floor climbed data */
		struct {
			int32_t      instantEEkcal;    /**< energy expenditure in kilocalorie/min since last output. Format is q15: 2^15 = 1 kcal/min */
			int32_t      instantEEmets;    /**< energy expenditure in METs(Metabolic Equivalent of Task) since last output. Format is q15: 2^15 = 1 METs */
			int32_t      cumulativeEEkcal; /**< cumulative energy expenditure since the last reset in kilocalorie. Format is q0: 1 = 1 kcal */
			int32_t      cumulativeEEmets; /**< cumulative energy expenditure since the last reset in METs (Metabolic Equivalent of Task). Format is q0: 1 = 1 METs */
		} energyexp;                       /**< energy expenditure data */
		struct {
			int32_t      distanceWalk;     /**< distance in meters */
			int32_t      distanceRun;      /**< distance in meters */
		} distance;                        /**< distance data */
		struct {
			int32_t      table[7];         /**< data encrypted table */
		} dataencryption;
		struct {
			float        tmp;              /**< temperature in deg celcius */
		} temperature;                     /**< temperature data */
		struct {
			float        percent;          /**< relative humidity in % */
		} humidity;                        /**< humidity data */
		struct {
			uint64_t     count;            /**< number of steps */
		} step;                            /**< step-counter data */
		struct {
			uint32_t     level;            /**< light level in lux */
		} light;                           /**< light data */
		struct {
			uint32_t     distance;         /**< distance in mm */
		} proximity;                       /**< proximity data */
		struct {
			uint32_t     pressure;         /**< pressure in Pa */
		} pressure;                        /**< pressure data */
		struct {
			int          event;            /**< BAC data begin/end event as of
			                                    enum inv_sensor_bac_event */
		} bac;                             /**< BAC data */
		struct {
			uint32_t     fxdata[12];       /**< PDR data in fixpoint*/
		} pdr;                             /**< PDR data */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			int16_t      delta_ts;         /**< timestamp delta between standard gyro and EIS gyro */
		} eis;                             /**< EIS data
		                                        @warning experimental: structure is likely to change in near future */
		struct {
			int32_t      vect[3];          /**< x,y,z vector data */
			uint32_t     fsr;              /**< full scale range */
		} raw3d;                           /**< 3d raw acc, mag or gyr*/
		struct {
			int32_t      raw;              /**< raw temperature value */
		} rawtemp;                         /**< Raw temperature data*/
		struct {
			uint8_t      status[6];        /**< raw temperature value */
		} tsimu_status;                    /**< TSIMU status data*/
		//inv_bool_t       event;            /**< event state for gesture-like sensor
		inv_bool_t       event;            /**< event state for gesture-like sensor
		                                        (SMD, B2S, Step-detector, Tilt-detector, Wake, Glance, Pick-Up, Shake, Double-tap, ...) */
		struct {
			int16_t delay_count;           /**< delay counter in us between FSYNC tag and previous gyro data */
		} fsync_event;                     /** < FSYNC tag (EIS sensor) */
		struct {
			unsigned     flags;             /** WOM status flags: non-zero value - motion detected
                                                                  bit0 - motion detected around X axis
                                                                  bit1 - motion detected around Y axis
                                                                  bit2 - motion detected around Z axis
                                            */
		} wom;                              /** Wake-up on motion data */
		struct {
			uint8_t *    buffer;           /**< pointer to buffer */
			uint32_t     size;             /**< current buffer size */
		} audio_buffer;                    /**< buffer of audio data */
		struct {
			struct {
				int        event;          /**< BAC data begin/end event as of  enum inv_sensor_bac_event */
			} bac;                         /**< BAC data */
			struct {
				uint64_t   count;          /**< number of steps */
			} step;                        /**< step-counter data */
			int32_t      cumulativeEEkcal; /**< cumulative energy expenditure since the last reset in kilocalorie. Format is q0: 1 = 1 kcal */
			int32_t      distance;         /**< sum of walk and run distance in meters */
		} bscd;                            /**< buffer of custom BSCD */
		struct {
			int32_t      raw_pressure;         /**< raw pressure */
			float        pressure;             /**< pressure in Pa */
			int32_t      raw_temperature;      /**< raw temperature */
			float        temperature;          /**< temperature in deg C */
		} custom_pressure;                        /**< pressure data */
		uint8_t          reserved[INV_SENSOR_EVENT_DATA_SIZE];     /**< reserved sensor data for future sensor */
	} data;                                /**< sensor data */
} inv_sensor_event_t;

/** @brief Sensor listener event callback definition
 *  @param[in] event     reference to sensor event
 *  @param[in] context   listener context
 *  @return    none
 */
typedef void (*inv_sensor_listener_event_cb_t)(const inv_sensor_event_t * event,
		void * context);

/** @brief Sensor event listener definition
 */
typedef struct inv_sensor_listener {
	inv_sensor_listener_event_cb_t event_cb; /**< sensor event callback */
	void *                         context;  /**< listener context */
} inv_sensor_listener_t;

/** @brief Helper to initialize a listener object
 */
static inline void inv_sensor_listener_init(inv_sensor_listener_t * listener,
	inv_sensor_listener_event_cb_t event_cb, void * context)
{
	listener->event_cb = event_cb;
	listener->context  = context;
}

/** @brief Helper to notify a listener of a new sensor event
 */
static inline void inv_sensor_listener_notify(const inv_sensor_listener_t * listener,
		const inv_sensor_event_t * event)
{
	if(listener) {
		listener->event_cb(event, listener->context);
	}
}

/** @brief Helper macro to retrieve sensor type (without wake-up flag) from a sensor id.
 */
#define INV_SENSOR_ID_TO_TYPE(sensor) \
	((unsigned int)(sensor) & ~INV_SENSOR_TYPE_WU_FLAG)

/** @brief Helper macro that check if given sensor is of known type
 */
#define INV_SENSOR_IS_VALID(sensor) \
	(INV_SENSOR_ID_TO_TYPE(sensor) < INV_SENSOR_TYPE_MAX)

/** @brief Helper macro that check if given sensor is a wake-up sensor
 */
#define INV_SENSOR_IS_WU(sensor) \
 	(((int)(sensor) & INV_SENSOR_TYPE_WU_FLAG) != 0)

/** @brief Utility function that returns a string from a sensor id
 *  Empty string is returned if sensor is invalid
 */
//const char INV_EXPORT * inv_sensor_str(int sensor);
const char * inv_sensor_str(int sensor);

/** @brief Alias for inv_sensor_str
 */
#define inv_sensor_2str 	inv_sensor_str

#ifdef __cplusplus
}
#endif

#endif /* _INV_SENSOR_TYPES_H_ */

/** @} */
