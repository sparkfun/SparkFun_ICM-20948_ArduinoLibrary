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

#include "ICM_20948_SensorTypes.h"

#include <assert.h>

const char * inv_sensor_2str(int sensor)
{
	static const char * sensor_str[INV_SENSOR_TYPE_MAX][2] = {
		{ "SENSOR_RESERVED",               "SENSOR_RESERVED_WU" },
		{ "SENSOR_ACCELEROMETER",          "SENSOR_ACCELEROMETER_WU" },
		{ "SENSOR_MAGNETOMETER",           "SENSOR_MAGNETOMETER_WU" },
		{ "SENSOR_ORIENTATION",            "SENSOR_ORIENTATION_WU" },
		{ "SENSOR_GYROSCOPE",              "SENSOR_GYROSCOPE_WU" },
		{ "SENSOR_LIGHT",                  "SENSOR_LIGHT_WU" },
		{ "SENSOR_PRESSURE",               "SENSOR_PRESSURE_WU" },
		{ "SENSOR_TEMPERATURE",            "SENSOR_TEMPERATURE_WU" },
		{ "SENSOR_PROXIMITY",              "SENSOR_PROXIMITY_WU" },
		{ "SENSOR_GRAVITY",                "SENSOR_GRAVITY_WU" },
		{ "SENSOR_LINEAR_ACCELERATION",    "SENSOR_LINEAR_ACCELERATION_WU" },
		{ "SENSOR_ROTATION_VECTOR",        "SENSOR_ROTATION_VECTOR_WU" },
		{ "SENSOR_HUMIDITY",               "SENSOR_HUMIDITY_WU" },
		{ "SENSOR_AMBIENT_TEMPERATURE",    "SENSOR_AMBIENT_TEMPERATURE_WU" },
		{ "SENSOR_UNCAL_MAGNETOMETER",     "SENSOR_UNCAL_MAGNETOMETER_WU" },
		{ "SENSOR_GAME_ROTATION_VECTOR",   "SENSOR_GAME_ROTATION_VECTOR_WU" },
		{ "SENSOR_UNCAL_GYROSCOPE",        "SENSOR_UNCAL_GYROSCOPE_WU" },
		{ "SENSOR_SMD",                    "SENSOR_SMD_WU" },
		{ "SENSOR_STEP_DETECTOR",          "SENSOR_STEP_DETECTOR_WU" },
		{ "SENSOR_STEP_COUNTER",           "SENSOR_STEP_COUNTER_WU" },
		{ "SENSOR_GEOMAG_ROTATION_VECTOR", "SENSOR_GEOMAG_ROTATION_VECTOR_WU" },
		{ "SENSOR_HEART_RATE",             "SENSOR_HEART_RATE_WU" },
		{ "SENSOR_TILT_DETECTOR",          "SENSOR_TILT_DETECTOR_WU" },
		{ "SENSOR_WAKE_GESTURE",           "SENSOR_WAKE_GESTURE_WU" },
		{ "SENSOR_GLANCE_GESTURE",         "SENSOR_GLANCE_GESTURE_WU" },
		{ "SENSOR_PICK_UP_GESTURE",        "SENSOR_PICK_UP_GESTURE_WU" },
		{ "SENSOR_BAC",                    "SENSOR_BAC_WU" },
		{ "SENSOR_PDR",                    "SENSOR_PDR_WU" },
		{ "SENSOR_B2S",                    "SENSOR_B2S_WU" },
		{ "SENSOR_3AXIS",                  "SENSOR_3AXIS_WU" },
		{ "SENSOR_EIS",                    "SENSOR_EIS_WU" },
		{ "SENSOR_OIS",                    "SENSOR_OIS_WU" },
		{ "SENSOR_RAW_ACCELEROMETER",      "SENSOR_RAW_ACCELEROMETER_WU" },
		{ "SENSOR_RAW_GYROSCOPE",          "SENSOR_RAW_GYROSCOPE_WU" },
		{ "SENSOR_RAW_MAGNETOMETER",       "SENSOR_RAW_MAGNETOMETER_WU"},
		{ "SENSOR_RAW_TEMPERATURE",        "SENSOR_RAW_TEMPERATURE_WU"},
		{ "SENSOR_CUSTOM_PRESSURE",        "SENSOR_CUSTOM_PRESSURE_WU"},
		{ "SENSOR_MIC",                    "SENSOR_MIC_WU" },
		{ "SENSOR_TSIMU",                  "SENSOR_TSIMU_WU" },
		{ "SENSOR_RAW_PPG",                "SENSOR_RAW_PPG_WU" },
		{ "SENSOR_HRV",                    "SENSOR_HRV_WU" },
		{ "SENSOR_SLEEP_ANALYSIS",         "SENSOR_SLEEP_ANALYSIS_WU"},
		{ "SENSOR_BAC_EXTENDED",           "SENSOR_BAC_EXTENDED_WU"},
		{ "SENSOR_BAC_STATISTICS",         "SENSOR_BAC_STATISTICS_WU"},
		{ "SENSOR_FLOOR_CLIMB_COUNTER",    "SENSOR_FLOOR_CLIMB_COUNTER_WU"},
		{ "SENSOR_ENERGY_EXPENDITURE",     "SENSOR_ENERGY_EXPENDITURE_WU"},
		{ "SENSOR_DISTANCE",               "SENSOR_DISTANCE_WU"},
		{ "SENSOR_SHAKE",                  "SENSOR_SHAKE_WU"},
		{ "SENSOR_DOUBLE_TAP",             "SENSOR_DOUBLE_TAP"},
		{ "SENSOR_CUSTOM0",                "SENSOR_CUSTOM0_WU" },
		{ "SENSOR_CUSTOM1",                "SENSOR_CUSTOM1_WU" },
		{ "SENSOR_CUSTOM2",                "SENSOR_CUSTOM2_WU" },
		{ "SENSOR_CUSTOM3",                "SENSOR_CUSTOM3_WU" },
		{ "SENSOR_CUSTOM4",                "SENSOR_CUSTOM4_WU" },
		{ "SENSOR_CUSTOM5",                "SENSOR_CUSTOM5_WU" },
		{ "SENSOR_CUSTOM6",                "SENSOR_CUSTOM6_WU" },
		{ "SENSOR_CUSTOM7",                "SENSOR_CUSTOM7_WU" },
		{ "SENSOR_WOM",                    "SENSOR_WOM_WU" },
		{ "SENSOR_SEDENTARY_REMIND",       "SENSOR_SEDENTARY_REMIND_WU"},
		{ "SENSOR_DATA_ENCRYPTION",        "SENSOR_DATA_ENCRYPTION_WU"},
		{ "SENSOR_FSYNC_EVENT",            "SENSOR_FSYNC_EVENT_WU"},
		{ "SENSOR_HIGH_RATE_GYRO",         "SENSOR_HIGH_RATE_GYRO_WU"},
		{ "SENSOR_CUSTOM_BAC_SC_EE_DIST",  "SENSOR_CUSTOM_BAC_SC_EE_DIST_WU"},
		{ "SENSOR_HRM_LOGGER",             "SENSOR_HRM_LOGGER_WU"},
		{ "SENSOR_PREDICTIVE_QUATERNION",  "SENSOR_PREDICTIVE_QUATERNION_WU"},
	};

	if (INV_SENSOR_IS_VALID(sensor)) {
		const char * s = sensor_str[INV_SENSOR_ID_TO_TYPE(sensor)][INV_SENSOR_IS_WU(sensor)];

		assert(s != 0); // we forgot to update the array after adding a sensor!

		if(s != 0)
			return s;
	}

	return "";
}
