/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
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

/** @defgroup SensorConfig Sensor Configuration
 *  @brief    General sensor configuration types definitions
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_SENSOR_CONFIG_H_
#define _INV_SENSOR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ICM_20948_InvBool.h" // Defines true and false

/** @brief Sensor type identifier definition
 */
enum inv_sensor_config {
	INV_SENSOR_CONFIG_RESERVED = 0,     /**< Reserved config ID: do not use */
	INV_SENSOR_CONFIG_MOUNTING_MATRIX,  /**< 3x3 mounting matrix */
	INV_SENSOR_CONFIG_GAIN,             /**< 3x3 gain matrix (to correct for cross-axis defect)*/
	INV_SENSOR_CONFIG_OFFSET,           /**< 3d offset vector  */
	INV_SENSOR_CONFIG_CONTEXT,          /**< arbitrary context buffer */
	INV_SENSOR_CONFIG_FSR,              /**< Full scale range */
	INV_SENSOR_CONFIG_RESET,            /**< Reset the specified service */
	INV_SENSOR_CONFIG_POWER_MODE,       /**< Low Power or Low Noise mode */
	INV_SENSOR_CONFIG_CUSTOM   = 128 ,  /**< Configuration ID above this value are device specific */
	INV_SENSOR_CONFIG_MAX      = 255,   /**< Absolute maximum value for sensor config */
};

/** @brief Define mounting matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_MOUNTING_MATRIX config ID)
 *         Mounting matrix value can be set (is supported by device implementation) to convert from
 *         sensor reference to system reference.
 *         Value is expetcted to be a rotation matrix.
 */
typedef struct inv_sensor_config_mounting_mtx {
	float matrix[3*3];
} inv_sensor_config_mounting_mtx_t;

/** @brief Define gain matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_GAIN config ID)
 *         Gain matrix value can be set (is supported by device implementation) to correct for
 *         cross-axis defect.
 */
typedef struct inv_sensor_config_gain {
	float gain[3*3];
} inv_sensor_config_gain_t;

/** @brief Define offset vector value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_OFFSET config ID)
 *         Offset value can be set (is supported by device implementation) to correct for bias defect.
 *         If applied to RAW sensor, value is expected to be in lsb.
 *         If applied to other sensor, value is expected to be in sensor unit (g, uT or dps).
 */
typedef struct inv_sensor_config_offset {
	float offset[3];
} inv_sensor_config_offset_t;

/** @brief Define configuration context value
 *         (associated with INV_SENSOR_CONFIG_CONTEXT config ID)
 *         Context is an arbitrary buffer specific to the sensor and device implemetation
 */
typedef struct inv_sensor_config_context {
	uint8_t context[64];
} inv_sensor_config_context_t;

/** @brief Define full-scale range value for accelero, gyro or mangetometer based sensor
 *         (associated with INV_SENSOR_CONFIG_FSR config ID)
 *         Value is expetcted to be expressed in mg, dps and uT for accelero, gyro or mangetometer
 *         eg: +/-2g = 2000
 *             +/-250 dps = 250
 *             +/-2000 uT = 2000
 */
typedef struct inv_sensor_config_fsr {
	uint32_t fsr;
} inv_sensor_config_fsr_t;

/** @brief Define chip power mode
 *         (associated with INV_SENSOR_CONFIG_POWER_MODE config ID)
 *         Value is expetcted to be 0 for low power or 1 for low noise
 */
typedef struct inv_sensor_config_powermode {
	uint8_t lowpower_or_highperformance;
} inv_sensor_config_powermode_t;

/** @brief Define the configuration for the energy expenditure's algorithm
 *  @param age      age in year; Range is (0;100).
 *  @param gender   gender is 0 for men, 1 for female.
 *  @param height   height in centimeter; Range is (50;250)
 *  @param weight   weight in kg; Range is (3;300)
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_energy_expenditure {
	int32_t age;
	int32_t gender;
	int32_t height;
	int32_t weight;
	uint32_t enableNotify;
} inv_sensor_config_energy_expenditure_t;

/** @brief Define the configuration for the distance's algorithm
 *  @param user_height height of the user in cm
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_distance{
	int32_t user_height;
	uint32_t enableNotify;
}inv_sensor_config_distance_t;

/** @brief Define the configuration for BAC
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_bac{
	uint32_t enableNotify;
}inv_sensor_config_bac_t;

/** @brief Define the configuration for steps counter
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_stepc{
	uint32_t enableNotify;
}inv_sensor_config_stepc_t;

/** @brief Define the configuration for the shake wrist's algorithm
 *  @param max_period    This parameter sets the maximal duration for half oscillation to detect a Shake wrist.
 *                       The default value is 20, recommend range [15 ; 40], 15 for the lower sensitivity and 40 for the higher sensitivity.
 *                       Notice that increasing the sensitivity will increase the number of false detection, and also slightly increase response time.
 *  @param dummy_padding Dummy byte for padding. Set it to 0.
 */
typedef struct inv_sensor_config_shake_wrist{
	uint8_t max_period;
	uint8_t dummy_padding;
}inv_sensor_config_shake_wrist_t;

/** @brief Define the configuration for the double tap's algorithm
 *  @param minimum_threshold This parameter sets the minimum threshold to reach in order to start a Tap detection.
 *                           Default value is 2000, recommended range [500 ; 2500]
 *  @param t_max             This parameter sets the maximum time after a Tap event in [sample]. Default value is 100, recommended range [30 ; 200].
 */
typedef struct inv_sensor_config_double_tap{
	int16_t minimum_threshold;
	uint16_t t_max;
}inv_sensor_config_double_tap_t;

/** @brief Define the configuration for the BSCD virtual sensor
 *  @param Age            age in year; Range is (0;100). Default is 35.
 *  @param Gender         gender is 0 for men, 1 for female. Default is 0
 *  @param Height         height in centimeter; Range is (50;250). Default is 175.
 *  @param Weight         weight in kg; Range is (3;300). Default is 75
 *  @param enableNotify   bitmask to enable/disable notify on a a specific sensor event
 *                        bit 0 (1): enable/disable notify on BAC event
 *                        bit 1 (2): enable/disable notify on step counter event
 *                        bit 2 (4): enable/disable notify on energy expenditure event
 *                        bit 3 (8): enable/disable notify on distance event
 */
typedef struct inv_sensor_config_BSCD
{
	int32_t Age;
	int32_t Gender;
	int32_t Height;
	int32_t Weight;
	uint32_t enableNotify;
} inv_sensor_config_BSCD_t;

#ifdef __cplusplus
}
#endif

#endif /* _INV_SENSOR_CONFIG_H_ */

/** @} */
