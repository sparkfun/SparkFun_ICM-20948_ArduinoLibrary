/*

This is a C-compatible interface to the features presented by the ICM 20948 9-axis device
The imementation of the interface is flexible

*/ 


#ifndef _ICM_20948_C_H_
#define _ICM_20948_C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ICM_20948_ENUMERATIONS.h"	// This is to give users access to usable value definiitons

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

#define ICM_20948_I2C_ADDR_AD0 	0x68 // Or 0x69 when AD0 is high
#define ICM_20948_I2C_ADDR_AD1	0x69 //
#define ICM_20948_WHOAMI		0xEA

typedef enum{
	ICM_20948_Stat_Ok = 0x00,	// The only return code that means all is well
	ICM_20948_Stat_Err,			// A general error
	ICM_20948_Stat_NotImpl,		// Returned by virtual functions that are not implemented
	ICM_20948_Stat_ParamErr,
	ICM_20948_Stat_WrongID,
	ICM_20948_Stat_InvSensor,	// Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
	ICM_20948_Stat_NoData,
	ICM_20948_Stat_SensorNotSupported,
}ICM_20948_Status_e;

typedef enum{
	ICM_20948_Internal_Acc = (1 << 0),
	ICM_20948_Internal_Gyr = (1 << 1),
	ICM_20948_Internal_Mag = (1 << 2),
	ICM_20948_Internal_Tmp = (1 << 3),
	ICM_20948_Internal_Mst = (1 << 4), 		// I2C Master Ineternal
}ICM_20948_InternalSensorID_bm;			// A bitmask of internal sensor IDs

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
}ICM_20948_axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
}ICM_20948_axis1bit16_t;

typedef struct{								
	uint8_t				a			: 2;
	uint8_t 			g			: 2;
	uint8_t 			reserved_0	: 4;
}ICM_20948_fss_t;							// Holds full-scale settings to be able to extract measurements with units

typedef struct{
	ICM_20948_axis3bit16_t	acc;
	ICM_20948_axis3bit16_t	gyr;
	ICM_20948_axis3bit16_t	mag;
	ICM_20948_axis1bit16_t	tmp;
	ICM_20948_fss_t			fss;		// Full-scale range settings for this measurement
}ICM_20948_AGMT_t;

typedef struct{
	ICM_20948_Status_e	(*write)( uint8_t regaddr, uint8_t* pdata, uint32_t len, void* user);
	ICM_20948_Status_e	(*read)( uint8_t regaddr, uint8_t* pdata, uint32_t len, void* user);
	// void				(*delay)(uint32_t ms);
	void*				user;
}ICM_20948_Serif_t;							// This is the vtable of serial interface functions
extern const ICM_20948_Serif_t NullSerif;	// Here is a default for initialization (NULL)

typedef struct{
	const ICM_20948_Serif_t*	_serif;				// Pointer to the assigned Serif (Serial Interface) vtable
}ICM_20948_Device_t;						// Definition of device struct type


// Here's the list of what I want to be able to do:
/*

perform a generic startup routine that sets most things in the optimal performance range
Read / check against Who Am I
Add magnetometer to auxillary I2C bus and read it's values from the sensor values locations, configure ODR when accelerometer and gyro are both disabled
read raw accel and gyro values
configure accel/gyro update rates and dlpf's
read raw temp values
configure temperature sensor
load DMP firmware into the device
read DMP results from the device
configure interrupts 
	- configure interrupt and FSYNC pins 
	- configure which interrupts activate the interrupt pin
respond to interrupts on INT
configure FIFO (and use it)



callbacks for the user to respond to interrupt events


*/

// ICM_20948_Status_e ICM_20948_Startup( ICM_20948_Device_t* pdev ); // For the time being this performs a standardized startup routine


ICM_20948_Status_e	ICM_20948_link_serif( ICM_20948_Device_t* pdev, const ICM_20948_Serif_t* s );	// Links a SERIF structure to the device

// Device Level
ICM_20948_Status_e	ICM_20948_set_bank			( ICM_20948_Device_t* pdev, uint8_t bank );									// Sets the bank
ICM_20948_Status_e	ICM_20948_sw_reset			( ICM_20948_Device_t* pdev );												// Performs a SW reset
ICM_20948_Status_e	ICM_20948_sleep				( ICM_20948_Device_t* pdev, bool on );										// Set sleep mode for the chip
ICM_20948_Status_e	ICM_20948_low_power			( ICM_20948_Device_t* pdev, bool on );										// Set low power mode for the chip
ICM_20948_Status_e	ICM_20948_set_clock_source	( ICM_20948_Device_t* pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source ); 		// Choose clock source
ICM_20948_Status_e	ICM_20948_get_who_am_i		( ICM_20948_Device_t* pdev, uint8_t* whoami );								// Return whoami in out prarmeter
ICM_20948_Status_e	ICM_20948_check_id			( ICM_20948_Device_t* pdev );												// Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI
ICM_20948_Status_e	ICM_20948_data_ready		( ICM_20948_Device_t* pdev );												// Returns 'Ok' if data is ready

// Internal Sensor Options
ICM_20948_Status_e	ICM_20948_set_sample_mode	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode );	// Use to set accel, gyro, and I2C master into cycled or continuous modes
ICM_20948_Status_e	ICM_20948_set_full_scale 	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss );
ICM_20948_Status_e	ICM_20948_set_dlpf_cfg		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, uint8_t cfg );			
ICM_20948_Status_e	ICM_20948_enable_dlpf		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, bool enable );
// TEMP_DIS







#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif 	/* _ICM_20948_C_H_ */