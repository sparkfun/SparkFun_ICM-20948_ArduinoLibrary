#include "ICM_20948_C.h"
#include "ICM_20948_REGISTERS.h"



const ICM_20948_Serif_t NullSerif = {
	NULL,	// write
	NULL,	// read
	NULL,
};

// Private function prototypes
ICM_20948_Status_e	ICM_20948_execute_r( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ); // Executes a R or W witht he serif vt as long as the pointers are not null
ICM_20948_Status_e	ICM_20948_execute_w( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len );





// Function definitions
ICM_20948_Status_e	ICM_20948_link_serif( ICM_20948_Device_t* pdev, const ICM_20948_Serif_t* s ){
	if(s == NULL){ return ICM_20948_Stat_ParamErr; }
	if(pdev == NULL){ return ICM_20948_Stat_ParamErr; }
	pdev->_serif = s;
	return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e	ICM_20948_execute_w( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ){
	if( pdev->_serif->write == NULL ){ return ICM_20948_Stat_NotImpl; }
	return (*pdev->_serif->write)( regaddr, pdata, len, pdev->_serif->user );
}

ICM_20948_Status_e	ICM_20948_execute_r( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ){
	if( pdev->_serif->read == NULL ){ return ICM_20948_Stat_NotImpl; }
	return (*pdev->_serif->read)( regaddr, pdata, len, pdev->_serif->user );
}





ICM_20948_Status_e	ICM_20948_set_bank( ICM_20948_Device_t* pdev, uint8_t bank ){
	if( bank > 3 ){ return ICM_20948_Stat_ParamErr; } // Only 4 possible banks
	return ICM_20948_execute_w( pdev, REG_BANK_SEL, &bank, 1 );
}

ICM_20948_Status_e	ICM_20948_sw_reset( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	reg.DEVICE_RESET = 1;

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_sleep				( ICM_20948_Device_t* pdev, bool on ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	if(on){ reg.SLEEP = 1; }
	else{ reg.SLEEP = 0; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_low_power			( ICM_20948_Device_t* pdev, bool on ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	if(on){ reg.LP_EN = 1; }
	else{ reg.LP_EN = 0; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_clock_source	( ICM_20948_Device_t* pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	
	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	reg.CLKSEL = source;

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}



ICM_20948_Status_e ICM_20948_get_who_am_i( ICM_20948_Device_t* pdev, uint8_t* whoami ){
	if( whoami == NULL ){ return ICM_20948_Stat_ParamErr; }
	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	return ICM_20948_execute_r( pdev, AGB0_REG_WHO_AM_I, whoami, 1 );
}

ICM_20948_Status_e	ICM_20948_check_id( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	uint8_t whoami = 0x00;
	retval = ICM_20948_get_who_am_i( pdev, &whoami );
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	if( whoami != ICM_20948_WHOAMI ){ return ICM_20948_Stat_WrongID; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_data_ready( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_INT_STATUS_1_t reg;
	retval = ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_STATUS_1, (uint8_t*)&reg, sizeof(ICM_20948_INT_STATUS_1_t));
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	if( reg.RAW_DATA_0_RDY_INT ){ return retval; }
	return ICM_20948_Stat_NoData;
}














ICM_20948_Status_e	ICM_20948_set_sample_mode( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_LP_CONFIG_t reg;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst ) ) ){ return ICM_20948_Stat_SensorNotSupported; }
	
	retval = ICM_20948_set_bank(pdev, 0);				// Must be in the right bank
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_LP_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_LP_CONFIG_t));
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	
	if( sensors & ICM_20948_Internal_Acc ){ reg.ACCEL_CYCLE = mode; }		// Set all desired sensors to this setting
	if( sensors & ICM_20948_Internal_Gyr ){ reg.GYRO_CYCLE = mode; }
	if( sensors & ICM_20948_Internal_Mst ){ reg.I2C_MST_CYCLE = mode; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_full_scale 	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		reg.ACCEL_FS_SEL = fss.a;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		reg.GYRO_FS_SEL = fss.g;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_dlpf_cfg		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, uint8_t cfg ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		reg.ACCEL_DLPFCFG = cfg;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		reg.GYRO_DLPFCFG = cfg;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}	

ICM_20948_Status_e	ICM_20948_enable_dlpf		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, bool enable ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		if( enable ){ reg.ACCEL_FCHOICE = 1; }
		else{ reg.ACCEL_FCHOICE = 0; }
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		if( enable ){ reg.GYRO_FCHOICE = 1; }
		else{ reg.GYRO_FCHOICE = 0; }
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}


