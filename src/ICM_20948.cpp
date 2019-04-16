#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h" // temporary

// Forward Declarations
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t* data, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t* buff, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_write_SPI(uint8_t reg, uint8_t* buff, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_read_SPI(uint8_t reg, uint8_t* buff, uint32_t len, void* user);





// Base
ICM_20948::ICM_20948(){

}

extern void printRawAGMT( ICM_20948_AGMT_t agmt);
ICM_20948_AGMT_t ICM_20948::getAGMT                 ( void ){
    status = ICM_20948_get_agmt( &_device, &agmt );
    return agmt;

	// // ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	// const uint8_t numbytes = 14;
	// uint8_t buff[numbytes];

	// // Get readings
	// (ICM_20948_Status_e)setBank( 0 ); 
	// read( (uint8_t)AGB0_REG_ACCEL_XOUT_H, buff, numbytes );

	// // pagmt->acc.axes.x = ((buff[0] << 8) | (buff[1] & 0xFF));
	// // pagmt->acc.axes.y = ((buff[2] << 8) | (buff[3] & 0xFF));
	// // pagmt->acc.axes.z = ((buff[4] << 8) | (buff[5] & 0xFF));

	// // pagmt->gyr.axes.x = ((buff[6] << 8) | (buff[7] & 0xFF));
	// // pagmt->gyr.axes.y = ((buff[8] << 8) | (buff[9] & 0xFF));
	// // pagmt->gyr.axes.z = ((buff[10] << 8) | (buff[11] & 0xFF));

	// // pagmt->tmp.val = ((buff[12] << 8) | (buff[13] & 0xFF));

    // for( uint8_t indi = 0; indi < numbytes; indi++ ){
    //     Serial.print("0x");
    //     if(buff[indi] < 16){
    //         Serial.print("0");
    //     }
    //     Serial.print(buff[indi], HEX);
    //     Serial.print(", ");
    // }
    // Serial.println();
}


const char* ICM_20948::statusString                 ( ICM_20948_Status_e stat ){
    ICM_20948_Status_e val;
    if( stat == ICM_20948_Stat_NUM){
        val = status;
    }else{
        val = stat;
    }

    switch(val){
        case ICM_20948_Stat_Ok : return "All is well."; break;
        case ICM_20948_Stat_Err : return "General Error"; break;
	    case ICM_20948_Stat_NotImpl : return "Not Implemented"; break;
        case ICM_20948_Stat_ParamErr : return "Parameter Error"; break;
        case ICM_20948_Stat_WrongID : return "Wrong ID"; break;
        case ICM_20948_Stat_InvalSensor : return "Invalid Sensor"; break;
        case ICM_20948_Stat_NoData : return "Data Underflow"; break;
        case ICM_20948_Stat_SensorNotSupported : return "Sensor Not Supported"; break;
        default :
            return "Unknown Status"; break;
        
    }
    return "None";
}



// Device Level
ICM_20948_Status_e	ICM_20948::setBank			    ( uint8_t bank ){
    status =  ICM_20948_set_bank( &_device, bank );
    return status;
}

ICM_20948_Status_e	ICM_20948::swReset			    ( void ){
    status = ICM_20948_sw_reset( &_device );
    return status;
}

ICM_20948_Status_e	ICM_20948::sleep				( bool on ){
    status = ICM_20948_sleep( &_device, on );
    return status;
}

ICM_20948_Status_e	ICM_20948::lowPower			( bool on ){
    status = ICM_20948_low_power( &_device, on );
    return status;
}

ICM_20948_Status_e	ICM_20948::setClockSource	    ( ICM_20948_PWR_MGMT_1_CLKSEL_e source ){
    status = ICM_20948_set_clock_source( &_device, source );
    return status;
}

ICM_20948_Status_e	ICM_20948::checkID			    ( void ){
    status = ICM_20948_check_id( &_device );
    return status;
}

bool	            ICM_20948::dataReady		    ( void ){
    status = ICM_20948_data_ready( &_device );
    if( status == ICM_20948_Stat_Ok ){ return true; }
    return false;
}

uint8_t	            ICM_20948::getWhoAmI		    ( void ){
    uint8_t retval = 0x00;
    status = ICM_20948_get_who_am_i( &_device, &retval );
    return retval;
}

bool                ICM_20948::isConnected         ( void ){
    status = checkID();
    if( status == ICM_20948_Stat_Ok ){ return true; }
    return false;
}


// Internal Sensor Options
ICM_20948_Status_e	ICM_20948::setSampleMode	    ( uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode ){
    status = ICM_20948_set_sample_mode( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode );
    return status;
}

ICM_20948_Status_e	ICM_20948::setFullScale 	    ( uint8_t sensor_id_bm, ICM_20948_fss_t fss ){
    status = ICM_20948_set_full_scale( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss );
    return status;
}

ICM_20948_Status_e	ICM_20948::setDLPFcfg		    ( uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg ){
    status = ICM_20948_set_dlpf_cfg( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg );
    return status;
}

ICM_20948_Status_e	ICM_20948::enableDLPF		    ( uint8_t sensor_id_bm, bool enable ){
    status = ICM_20948_enable_dlpf( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable );
    return status;
}

ICM_20948_Status_e	ICM_20948::setSampleRate	    ( uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt ){
    status = ICM_20948_set_sample_rate( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt );
    return status;
}





// Interface Options
ICM_20948_Status_e	ICM_20948::i2cMasterPassthrough 	( bool passthrough ){
    status = ICM_20948_i2c_master_passthrough ( &_device, passthrough );
    return status;
}

ICM_20948_Status_e	ICM_20948::i2cMasterEnable          ( bool enable ){
    status = ICM_20948_i2c_master_enable( &_device, enable );
    return status;
}

ICM_20948_Status_e	ICM_20948::i2cMasterConfigureSlave  ( uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap ){
    status = ICM_20948_i2c_master_configure_slave 		( &_device, slave, addr, reg, len, Rw, enable, data_only, grp, swap );
    return status;
}

ICM_20948_Status_e 	ICM_20948::i2cMasterSLV4Transaction( uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len, bool Rw, bool send_reg_addr ){
    status = ICM_20948_i2c_master_slv4_txn( &_device, addr, reg, data, len, Rw, send_reg_addr );
    return status;
}
ICM_20948_Status_e	ICM_20948::i2cMasterSingleW        ( uint8_t addr, uint8_t reg, uint8_t data ){
    status = ICM_20948_i2c_master_single_w( &_device, addr, reg, &data );
    return status;
}
uint8_t	ICM_20948::i2cMasterSingleR        ( uint8_t addr, uint8_t reg ){
    uint8_t data;
    status = ICM_20948_i2c_master_single_r( &_device, addr, reg, &data );
    return data;
}










// direct read/write
ICM_20948_Status_e  ICM_20948::read                 ( uint8_t reg, uint8_t* pdata, uint32_t len){
    status = ICM_20948_execute_r( &_device, reg, pdata, len );
}

ICM_20948_Status_e  ICM_20948::write                ( uint8_t reg, uint8_t* pdata, uint32_t len){
    status = ICM_20948_execute_w( &_device, reg, pdata, len );
}




























// I2C
ICM_20948_I2C::ICM_20948_I2C(){

}

ICM_20948_Status_e ICM_20948_I2C::begin(TwoWire &wirePort, bool ad0val, uint8_t ad0pin){
    // Associate 
	_ad0 = ad0pin;
	_i2c = &wirePort;
	_ad0val = ad0val;

    _addr = ICM_20948_I2C_ADDR_AD0;
    if( _ad0val ){ _addr = ICM_20948_I2C_ADDR_AD1; }

    // Set pinmodes
	if(_ad0 != ICM_20948_ARD_UNUSED_PIN){ pinMode(_ad0, OUTPUT); }

    // Set pins to default positions
	if(_ad0 != ICM_20948_ARD_UNUSED_PIN){ digitalWrite(_ad0, _ad0val); }

    // _i2c->begin(); // Moved into user's sketch

    // Set up the serif
    _serif.write = ICM_20948_write_I2C;
    _serif.read = ICM_20948_read_I2C;
    _serif.user = (void*)this;              // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

    return ICM_20948_Stat_Ok;
}







// SPI
SPISettings ICM_20948_SPI_DEFAULT_SETTINGS(ICM_20948_SPI_DEFAULT_FREQ, ICM_20948_SPI_DEFAULT_ORDER, ICM_20948_SPI_DEFAULT_MODE);

ICM_20948_SPI::ICM_20948_SPI(){

}

ICM_20948_Status_e ICM_20948_SPI::begin( uint8_t csPin, SPIClass &spiPort){
    // Associate
    _spi = &spiPort;
    _spisettings = ICM_20948_SPI_DEFAULT_SETTINGS;
	_cs = csPin;


	// Set pinmodes
	pinMode(_cs, OUTPUT);

	// Set pins to default positions
	digitalWrite(_cs, HIGH);

    // _spi->begin(); // Moved into user's sketch

    // Set up the serif
    _serif.write = ICM_20948_write_SPI;
    _serif.read = ICM_20948_read_SPI;
    _serif.user = (void*)this;              // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

    return ICM_20948_Stat_Ok;
}


















// serif functions for the I2C and SPI classes
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t* data, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    TwoWire* _i2c = ((ICM_20948_I2C*)user)->_i2c;             // Cast user field to ICM_20948_I2C type and extract the I2C interface pointer
    uint8_t addr = ((ICM_20948_I2C*)user)->_addr;
    if(_i2c == NULL){ return ICM_20948_Stat_ParamErr; }

    _i2c->beginTransmission(addr);
    _i2c->write(reg);
    _i2c->write(data, len);
    _i2c->endTransmission();

    // for( uint32_t indi = 0; indi < len; indi++ ){
    //     _i2c->beginTransmission(addr);
    //     _i2c->write(reg + indi);
    //     _i2c->write(*(data + indi) );
    //     _i2c->endTransmission();
    //     delay(10);
    // }

    // delay(10);

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t* buff, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    TwoWire* _i2c = ((ICM_20948_I2C*)user)->_i2c;
    uint8_t addr = ((ICM_20948_I2C*)user)->_addr;
    if(_i2c == NULL){ return ICM_20948_Stat_ParamErr; }

    _i2c->beginTransmission(addr);
    _i2c->write(reg);
    _i2c->endTransmission(false); // Send repeated start

    uint32_t offset = 0;
    uint32_t num_received = _i2c->requestFrom(addr, len);
    // while(_i2c->available()){
    //     if(len > 0){
    //         *(buff + offset) = _i2c->read();
    //         len--;
    //     }else{
    //         break;
    //     }
    // }

    if (num_received == len) {
        for(uint8_t i = 0; i < len; i++){ 
            buff[i] = _i2c->read();
        }
        return ICM_20948_Stat_Ok;
    } else {
        return ICM_20948_Stat_NoData;
    }

    if(len != 0){ return ICM_20948_Stat_NoData; }
    return ICM_20948_Stat_Ok;
}




ICM_20948_Status_e ICM_20948_write_SPI(uint8_t reg, uint8_t* data, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    SPIClass* _spi = ((ICM_20948_SPI*)user)->_spi;             // Cast user field to ICM_20948_SPI type and extract the SPI interface pointer
    uint8_t cs = ((ICM_20948_SPI*)user)->_cs;
    SPISettings spisettings = ((ICM_20948_SPI*)user)->_spisettings;
    if(_spi == NULL){ return ICM_20948_Stat_ParamErr; }

    digitalWrite(cs, LOW);
    // delayMicroseconds(5);
    _spi->beginTransaction(spisettings);
    _spi->transfer( ((reg & 0x7F) | 0x00) );
    //  SPI.transfer(data, len); // Can't do this thanks to Arduino's poor implementation
    for(uint32_t indi = 0; indi < len; indi++){
        _spi->transfer(*(data + indi));
    }
    _spi->endTransaction();
    // delayMicroseconds(5);
    digitalWrite(cs, HIGH);

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_SPI(uint8_t reg, uint8_t* buff, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    SPIClass* _spi = ((ICM_20948_SPI*)user)->_spi;
    uint8_t cs = ((ICM_20948_SPI*)user)->_cs;
    SPISettings spisettings = ((ICM_20948_SPI*)user)->_spisettings;
    if(_spi == NULL){ return ICM_20948_Stat_ParamErr; }

    digitalWrite(cs, LOW);
    //   delayMicroseconds(5);
    _spi->beginTransaction(spisettings);
    _spi->transfer( ((reg & 0x7F) | 0x80) );
    //  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
    for(uint32_t indi = 0; indi < len; indi++){
        *(buff + indi) = _spi->transfer(0x00);
    }
    _spi->endTransaction();
    //   delayMicroseconds(5);
    digitalWrite(cs, HIGH);

    return ICM_20948_Stat_Ok;
}
