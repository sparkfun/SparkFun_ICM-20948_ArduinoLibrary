#include "ICM_20948.h"

// Forward Declarations
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t* data, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t* buff, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_write_SPI(uint8_t reg, uint8_t* buff, uint32_t len, void* user);
ICM_20948_Status_e ICM_20948_read_SPI(uint8_t reg, uint8_t* buff, uint32_t len, void* user);





// Base
ICM_20948::ICM_20948(){

}








// I2C
ICM_20948_I2C::ICM_20948_I2C(){

}

ICM_20948_Status_e ICM_20948_I2C::begin(TwoWire &wirePort, bool ad0val, uint8_t ad0pin){
    // Associate 
	_ad0 = ad0pin;
	_i2c = &wirePort;
	_ad0val = ad0val;

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
    _i2c->requestFrom(addr, len);
    while(_i2c->available()){
        if(len--){
            *(buff + offset) = _i2c->read();
        }
    }
    _i2c->endTransmission();

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
