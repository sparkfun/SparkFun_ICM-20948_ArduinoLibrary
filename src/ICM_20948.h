/*

A C++ interface to the ICM-20948

*/

#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "util/ICM_20948_C.h"	// The C backbone

#include "Arduino.h"   // Arduino support
#include "Wire.h"
#include "SPI.h"

#define ICM_20948_ARD_UNUSED_PIN 0xFF


// Base
class ICM_20948 {
private:
protected:
    ICM_20948_Device_t _device;
public:
    
    ICM_20948();
};


// I2C
class ICM_20948_I2C : public ICM_20948 {
private:
protected:
public:
    TwoWire*                _i2c;
    uint8_t                 _addr;
    uint8_t                 _ad0;
    bool                    _ad0val;
    ICM_20948_Serif_t       _serif;

    ICM_20948_I2C(); // Constructor

    ICM_20948_Status_e begin(TwoWire &wirePort = Wire, bool ad0val = true, uint8_t ad0pin = ICM_20948_ARD_UNUSED_PIN);

};



// SPI
#define ICM_20948_SPI_DEFAULT_FREQ 1000000
#define ICM_20948_SPI_DEFAULT_ORDER MSBFIRST
#define ICM_20948_SPI_DEFAULT_MODE SPI_MODE3

class ICM_20948_SPI : public ICM_20948 {
private:
protected:
public:
    SPIClass*               _spi;
    SPISettings             _spisettings;
    uint8_t                 _cs;
    ICM_20948_Serif_t       _serif;

    ICM_20948_SPI(); // Constructor

    ICM_20948_Status_e begin( uint8_t csPin, SPIClass &spiPort);

    // read(  )
};





#endif /* _ICM_20948_H_ */