/*

A C++ interface to the ICM-20948

*/

#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "util/ICM_20948_C.h"	// The C backbone

#include "Wire.h"   // Arduino support
#include "SPI.h"

class ICM_20948 {
private:
protected:
    ICM_20948_Device_t device;
public:
    
};



class ICM_20948_I2C {
private:
protected:
public:
    TwoWire*    _i2c;
    uint8_t     _addr;

    ICM_20948_I2C(); // Constructor


};


class ICM_20948_SPI {
private:
protected:
public:
    SPIClass*   _spi;
    SPISettings _spisettings;
    uint8_t     _cs;

    ICM_20948_SPI(); // Constructor

    ICM_20948_Status_e begin();

    read(  )
};





#endif /* _ICM_20948_H_ */