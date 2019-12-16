/*

A C++ interface to the ICM-20948

*/

#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "util/ICM_20948_C.h" // The C backbone
#include "util/AK09916_REGISTERS.h"

#include "Arduino.h" // Arduino support
#include "Wire.h"
#include "SPI.h"

#define ICM_20948_ARD_UNUSED_PIN 0xFF

// Base
class ICM_20948
{
private:
protected:
    ICM_20948_Device_t _device;

    float getTempC(int16_t val);
    float getGyrDPS(int16_t axis_val);
    float getAccMG(int16_t axis_val);
    float getMagUT(int16_t axis_val);

public:
    ICM_20948(); // Constructor

    ICM_20948_AGMT_t agmt;          // Acceleometer, Gyroscope, Magenetometer, and Temperature data
    ICM_20948_AGMT_t getAGMT(void); // Updates the agmt field in the object and also returns a copy directly

    float magX(void); // micro teslas
    float magY(void); // micro teslas
    float magZ(void); // micro teslas

    float accX(void); // milli g's
    float accY(void); // milli g's
    float accZ(void); // milli g's

    float gyrX(void); // degrees per second
    float gyrY(void); // degrees per second
    float gyrZ(void); // degrees per second

    float temp(void); // degrees celsius

    ICM_20948_Status_e status;                                              // Status from latest operation
    const char *statusString(ICM_20948_Status_e stat = ICM_20948_Stat_NUM); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

    // Device Level
    ICM_20948_Status_e setBank(uint8_t bank);                                // Sets the bank
    ICM_20948_Status_e swReset(void);                                        // Performs a SW reset
    ICM_20948_Status_e sleep(bool on = false);                               // Set sleep mode for the chip
    ICM_20948_Status_e lowPower(bool on = true);                             // Set low power mode for the chip
    ICM_20948_Status_e setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
    ICM_20948_Status_e checkID(void);                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

    bool dataReady(void);    // Returns 'true' if data is ready
    uint8_t getWhoAmI(void); // Return whoami in out prarmeter
    bool isConnected(void);  // Returns true if communications with the device are sucessful

    // Internal Sensor Options
    ICM_20948_Status_e setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
    ICM_20948_Status_e setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss);
    ICM_20948_Status_e setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
    ICM_20948_Status_e enableDLPF(uint8_t sensor_id_bm, bool enable);
    ICM_20948_Status_e setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

    // Interrupts on INT and FSYNC Pins
    ICM_20948_Status_e clearInterrupts(void);

    ICM_20948_Status_e cfgIntActiveLow(bool active_low);
    ICM_20948_Status_e cfgIntOpenDrain(bool open_drain);
    ICM_20948_Status_e cfgIntLatch(bool latching);         // If not latching then the interrupt is a 50 us pulse
    ICM_20948_Status_e cfgIntAnyReadToClear(bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
    ICM_20948_Status_e cfgFsyncActiveLow(bool active_low);
    ICM_20948_Status_e cfgFsyncIntMode(bool interrupt_mode); // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

    ICM_20948_Status_e intEnableI2C(bool enable);
    ICM_20948_Status_e intEnableDMP(bool enable);
    ICM_20948_Status_e intEnablePLL(bool enable);
    ICM_20948_Status_e intEnableWOM(bool enable);
    ICM_20948_Status_e intEnableWOF(bool enable);
    ICM_20948_Status_e intEnableRawDataReady(bool enable);
    ICM_20948_Status_e intEnableOverflowFIFO(uint8_t bm_enable);
    ICM_20948_Status_e intEnableWatermarkFIFO(uint8_t bm_enable);

    // Interface Options
    ICM_20948_Status_e i2cMasterPassthrough(bool passthrough = true);
    ICM_20948_Status_e i2cMasterEnable(bool enable = true);
    ICM_20948_Status_e i2cMasterReset();

    //Used for configuring slaves 0-3
    ICM_20948_Status_e i2cMasterConfigureSlave(uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false);
    ICM_20948_Status_e i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

    //Used for configuring the Magnetometer
    ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
    uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg);

    // Default Setup
    ICM_20948_Status_e startupDefault(void);

    // direct read/write
    ICM_20948_Status_e read(uint8_t reg, uint8_t *pdata, uint32_t len);
    ICM_20948_Status_e write(uint8_t reg, uint8_t *pdata, uint32_t len);

    //Mag specific
    ICM_20948_Status_e startupMagnetometer(void);
    ICM_20948_Status_e magWhoIAm(void);
    uint8_t readMag(AK09916_Reg_Addr_e reg);
    ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata);
};

// I2C

// Forward declarations of TwoWire and Wire for board/variant combinations that don't have a default 'SPI'
class TwoWire;
extern TwoWire Wire;

class ICM_20948_I2C : public ICM_20948
{
private:
protected:
public:
    TwoWire *_i2c;
    uint8_t _addr;
    uint8_t _ad0;
    bool _ad0val;
    ICM_20948_Serif_t _serif;

    ICM_20948_I2C(); // Constructor

    virtual ICM_20948_Status_e begin(TwoWire &wirePort = Wire, bool ad0val = true, uint8_t ad0pin = ICM_20948_ARD_UNUSED_PIN);
};

// SPI
#define ICM_20948_SPI_DEFAULT_FREQ 4000000
#define ICM_20948_SPI_DEFAULT_ORDER MSBFIRST
#define ICM_20948_SPI_DEFAULT_MODE SPI_MODE0

// Forward declarations of SPIClass and SPI for board/variant combinations that don't have a default 'SPI'
class SPIClass;
extern SPIClass SPI;

class ICM_20948_SPI : public ICM_20948
{
private:
protected:
public:
    SPIClass *_spi;
    SPISettings _spisettings;
    uint8_t _cs;
    ICM_20948_Serif_t _serif;

    ICM_20948_SPI(); // Constructor

    ICM_20948_Status_e begin(uint8_t csPin, SPIClass &spiPort = SPI, uint32_t SPIFreq = ICM_20948_SPI_DEFAULT_FREQ);
};

#endif /* _ICM_20948_H_ */