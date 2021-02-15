#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h"
#include "util/AK09916_REGISTERS.h"

// Forward Declarations
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_write_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_read_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

// Base
ICM_20948::ICM_20948()
{
}

void ICM_20948::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true; //Should we print the commands we send? Good for debugging
}
void ICM_20948::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

// Debug Printing: based on gfvalvo's flash string helper code:
// https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void ICM_20948::debugPrint(const char *line)
{
  doDebugPrint([](const char *ptr) {return *ptr;}, line);
}

void ICM_20948::debugPrint(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);},
      (const char*) line);
}

void ICM_20948::debugPrintln(const char *line)
{
  doDebugPrint([](const char *ptr) {return *ptr;}, line, true);
}

void ICM_20948::debugPrintln(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);},
      (const char*) line, true);
}

void ICM_20948::doDebugPrint(char (*funct)(const char *), const char *string, bool newLine)
{
  if (_printDebug == false)
    return; // Bail if debugging is not enabled

  char ch;

  while ((ch = funct(string++)))
  {
    _debugSerial->print(ch);
  }

  if (newLine)
  {
    _debugSerial->println();
  }
}

void ICM_20948::debugPrintf(int i)
{
  if (_printDebug == true)
    _debugSerial->print(i);
}

void ICM_20948::debugPrintf(float f)
{
  if (_printDebug == true)
    _debugSerial->print(f);
}

void ICM_20948::debugPrintStatus(ICM_20948_Status_e stat)
{
    switch (stat)
    {
    case ICM_20948_Stat_Ok:
        debugPrint(F("All is well."));
        break;
    case ICM_20948_Stat_Err:
        debugPrint(F("General Error"));
        break;
    case ICM_20948_Stat_NotImpl:
        debugPrint(F("Not Implemented"));
        break;
    case ICM_20948_Stat_ParamErr:
        debugPrint(F("Parameter Error"));
        break;
    case ICM_20948_Stat_WrongID:
        debugPrint(F("Wrong ID"));
        break;
    case ICM_20948_Stat_InvalSensor:
        debugPrint(F("Invalid Sensor"));
        break;
    case ICM_20948_Stat_NoData:
        debugPrint(F("Data Underflow"));
        break;
    case ICM_20948_Stat_SensorNotSupported:
        debugPrint(F("Sensor Not Supported"));
        break;
    case ICM_20948_Stat_DMPNotSupported:
        debugPrint(F("DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?"));
        break;
    case ICM_20948_Stat_DMPVerifyFail:
        debugPrint(F("DMP Firmware Verification Failed"));
        break;
    case ICM_20948_Stat_FIFONoDataAvail:
        debugPrint(F("No FIFO Data Available"));
        break;
    case ICM_20948_Stat_FIFOMoreDataAvail:
        debugPrint(F("More FIFO Data Available"));
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader:
        debugPrint(F("Unrecognised DMP Header"));
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader2:
        debugPrint(F("Unrecognised DMP Header2"));
        break;
    case ICM_20948_Stat_InvalDMPRegister:
        debugPrint(F("Invalid DMP Register"));
        break;
    default:
        debugPrint(F("Unknown Status"));
        break;
    }
}

ICM_20948_AGMT_t ICM_20948::getAGMT(void)
{
    status = ICM_20948_get_agmt(&_device, &agmt);

    return agmt;
}

float ICM_20948::magX(void)
{
    return getMagUT(agmt.mag.axes.x);
}

float ICM_20948::magY(void)
{
    return getMagUT(agmt.mag.axes.y);
}

float ICM_20948::magZ(void)
{
    return getMagUT(agmt.mag.axes.z);
}

float ICM_20948::getMagUT(int16_t axis_val)
{
    return (((float)axis_val) * 0.15);
}

float ICM_20948::accX(void)
{
    return getAccMG(agmt.acc.axes.x);
}

float ICM_20948::accY(void)
{
    return getAccMG(agmt.acc.axes.y);
}

float ICM_20948::accZ(void)
{
    return getAccMG(agmt.acc.axes.z);
}

float ICM_20948::getAccMG(int16_t axis_val)
{
    switch (agmt.fss.a)
    {
    case 0:
        return (((float)axis_val) / 16.384);
        break;
    case 1:
        return (((float)axis_val) / 8.192);
        break;
    case 2:
        return (((float)axis_val) / 4.096);
        break;
    case 3:
        return (((float)axis_val) / 2.048);
        break;
    default:
        return 0;
        break;
    }
}

float ICM_20948::gyrX(void)
{
    return getGyrDPS(agmt.gyr.axes.x);
}

float ICM_20948::gyrY(void)
{
    return getGyrDPS(agmt.gyr.axes.y);
}

float ICM_20948::gyrZ(void)
{
    return getGyrDPS(agmt.gyr.axes.z);
}

float ICM_20948::getGyrDPS(int16_t axis_val)
{
    switch (agmt.fss.g)
    {
    case 0:
        return (((float)axis_val) / 131);
        break;
    case 1:
        return (((float)axis_val) / 65.5);
        break;
    case 2:
        return (((float)axis_val) / 32.8);
        break;
    case 3:
        return (((float)axis_val) / 16.4);
        break;
    default:
        return 0;
        break;
    }
}

float ICM_20948::temp(void)
{
    return getTempC(agmt.tmp.val);
}

float ICM_20948::getTempC(int16_t val)
{
    return (((float)val - 21) / 333.87) + 21;
}

const char *ICM_20948::statusString(ICM_20948_Status_e stat)
{
    ICM_20948_Status_e val;
    if (stat == ICM_20948_Stat_NUM)
    {
        val = status;
    }
    else
    {
        val = stat;
    }

    switch (val)
    {
    case ICM_20948_Stat_Ok:
        return "All is well.";
        break;
    case ICM_20948_Stat_Err:
        return "General Error";
        break;
    case ICM_20948_Stat_NotImpl:
        return "Not Implemented";
        break;
    case ICM_20948_Stat_ParamErr:
        return "Parameter Error";
        break;
    case ICM_20948_Stat_WrongID:
        return "Wrong ID";
        break;
    case ICM_20948_Stat_InvalSensor:
        return "Invalid Sensor";
        break;
    case ICM_20948_Stat_NoData:
        return "Data Underflow";
        break;
    case ICM_20948_Stat_SensorNotSupported:
        return "Sensor Not Supported";
        break;
    case ICM_20948_Stat_DMPNotSupported:
        return "DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?";
        break;
    case ICM_20948_Stat_DMPVerifyFail:
        return "DMP Firmware Verification Failed";
        break;
    case ICM_20948_Stat_FIFONoDataAvail:
        return "No FIFO Data Available";
        break;
    case ICM_20948_Stat_FIFOMoreDataAvail:
        return "More FIFO Data Available";
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader:
        return "Unrecognised DMP Header";
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader2:
        return "Unrecognised DMP Header2";
        break;
    case ICM_20948_Stat_InvalDMPRegister:
        return "Invalid DMP Register";
        break;
    default:
        return "Unknown Status";
        break;
    }
    return "None";
}

// Device Level
ICM_20948_Status_e ICM_20948::setBank(uint8_t bank)
{
    status = ICM_20948_set_bank(&_device, bank);
    return status;
}

ICM_20948_Status_e ICM_20948::swReset(void)
{
    status = ICM_20948_sw_reset(&_device);
    return status;
}

ICM_20948_Status_e ICM_20948::sleep(bool on)
{
    status = ICM_20948_sleep(&_device, on);
    return status;
}

ICM_20948_Status_e ICM_20948::lowPower(bool on)
{
    status = ICM_20948_low_power(&_device, on);
    return status;
}

ICM_20948_Status_e ICM_20948::setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
    status = ICM_20948_set_clock_source(&_device, source);
    return status;
}

ICM_20948_Status_e ICM_20948::checkID(void)
{
    status = ICM_20948_check_id(&_device);
    if (status != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::checkID: ICM_20948_check_id returned: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
    }
    return status;
}

bool ICM_20948::dataReady(void)
{
    status = ICM_20948_data_ready(&_device);
    if (status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    return false;
}

uint8_t ICM_20948::getWhoAmI(void)
{
    uint8_t retval = 0x00;
    status = ICM_20948_get_who_am_i(&_device, &retval);
    return retval;
}

bool ICM_20948::isConnected(void)
{
    status = checkID();
    if (status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    debugPrint(F("ICM_20948::isConnected: checkID returned: "));
    debugPrintStatus(status);
    debugPrintln(F(""));
    return false;
}

// Internal Sensor Options
ICM_20948_Status_e ICM_20948::setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
    status = ICM_20948_set_sample_mode(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
    return status;
}

ICM_20948_Status_e ICM_20948::setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
    status = ICM_20948_set_full_scale(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
    return status;
}

ICM_20948_Status_e ICM_20948::setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
    status = ICM_20948_set_dlpf_cfg(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
    return status;
}

ICM_20948_Status_e ICM_20948::enableDLPF(uint8_t sensor_id_bm, bool enable)
{
    status = ICM_20948_enable_dlpf(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
    return status;
}

ICM_20948_Status_e ICM_20948::setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
    status = ICM_20948_set_sample_rate(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
    return status;
}

// Interrupts on INT Pin
ICM_20948_Status_e ICM_20948::clearInterrupts(void)
{
    ICM_20948_INT_STATUS_t int_stat;
    ICM_20948_INT_STATUS_1_t int_stat_1;

    // read to clear interrupts
    status = ICM_20948_set_bank(&_device, 0);
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(ICM_20948_INT_STATUS_t));
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS_1, (uint8_t *)&int_stat_1, sizeof(ICM_20948_INT_STATUS_1_t));
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }

    // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntActiveLow(bool active_low)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_ACTL = active_low;                           // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntOpenDrain(bool open_drain)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_OPEN = open_drain;                           // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntLatch(bool latching)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_LATCH_EN = latching;                         // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntAnyReadToClear(bool enabled)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT_ANYRD_2CLEAR = enabled;                       // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncActiveLow(bool active_low)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.ACTL_FSYNC = active_low;                          // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncIntMode(bool interrupt_mode)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.FSYNC_INT_MODE_EN = interrupt_mode;               // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

//      All these individual functions will use a read->set->write method to leave other settings untouched
ICM_20948_Status_e ICM_20948::intEnableI2C(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.I2C_MST_INT_EN = enable;                        // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.I2C_MST_INT_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableDMP(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.DMP_INT1_EN = enable;                           // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.DMP_INT1_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnablePLL(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.PLL_RDY_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.PLL_RDY_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOM(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.WOM_INT_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.WOM_INT_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOF(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.REG_WOF_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.REG_WOF_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableRawDataReady(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.RAW_DATA_0_RDY_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableOverflowFIFO(uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWatermarkFIFO(uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::WOMThreshold(uint8_t threshold)
{
    ICM_20948_ACCEL_WOM_THR_t thr;                      // storage
    status = ICM_20948_wom_threshold(&_device, NULL, &thr); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    thr.WOM_THRESHOLD = threshold;                            // change the setting
    status = ICM_20948_wom_threshold(&_device, &thr, &thr); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (thr.WOM_THRESHOLD != threshold)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

// Interface Options
ICM_20948_Status_e ICM_20948::i2cMasterPassthrough(bool passthrough)
{
    status = ICM_20948_i2c_master_passthrough(&_device, passthrough);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterEnable(bool enable)
{
    status = ICM_20948_i2c_master_enable(&_device, enable);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterReset()
{
    status = ICM_20948_i2c_master_reset(&_device);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
    status = ICM_20948_i2c_controller_configure_peripheral(&_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap);
    return status;
}

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
//https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
ICM_20948_Status_e ICM_20948::i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
  return (i2cControllerConfigurePeripheral(peripheral, addr, reg, len, Rw, enable, data_only, grp, swap));
}

ICM_20948_Status_e ICM_20948::i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
    status = ICM_20948_i2c_controller_periph4_txn(&_device, addr, reg, data, len, Rw, send_reg_addr);
    return status;
}

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
//https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
ICM_20948_Status_e ICM_20948::i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  return (i2cControllerPeriph4Transaction(addr, reg, data, len, Rw, send_reg_addr));
}

ICM_20948_Status_e ICM_20948::i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
    status = ICM_20948_i2c_master_single_w(&_device, addr, reg, &data);
    return status;
}
uint8_t ICM_20948::i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
    uint8_t data = 0;
    status = ICM_20948_i2c_master_single_r(&_device, addr, reg, &data);
    if (status != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::i2cMasterSingleR: ICM_20948_i2c_master_single_r returned: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
    }
    return data;
}

ICM_20948_Status_e ICM_20948::startupDefault(bool minimal)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    retval = checkID();
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: checkID returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = swReset();
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: swReset returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }
    delay(50);

    retval = sleep(false);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: sleep returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = lowPower(false);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: lowPower returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    if (minimal) // Return now if minimal is true
    {
        debugPrintln(F("ICM_20948::startupDefault: minimal startup complete!"));
        return status;
    }

    retval = setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: setSampleMode returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

    ICM_20948_fss_t FSS;
    FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    retval = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: setFullScale returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d473bw_n499bw;
    dlpcfg.g = gyr_d361bw4_n376bw5;
    retval = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: setDLPFcfg returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = enableDLPF(ICM_20948_Internal_Acc, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: enableDLPF (Acc) returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = enableDLPF(ICM_20948_Internal_Gyr, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: enableDLPF (Gyr) returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = startupMagnetometer();
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupDefault: startupMagnetometer returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    return status;
}

// direct read/write
ICM_20948_Status_e ICM_20948::read(uint8_t reg, uint8_t *pdata, uint32_t len)
{
    status = ICM_20948_execute_r(&_device, reg, pdata, len);
    return (status);
}

ICM_20948_Status_e ICM_20948::write(uint8_t reg, uint8_t *pdata, uint32_t len)
{
    status = ICM_20948_execute_w(&_device, reg, pdata, len);
    return (status);
}

uint8_t ICM_20948::readMag(AK09916_Reg_Addr_e reg)
{
    uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
    return data;
}

ICM_20948_Status_e ICM_20948::writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
    status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
    return status;
}

// FIFO

ICM_20948_Status_e ICM_20948::enableFIFO(bool enable)
{
    status = ICM_20948_enable_FIFO(&_device, enable);
    return status;
}

ICM_20948_Status_e ICM_20948::resetFIFO(void)
{
  status = ICM_20948_reset_FIFO(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::setFIFOmode(bool snapshot)
{
    // Default to Stream (non-Snapshot) mode
    status = ICM_20948_set_FIFO_mode(&_device, snapshot);
    return status;
}

ICM_20948_Status_e ICM_20948::getFIFOcount(uint16_t *count)
{
  status = ICM_20948_get_FIFO_count(&_device, count);
  return status;
}

ICM_20948_Status_e ICM_20948::readFIFO(uint8_t *data)
{
  status = ICM_20948_read_FIFO(&_device, data);
  return status;
}

// DMP

ICM_20948_Status_e ICM_20948::enableDMP(bool enable)
{
    if (_device._dmp_firmware_available == true) // Should we attempt to enable the DMP?
    {
        status = ICM_20948_enable_DMP(&_device, enable == true ? 1 : 0 );
        return status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::resetDMP(void)
{
    status = ICM_20948_reset_DMP(&_device);
    return status;
}

ICM_20948_Status_e ICM_20948::loadDMPFirmware(void)
{
    if (_device._dmp_firmware_available == true) // Should we attempt to load the DMP firmware?
    {
        status = ICM_20948_firmware_load(&_device);
        return status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::setDMPstartAddress(unsigned short address)
{
    if (_device._dmp_firmware_available == true) // Should we attempt to set the start address?
    {
        status = ICM_20948_set_dmp_start_address(&_device, address);
        return status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to enable the sensor?
  {
    status = inv_icm20948_enable_dmp_sensor(&_device, sensor, enable == true ? 1 : 0);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to enable the sensor interrupt?
  {
    status = inv_icm20948_enable_dmp_sensor_int(&_device, sensor, enable == true ? 1 : 0);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to write to the DMP?
  {
    status = inv_icm20948_write_mems(&_device, reg, length, data);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::readDMPmems(unsigned short reg, unsigned int length, unsigned char *data)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to read from the DMP?
  {
    status = inv_icm20948_read_mems(&_device, reg, length, data);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::setDMPODRrate(enum DMP_ODR_Registers odr_reg, int rate)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to set the DMP ODR?
  {
    // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
  	// Setting value can be calculated as follows:
  	// Value = (DMP running rate (225Hz) / ODR ) - 1
  	// E.g. For a 25Hz ODR rate, value= (225/25) - 1 = 8.

    if (rate <= 0) // Check rate is valid
        rate = 1;
    if (rate > 225)
        rate = 225;
    uint16_t interval = (225 / rate) - 1;
    status = inv_icm20948_set_dmp_sensor_period(&_device, odr_reg, interval);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e ICM_20948::readDMPdataFromFIFO(icm_20948_DMP_data_t *data)
{
  if (_device._dmp_firmware_available == true) // Should we attempt to set the data from the FIFO?
  {
    status = inv_icm20948_read_dmp_data(&_device, data);
    return status;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

// I2C
ICM_20948_I2C::ICM_20948_I2C()
{
}

ICM_20948_Status_e ICM_20948_I2C::begin(TwoWire &wirePort, bool ad0val, uint8_t ad0pin)
{
    // Associate
    _ad0 = ad0pin;
    _i2c = &wirePort;
    _ad0val = ad0val;

    _addr = ICM_20948_I2C_ADDR_AD0;
    if (_ad0val)
    {
        _addr = ICM_20948_I2C_ADDR_AD1;
    }

    // Set pinmodes
    if (_ad0 != ICM_20948_ARD_UNUSED_PIN)
    {
        pinMode(_ad0, OUTPUT);
    }

    // Set pins to default positions
    if (_ad0 != ICM_20948_ARD_UNUSED_PIN)
    {
        digitalWrite(_ad0, _ad0val);
    }

    // _i2c->begin(); // Moved into user's sketch

    // Set up the serif
    _serif.write = ICM_20948_write_I2C;
    _serif.read = ICM_20948_read_I2C;
    _serif.user = (void *)this; // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

#if defined(ICM_20948_USE_DMP)
    _device._dmp_firmware_available = true; // Initialize _dmp_firmware_available
#else
    _device._dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

    _device._firmware_loaded = false; // Initialize _firmware_loaded
    _device._last_bank = 255;  // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
    _device._last_mems_bank = 255;  // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.

    // Perform default startup
    // Do a minimal startupDefault if using the DMP. User can always call startupDefault(false) manually if required.
    status = startupDefault(_device._dmp_firmware_available);
    if (status != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948_I2C::begin: startupDefault returned: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
    }
    return status;
}

ICM_20948_Status_e ICM_20948::startupMagnetometer(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
    i2cMasterEnable(true);

    //After a ICM reset the Mag sensor may stop responding over the I2C master
    //Reset the Master I2C until it responds
    uint8_t tries = 0;
    while (tries < MAX_MAGNETOMETER_STARTS)
    {
        tries++;

        //See if we can read the WhoIAm register correctly
        retval = magWhoIAm();
        if (retval == ICM_20948_Stat_Ok)
            break; //WIA matched!

        i2cMasterReset(); //Otherwise, reset the master I2C and try again

        delay(10);
    }

    if (tries == MAX_MAGNETOMETER_STARTS)
    {
        debugPrint(F("ICM_20948::startupMagnetometer: reached MAX_MAGNETOMETER_STARTS ("));
        debugPrintf((int)MAX_MAGNETOMETER_STARTS);
        debugPrintln(F("). Returning ICM_20948_Stat_WrongID"));
        status = ICM_20948_Stat_WrongID;
        return status;
    }
    else
    {
      debugPrint(F("ICM_20948::startupMagnetometer: successful magWhoIAm after "));
      debugPrintf((int)tries);
      if (tries == 1)
        debugPrintln(F(" try"));
      else
        debugPrintln(F(" tries"));
    }

    //Set up magnetometer
    AK09916_CNTL2_Reg_t reg;
    reg.MODE = AK09916_mode_cont_100hz;
    retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupMagnetometer: writeMag returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    retval = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::startupMagnetometer: i2cMasterConfigurePeripheral returned: "));
        debugPrintStatus(retval);
        debugPrintln(F(""));
        status = retval;
        return status;
    }

    return status;
}

ICM_20948_Status_e ICM_20948::magWhoIAm(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    uint8_t whoiam1, whoiam2;
    whoiam1 = readMag(AK09916_REG_WIA1);
    // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
    // i2cMasterSingleR updates status so it is OK to set retval to status here
    retval = status;
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
        debugPrintf((int)whoiam1);
        debugPrint(F(" (should be 72) readMag set status to: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
        return retval;
    }
    whoiam2 = readMag(AK09916_REG_WIA2);
    // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
    // i2cMasterSingleR updates status so it is OK to set retval to status here
    retval = status;
    if (retval != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
        debugPrintf((int)whoiam1);
        debugPrint(F(" (should be 72) whoiam2: "));
        debugPrintf((int)whoiam2);
        debugPrint(F(" (should be 9) readMag set status to: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
        return retval;
    }

    if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
    {
        retval = ICM_20948_Stat_Ok;
        status = retval;
        return status;
    }

    debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
    debugPrintf((int)whoiam1);
    debugPrint(F(" (should be 72) whoiam2: "));
    debugPrintf((int)whoiam2);
    debugPrintln(F(" (should be 9). Returning ICM_20948_Stat_WrongID"));

    retval = ICM_20948_Stat_WrongID;
    status = retval;
    return status;
}

// SPI

// SPISettings ICM_20948_SPI_DEFAULT_SETTINGS(ICM_20948_SPI_DEFAULT_FREQ, ICM_20948_SPI_DEFAULT_ORDER, ICM_20948_SPI_DEFAULT_MODE);

ICM_20948_SPI::ICM_20948_SPI()
{
}

ICM_20948_Status_e ICM_20948_SPI::begin(uint8_t csPin, SPIClass &spiPort, uint32_t SPIFreq)
{
    if (SPIFreq > 7000000)
        SPIFreq = 7000000; // Limit SPI frequency to 7MHz

    // Associate
    _spi = &spiPort;
    _spisettings = SPISettings(SPIFreq, ICM_20948_SPI_DEFAULT_ORDER, ICM_20948_SPI_DEFAULT_MODE);
    _cs = csPin;

    // Set pinmodes
    pinMode(_cs, OUTPUT);

    // Set pins to default positions
    digitalWrite(_cs, HIGH);

    // _spi->begin(); // Moved into user's sketch

    // 'Kickstart' the SPI hardware.
    _spi->beginTransaction(_spisettings);
    _spi->transfer(0x00);
    _spi->endTransaction();

    // Set up the serif
    _serif.write = ICM_20948_write_SPI;
    _serif.read = ICM_20948_read_SPI;
    _serif.user = (void *)this; // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

#if defined(ICM_20948_USE_DMP)
    _device._dmp_firmware_available = true; // Initialize _dmp_firmware_available
#else
    _device._dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

    _device._firmware_loaded = false; // Initialize _firmware_loaded
    _device._last_bank = 255;  // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
    _device._last_mems_bank = 255;  // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.

    // Perform default startup
    // Do a minimal startupDefault if using the DMP. User can always call startupDefault(false) manually if required.
    status = startupDefault(_device._dmp_firmware_available);
    if (status != ICM_20948_Stat_Ok)
    {
        debugPrint(F("ICM_20948_SPI::begin: startupDefault returned: "));
        debugPrintStatus(status);
        debugPrintln(F(""));
    }

    return status;
}

// serif functions for the I2C and SPI classes
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    TwoWire *_i2c = ((ICM_20948_I2C *)user)->_i2c; // Cast user field to ICM_20948_I2C type and extract the I2C interface pointer
    uint8_t addr = ((ICM_20948_I2C *)user)->_addr;
    if (_i2c == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    _i2c->beginTransmission(addr);
    _i2c->write(reg);
    _i2c->write(data, (uint8_t)len);
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

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    TwoWire *_i2c = ((ICM_20948_I2C *)user)->_i2c;
    uint8_t addr = ((ICM_20948_I2C *)user)->_addr;
    if (_i2c == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

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

    if (num_received == len)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            buff[i] = _i2c->read();
        }
        return ICM_20948_Stat_Ok;
    }
    else
    {
        return ICM_20948_Stat_NoData;
    }

    if (len != 0)
    {
        return ICM_20948_Stat_NoData;
    }
    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_write_SPI(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    SPIClass *_spi = ((ICM_20948_SPI *)user)->_spi; // Cast user field to ICM_20948_SPI type and extract the SPI interface pointer
    uint8_t cs = ((ICM_20948_SPI *)user)->_cs;
    SPISettings spisettings = ((ICM_20948_SPI *)user)->_spisettings;
    if (_spi == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states even when sharing the SPI bus with devices that use other modes
    _spi->beginTransaction(spisettings);
    _spi->transfer(0x00);
    _spi->endTransaction();

    _spi->beginTransaction(spisettings);
    digitalWrite(cs, LOW);
    // delayMicroseconds(5);
    _spi->transfer(((reg & 0x7F) | 0x00));
    //  SPI.transfer(data, len); // Can't do this thanks to Arduino's poor implementation
    for (uint32_t indi = 0; indi < len; indi++)
    {
        _spi->transfer(*(data + indi));
    }
    // delayMicroseconds(5);
    digitalWrite(cs, HIGH);
    _spi->endTransaction();

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    SPIClass *_spi = ((ICM_20948_SPI *)user)->_spi;
    uint8_t cs = ((ICM_20948_SPI *)user)->_cs;
    SPISettings spisettings = ((ICM_20948_SPI *)user)->_spisettings;
    if (_spi == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states
    _spi->beginTransaction(spisettings);
    _spi->transfer(0x00);
    _spi->endTransaction();

    _spi->beginTransaction(spisettings);
    digitalWrite(cs, LOW);
    //   delayMicroseconds(5);
    _spi->transfer(((reg & 0x7F) | 0x80));
    //  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
    for (uint32_t indi = 0; indi < len; indi++)
    {
        *(buff + indi) = _spi->transfer(0x00);
    }
    //   delayMicroseconds(5);
    digitalWrite(cs, HIGH);
    _spi->endTransaction();

    return ICM_20948_Stat_Ok;
}
