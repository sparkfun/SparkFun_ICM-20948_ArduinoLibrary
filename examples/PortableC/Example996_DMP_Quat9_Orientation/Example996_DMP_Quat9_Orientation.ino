//#define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI // uncomment to use SPI instead

#define SERIAL_PORT Serial

#ifdef USE_SPI
  #define SPI_PORT SPI
  #define CS_PIN 2
  #define SPI_CLK 1000000
  SPISettings mySettings(SPI_CLK, MSBFIRST, SPI_MODE3);
#else
  #define WIRE_PORT Wire
  #define I2C_ADDR ICM_20948_I2C_ADDR_AD1
#endif

// These are the interface functions that you would define for your system. They can use either I2C or SPI,
// or really **any** protocol as long as it successfully reads / writes the desired data into the ICM in the end
#ifdef USE_SPI
  ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);
  ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
#else
  ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
  ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
#endif

// You declare a "Serial Interface" (serif) type and give it the pointers to your interface functions
#ifdef USE_SPI
const ICM_20948_Serif_t mySerif = {
    my_write_spi, // write
    my_read_spi,  // read
    NULL,         // this pointer is passed into your functions when they are called.
};
#else
const ICM_20948_Serif_t mySerif = {
    my_write_i2c, // write
    my_read_i2c,  // read
    NULL,
};
#endif

// Now declare the structure that represents the ICM.
ICM_20948_Device_t myICM;

void setup()
{
  // Perform platform initialization
  SERIAL_PORT.begin(115200);

#ifdef USE_SPI
  SPI_PORT.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  // Aha! The SPI initialization monster bytes again! Let's send one byte out to 'set' the pins into a good starting state
  SPI_PORT.beginTransaction(mySettings);
  SPI_PORT.transfer(0x00);
  SPI_PORT.endTransaction();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // Initialize myICM
  ICM_20948_init_struct(&myICM);

  // Link the serif
  ICM_20948_link_serif(&myICM, &mySerif);

  ICM_20948_Status_e stat = ICM_20948_Stat_Err;

  // Check the device presense
  while(1)
  {
    uint8_t whoami = 0x00;
    stat = ICM_20948_get_who_am_i(&myICM, &whoami);      

    if( (stat == ICM_20948_Stat_Ok) || (whoami == ICM_20948_WHOAMI) )
    {
      break ;
    }

    #ifndef QUAT_ANIMATION
      SERIAL_PORT.println("Unsuccessfull whoami check. Trying again...");
    #endif

    delay(500);
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  ICM_20948_sw_reset(&myICM);
  delay(100);

  // Wake the sensor up
  ICM_20948_sleep(&myICM, false);
  ICM_20948_low_power(&myICM, false);

  // Disabled DMP makes no sense for the DMP test sketch
  myICM._dmp_firmware_available = true;

  // DEFAULT STARTUP:

  // minimal magnetometer startup
  ICM_20948_i2c_master_passthrough(&myICM, false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  ICM_20948_i2c_master_enable(&myICM, true);

  // resetMag
  uint8_t SRST = 1;
  ICM_20948_i2c_master_single_w(&myICM, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &SRST);

  // After a ICM reset the Mag sensor may stop responding over the I2C master
  // Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < 10)
  {
    tries++;

    uint8_t whoami = 0x00;
    stat = ICM_20948_get_who_am_i(&myICM, &whoami);      

    if( (stat == ICM_20948_Stat_Ok) || (whoami == ICM_20948_WHOAMI) )
    {
      break ;
    }

    #ifndef QUAT_ANIMATION
      SERIAL_PORT.println("Magnetometer did not start. Trying again...");
    #endif

    ICM_20948_i2c_master_reset(&myICM); //Otherwise, reset the master I2C and try again

    delay(500);
  }

  if (tries == 10)
  {
    #ifndef QUAT_ANIMATION
      SERIAL_PORT.println("Magnetometer did not start after 10 tries");
    #endif
  }

  // Set Gyro and Accelerometer to a particular sample mode
  ICM_20948_set_sample_mode(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myfss = { .a = gpm2, .g = dps250 };
  ICM_20948_set_full_scale(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss);

  // Set up DLPF configuration
  ICM_20948_dlpcfg_t myDLPcfg = { .a = acc_d473bw_n499bw, .g = gyr_d361bw4_n376bw5 };
  ICM_20948_set_dlpf_cfg(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

  // Choose whether or not to use DLPF
  ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Acc, false);
  ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Gyr, false);

  // INITIALIZE DMP:
  
  ICM_20948_i2c_controller_configure_peripheral(&myICM, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, NULL);
  ICM_20948_i2c_controller_configure_peripheral(&myICM, 1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);

  // Set the I2C Master ODR configuration
  ICM_20948_set_bank(&myICM, 3);
  uint8_t mstODRconfig = 0x04;
  ICM_20948_execute_w(&myICM, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);

  // Configure clock source through PWR_MGMT_1
  ICM_20948_set_clock_source(&myICM, ICM_20948_Clock_Auto);

  // Enable accel and gyro sensors through PWR_MGMT_2
  ICM_20948_set_bank(&myICM, 0);
  uint8_t pwrMgmt2  = 0x40;
  ICM_20948_execute_w(&myICM, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);  

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  ICM_20948_set_sample_mode(&myICM, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);

  // Disable the FIFO
  ICM_20948_enable_FIFO(&myICM, false);

  // Disable the DMP
  ICM_20948_enable_DMP(&myICM, false);   

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS = { .a = gpm4, .g = dps2000}; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  ICM_20948_set_full_scale(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Gyr, true);

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  ICM_20948_set_bank(&myICM, 0);
  uint8_t zero = 0;
  ICM_20948_execute_w(&myICM, AGB0_REG_FIFO_EN_1, &zero, 1);  
  ICM_20948_execute_w(&myICM, AGB0_REG_FIFO_EN_2, &zero, 1);   

  // Turn off data ready interrupt through INT_ENABLE_1
  ICM_20948_INT_enable_t en; 
  ICM_20948_int_enable(&myICM, NULL, &en);
  en.RAW_DATA_0_RDY_EN = false;
  ICM_20948_int_enable(&myICM, &en, &en) ;

  // Reset FIFO through FIFO_RST
  ICM_20948_reset_FIFO(&myICM);

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV. ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2. ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  ICM_20948_smplrt_t mySmplrt = { .a = 19, .g = 19};
  ICM_20948_set_sample_rate(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);    

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  ICM_20948_set_dmp_start_address(&myICM, DMP_START_ADDRESS);
  
  // Now load the DMP firmware
  ICM_20948_firmware_load(&myICM);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  ICM_20948_set_dmp_start_address(&myICM, DMP_START_ADDRESS);

  // Set the Hardware Fix Disable register to 0x48
  ICM_20948_set_bank(&myICM, 0);
  uint8_t fix = 0x48;
  ICM_20948_execute_w(&myICM, AGB0_REG_HW_FIX_DISABLE, &fix, 1);  

  // Set the Single FIFO Priority Select register to 0xE4
  ICM_20948_set_bank(&myICM, 0);
  uint8_t fifoPrio = 0xE4;
  ICM_20948_execute_w(&myICM, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);  

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const uint8_t accScale[4] = {0x04, 0x00, 0x00, 0x00};
  inv_icm20948_write_mems(&myICM, ACC_SCALE, 4, accScale);
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const uint8_t accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  inv_icm20948_write_mems(&myICM, ACC_SCALE2, 4, accScale2);       

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const uint8_t mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const uint8_t mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  inv_icm20948_write_mems(&myICM, CPASS_MTX_00, 4, mountMultiplierPlus);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_01, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_02, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_10, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_11, 4, mountMultiplierMinus);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_12, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_20, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_21, 4, mountMultiplierZero);  
  inv_icm20948_write_mems(&myICM, CPASS_MTX_22, 4, mountMultiplierMinus); 

  // Configure the B2S Mounting Matrix
  const uint8_t b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  inv_icm20948_write_mems(&myICM, B2S_MTX_00, 4, b2sMountMultiplierPlus); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_01, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_02, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_10, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_11, 4, b2sMountMultiplierPlus); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_12, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_20, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_21, 4, b2sMountMultiplierZero); 
  inv_icm20948_write_mems(&myICM, B2S_MTX_22, 4, b2sMountMultiplierPlus);     

  // Configure the DMP Gyro Scaling Factor
  inv_icm20948_set_gyro_sf(&myICM, 19, 3);

  // Configure the Gyro full scale
  const uint8_t gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  inv_icm20948_write_mems(&myICM, GYRO_FULLSCALE, 4, gyroFullScale);     

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const uint8_t accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  inv_icm20948_write_mems(&myICM, ACCEL_ONLY_GAIN, 4, accelOnlyGain);  

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const uint8_t accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  inv_icm20948_write_mems(&myICM, ACCEL_ALPHA_VAR, 4, accelAlphaVar); 

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const uint8_t accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  inv_icm20948_write_mems(&myICM, ACCEL_A_VAR, 4, accelAVar); 

  // Configure the Accel Cal Rate
  const uint8_t accelCalRate[2] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  inv_icm20948_write_mems(&myICM, ACCEL_CAL_RATE, 2, accelCalRate); 

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const uint8_t compassRate[2] = {0x00, 0x45}; // 69Hz
  inv_icm20948_write_mems(&myICM, CPASS_TIME_BUFFER, 2, compassRate);    

  // Enable the DMP orientation sensor
  inv_icm20948_enable_dmp_sensor_int(&myICM, INV_ICM20948_SENSOR_ORIENTATION, true);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  inv_icm20948_set_dmp_sensor_period(&myICM, DMP_ODR_Reg_Quat9, 0);

  // Enable the FIFO
  ICM_20948_enable_FIFO(&myICM, true);

  // Enable the DMP
  ICM_20948_enable_DMP(&myICM, true);

  // Reset DMP
  ICM_20948_reset_DMP(&myICM);

  // Reset FIFO
  ICM_20948_reset_FIFO(&myICM);  
}

void loop()
{
  icm_20948_DMP_data_t data;

  ICM_20948_Status_e stat = ICM_20948_Stat_Err;
  stat = inv_icm20948_read_dmp_data(&myICM, &data);
  if ((stat == ICM_20948_Stat_Ok) || (stat == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

#ifndef QUAT_ANIMATION
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.print(F(" Accuracy:"));
      SERIAL_PORT.println(data.Quat9.Data.Accuracy);
#else
      // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
      SERIAL_PORT.print(F("{\"quat_w\":"));
      SERIAL_PORT.print(q0, 3);
      SERIAL_PORT.print(F(", \"quat_x\":"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(", \"quat_y\":"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(", \"quat_z\":"));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.println(F("}"));
#endif
    }
  }

  if (stat != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

///////////////////////////////////////////////////////////////
/* Here's where you actually define your interface functions */
///////////////////////////////////////////////////////////////

#ifdef USE_SPI
ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  SPI_PORT.beginTransaction(mySettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI_PORT.transfer(((reg & 0x7F) | 0x00));
  for (uint32_t indi = 0; indi < len; indi++)
  {
    SPI_PORT.transfer(*(data + indi));
  }
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI_PORT.endTransaction();

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  SPI_PORT.beginTransaction(mySettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI_PORT.transfer(((reg & 0x7F) | 0x80));
  for (uint32_t indi = 0; indi < len; indi++)
  {
    *(buff + indi) = SPI_PORT.transfer(0x00);
  }
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI_PORT.endTransaction();

  return ICM_20948_Stat_Ok;
}

#else

ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  WIRE_PORT.beginTransmission(I2C_ADDR);
  WIRE_PORT.write(reg);
  WIRE_PORT.write(data, len);
  WIRE_PORT.endTransmission();

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  WIRE_PORT.beginTransmission(I2C_ADDR);
  WIRE_PORT.write(reg);
  WIRE_PORT.endTransmission(false); // Send repeated start

  uint32_t num_received = WIRE_PORT.requestFrom(I2C_ADDR, len);
  if (num_received == len)
  {
    for (uint32_t i = 0; i < len; i++)
    {
      buff[i] = WIRE_PORT.read();
    }
  }

  return ICM_20948_Stat_Ok;
}

#endif
