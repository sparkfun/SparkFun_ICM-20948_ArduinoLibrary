/****************************************************************
 * Example5_DMP.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, February 9th 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * ** This example is based on the InvenSense Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for providing this.
 * 
 * ** Important note: by default the DMP functionality is disabled in the library. This is to save program memory.
 * ** The DMP firmware takes up 14290 Bytes of program memory. To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address.
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif


void setup() {

  SERIAL_PORT.begin(115200); // Start the serial console
  SERIAL_PORT.println(F("ICM-20948 Example"));

  delay(100);

  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();

  SERIAL_PORT.println(F("Press any key to continue..."));

  while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while( !initialized ){

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT );
#else
    myICM.begin( WIRE_PORT, AD0_VAL );
#endif

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

  SERIAL_PORT.println("Device connected!");

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration

  bool success = true; // Use success to show if the configuration was successful

  // Configure clock source through PWR_MGMT_1
  // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  success &= (myICM.setClockSource(ICM_20948_Clock_Auto) == ICM_20948_Stat_Ok); // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  uint8_t zero = 0;
  success &= (myICM.write(AGB0_REG_PWR_MGMT_2, &zero, 1) == ICM_20948_Stat_Ok); // Write one byte to the PWR_MGMT_2 register

  // Configure Gyro/Accel in Low power mode with LP_CONFIG
  success &= (myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ) == ICM_20948_Stat_Ok);

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
  myFSS.g = dps2000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  success &= (myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS ) == ICM_20948_Stat_Ok);

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  //success &= (myICM.intEnableOverflowFIFO( 0x1F ) == ICM_20948_Stat_Ok); // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  success &= (myICM.write(AGB0_REG_FIFO_EN_1, &zero, 1) == ICM_20948_Stat_Ok);
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  success &= (myICM.write(AGB0_REG_FIFO_EN_2, &zero, 1) == ICM_20948_Stat_Ok);

  // Turn off data ready interrupt through INT_ENABLE_1
  success &= (myICM.intEnableRawDataReady(false) == ICM_20948_Stat_Ok);

  // Reset FIFO through FIFO_RST
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 43; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 43 = 25Hz
  mySmplrt.a = 44; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 44 = 25Hz
  myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt );
  
  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  success &= (myICM.loadDMPFirmware() == ICM_20948_Stat_Ok);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x4000000 when FSR is 4g
  const unsigned char accScale[4] = {0x40, 0x00, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE, 4, &accScale[0]) == ICM_20948_Stat_Ok); // Write 0x4000000 to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x40000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE2, 4, &accScale2[0]) == ICM_20948_Stat_Ok); // Write 0x40000 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierMinus[4] = {0xC0, 0x00, 0x00, 0x00};
  success &= (myICM.writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Enable DMP interrupt
  //success &= (myICM.intEnableDMP(true) == ICM_20948_Stat_Ok);

  // Check success
  if( success )
    SERIAL_PORT.println("DMP enabled!");
  else
  {
    SERIAL_PORT.print("Enable DMP failed! Status is: ");
    SERIAL_PORT.println( myICM.statusString() );
  }
  
}

void loop()
{
  uint16_t count;
  myICM.getFIFOcount(&count);
  if( myICM.status == ICM_20948_Stat_Ok )
  {
    SERIAL_PORT.print("FIFO count is: ");
    SERIAL_PORT.println( count );
  }
  else
  {
    SERIAL_PORT.print("getFIFOcount failed! Status is: ");
    SERIAL_PORT.println( myICM.statusString() );
  }

  delay(1000);
}


// Below here are some helper functions to print the data nicely!

void printPaddedInt16b( int16_t val ){
  if(val > 0){
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  }else{
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
