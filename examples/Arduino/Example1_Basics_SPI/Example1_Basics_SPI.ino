#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h" // temporary

#define SERIAL_PORT Serial

#define CS_PIN 3

ICM_20948_SPI myICM;

void setup() {
  // put your setup code here, to run once:

  SERIAL_PORT.begin(115200);
//  while(!SERIAL_PORT){}; // NOTE: make sure while(!SERIAL_PORT) does not accidentally call Wire.begin a bunch of times

  SPI.begin();
  myICM.begin(CS_PIN, SPI);

  myICM._spi->beginTransaction(myICM._spisettings);
  myICM._spi->transfer( 0x00 );
  myICM._spi->endTransaction();
  

  while(!myICM.isConnected()){
    SERIAL_PORT.print("Could not find the device at chip select: ");
    SERIAL_PORT.print(myICM._cs);
    SERIAL_PORT.print(". Trying again");
    SERIAL_PORT.println();
    delay(500);
  }

  SERIAL_PORT.println("Device connected!");

  // For SPI we need to disable the I2C interface
  uint8_t user_ctrl = 0xFF;
  myICM.read(AGB0_REG_USER_CTRL, &user_ctrl, 1);
  Serial.println(user_ctrl, HEX);
  user_ctrl = 0x10;
  myICM.write(AGB0_REG_USER_CTRL, &user_ctrl, 1);

  delay(200);
  
  myICM.read(AGB0_REG_USER_CTRL, &user_ctrl, 1);
  Serial.println(user_ctrl, HEX);
//  while(1){};

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset( );
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Set Gyro and Accelerometer to a particular sample mode
  myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous ); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm2;     // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myFSS.g = dps2000;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );  
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up DLPF configuration
  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.a = acc_d473bw_n499bw;
  myDLPcfg.g = gyr_d361bw4_n376bw5;
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, false );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, false );
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  // Now wake the sensor up
  myICM.sleep( false );
  myICM.lowPower( false );

}

void loop() {
  // put your main code here, to run repeatedly:

  if( myICM.dataReady() ){
    myICM.getAGMT();
//    printRawAGMT( myICM.agmt );
    printScaledAGMT( myICM.agmt );

    delay(20);
    
  }else{
    Serial.println("0");
  }

}

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

float getAccMG( int16_t raw, uint8_t fss ){
  switch(fss){
    case 0 : return (((float)raw)/16.384); break;
    case 1 : return (((float)raw)/8.192); break;
    case 2 : return (((float)raw)/4.096); break;
    case 3 : return (((float)raw)/2.048); break;
    default : return 0; break;
  }
}

float getGyrDPS( int16_t raw, uint8_t fss ){
  switch(fss){
    case 0 : return (((float)raw)/131); break;
    case 1 : return (((float)raw)/65.5); break;
    case 2 : return (((float)raw)/32.8); break;
    case 3 : return (((float)raw)/16.4); break;
    default : return 0; break;
  }
}

float getMagUT( int16_t raw ){
//  return (((float)raw)*0.15);
// todo: this is a much more complicated formula, atually
}

float getTmpC( int16_t raw ){
  return (((float)raw)/333.87) + 21;
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("Scaled. Acc (mg) [ ");
  Serial.print( getAccMG(agmt.acc.axes.x, agmt.fss.a ) );
  Serial.print(", ");
  Serial.print( getAccMG(agmt.acc.axes.y, agmt.fss.a ) );
  Serial.print(", ");
  Serial.print( getAccMG(agmt.acc.axes.z, agmt.fss.a ) );
  Serial.print(" ], Gyr (DPS) [ ");
  Serial.print( getGyrDPS(agmt.gyr.axes.x, agmt.fss.g ) );
  Serial.print(", ");
  Serial.print( getGyrDPS(agmt.gyr.axes.y, agmt.fss.g ) );
  Serial.print(", ");
  Serial.print( getGyrDPS(agmt.gyr.axes.z, agmt.fss.g ) );
  Serial.print(" ], Mag (uT) [ ");
  Serial.print( getMagUT(agmt.mag.axes.x) );
  Serial.print(", ");
  Serial.print( getMagUT(agmt.mag.axes.y) );
  Serial.print(", ");
  Serial.print( getMagUT(agmt.mag.axes.z) );
  Serial.print(" ], Tmp (C) [ ");
  Serial.print( getTmpC(agmt.tmp.val) );
  Serial.print(" ]");
  Serial.println();
}
