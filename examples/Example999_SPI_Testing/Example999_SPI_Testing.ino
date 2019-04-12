#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h" // Temporary


#include <SPI.h>

#define CS_PIN 2
#define SPI_CLK 1000000
SPISettings mySettings(SPI_CLK, MSBFIRST, SPI_MODE3);

ICM_20948_Status_e mywrite(uint8_t reg, uint8_t* data, uint32_t len, void* user);
ICM_20948_Status_e myread(uint8_t reg, uint8_t* buff, uint32_t len, void* user);


ICM_20948_Device_t myICM;
const ICM_20948_Serif_t mySerif = {
  mywrite, // mywrite
  myread, // myread
};

uint8_t myvar = ICM_20948_Sample_Mode_Continuous;

void setup() {
  // put your setup code here, to run once:


  // Perform platform initialization
  Serial.begin(115200);
  SPI.begin();

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Aha! The SPI initialization monster bytes again! Let's send one byte out to 'set' the pins into a good starting state
  SPI.beginTransaction(mySettings);
  SPI.transfer( 0x00 );
  SPI.endTransaction();
  // Note to self - this should be performed any time that the SPI_MODE changes, just to be safe
  
  // Link the serif
  ICM_20948_link_serif(&myICM, &mySerif);

  while(ICM_20948_check_id( &myICM ) != ICM_20948_Stat_Ok){
    Serial.println("whoami does not match. Halting...");
     delay(1000);
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  ICM_20948_sw_reset( &myICM );
  delay(250);

  // Set Gyro and Accelerometer to a particular sample mode
  ICM_20948_set_sample_mode( &myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous ); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myfss;
  myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  ICM_20948_set_full_scale( &myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss );

  // Set up DLPF configuration
  ICM_20948_set_dlpf_cfg    ( &myICM, ICM_20948_Internal_Acc, acc_d473bw_n499bw );
  ICM_20948_set_dlpf_cfg    ( &myICM, ICM_20948_Internal_Gyr, gyr_d361bw4_n376bw5 );

  // Choose whether or not to use DLPF
  ICM_20948_enable_dlpf    ( &myICM, ICM_20948_Internal_Acc, false );
  ICM_20948_enable_dlpf    ( &myICM, ICM_20948_Internal_Gyr, false );


  // Now wake the sensor up
  ICM_20948_sleep         ( &myICM, false );
  ICM_20948_low_power     ( &myICM, false );


}

void loop() {
  // put your main code here, to run repeatedly:

  delay(20);

//  if(ICM_20948_data_myready( &myICM ) == ICM_20948_Stat_Ok){
////    Serial.println("1");
//  }else{
////    Serial.println("0");
//  }

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_INT_STATUS_1_t reg;
  retval = ICM_20948_set_bank( &myICM, 0); // Must be in the right bank
  retval = myread( AGB0_REG_INT_STATUS_1, (uint8_t*)&reg, sizeof(ICM_20948_INT_STATUS_1_t), NULL);

  Serial.println(*((uint8_t*)&reg), BIN);
  

    
//    ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
//    getAGMT( &agmt );
//    printRawAGMT( agmt );
//    printScaledAGMT(agmt);

  

}

ICM_20948_Status_e mywrite(uint8_t reg, uint8_t* data, uint32_t len, void* user){
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI.beginTransaction(mySettings);
  SPI.transfer( ((reg & 0x7F) | 0x00) );
//  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
  for(uint32_t indi = 0; indi < len; indi++){
    SPI.transfer(*(data + indi));
  }
  SPI.endTransaction();
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e myread(uint8_t reg, uint8_t* buff, uint32_t len, void* user){
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI.beginTransaction(mySettings);
  SPI.transfer( ((reg & 0x7F) | 0x80) );
//  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
  for(uint32_t indi = 0; indi < len; indi++){
    *(buff + indi) = SPI.transfer(0x00);
  }
  SPI.endTransaction();
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);

  return ICM_20948_Stat_Ok;
}
//
//void setBank(uint8_t bank){
//  bank &= 0x03; // mask off values above 3
//  bank = bank << 4; // Shift into the right place
//  mywrite(REG_BANK_SEL, &bank, 1);
//}
//
//void getAGMT( ICM_20948_AGMT_t* p ){
//  const uint8_t numbytes = 14;
//  uint8_t buff[numbytes];
//  
//  setBank(0); 
//  myread( AGB0_REG_ACCEL_XOUT_H, buff, numbytes);
//
//  p->acc.i16bit[0] = ((buff[0] << 8) | (buff[1] & 0xFF));
//  p->acc.i16bit[1] = ((buff[2] << 8) | (buff[3] & 0xFF));
//  p->acc.i16bit[2] = ((buff[4] << 8) | (buff[5] & 0xFF));
//
//  p->gyr.i16bit[0] = ((buff[6] << 8) | (buff[7] & 0xFF));
//  p->gyr.i16bit[1] = ((buff[8] << 8) | (buff[9] & 0xFF));
//  p->gyr.i16bit[2] = ((buff[10] << 8) | (buff[11] & 0xFF));
//
//  // ToDo: get magnetometer myreadings
////  p->mag.i16bit[0] = ((buff[] << 8) | (buff[] & 0xFF));
////  p->mag.i16bit[1] = ((buff[] << 8) | (buff[] & 0xFF));
////  p->mag.i16bit[2] = ((buff[] << 8) | (buff[] & 0xFF));
//
//  p->tmp.i16bit = ((buff[12] << 8) | (buff[13] & 0xFF));
//
//
//
//  setBank(2);
//  ICM_20948_ACCEL_CONFIG_t acfg; 
//  myread( AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, 1*sizeof(acfg));
//  p->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range 
//                                // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
//  setBank(2);
//  ICM_20948_GYRO_CONFIG_1_t gcfg1;
//  myread( AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg1, 1*sizeof(gcfg1));
//  p->fss.g = gcfg1.GYRO_FS_SEL;
//
////  Serial.print("acc cfg: 0x"); Serial.print(*((uint8_t*)&acfg), HEX); Serial.println();
//
//  ICM_20948_ACCEL_CONFIG_2_t acfg2;
//  myread( AGB2_REG_ACCEL_CONFIG_2, (uint8_t*)&acfg2, 1*sizeof(acfg2));
////  Serial.print("acc cfg2: 0x"); Serial.print(*((uint8_t*)&acfg2), HEX); Serial.println();
//  
//}
//
//void fillZeros16b( int16_t val ){
////  if(val > 0){
////    Serial.print(" ");
////    if(val > 10000){ Serial.print("0"); }
////    if(val > 1000 ){ Serial.print("0"); }
////    if(val > 100  ){ Serial.print("0"); }
////    if(val > 10   ){ Serial.print("0"); }
////  }else{
////    if(val < 10000){ Serial.print("0"); }
////    if(val < 1000 ){ Serial.print("0"); }
////    if(val < 100  ){ Serial.print("0"); }
////    if(val < 10   ){ Serial.print("0"); }
////  }
//}
//
//void printRawAGMT( ICM_20948_AGMT_t agmt){
//  Serial.print("RAW. Acc [ ");
//  fillZeros16b( agmt.acc.i16bit[0] );
//  Serial.print( agmt.acc.i16bit[0] );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[1] );
//  Serial.print( agmt.acc.i16bit[1] );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[2] );
//  Serial.print( agmt.acc.i16bit[2] );
//  Serial.print(" ], Gyr [ ");
//  fillZeros16b( agmt.gyr.i16bit[0] );
//  Serial.print( agmt.gyr.i16bit[0] );
//  Serial.print(", ");
//  fillZeros16b( agmt.gyr.i16bit[1] );
//  Serial.print( agmt.gyr.i16bit[1] );
//  Serial.print(", ");
//  fillZeros16b( agmt.gyr.i16bit[2] );
//  Serial.print( agmt.gyr.i16bit[2] );
//  Serial.print(" ], Mag [ ");
//  fillZeros16b( agmt.mag.i16bit[0] );
//  Serial.print( agmt.mag.i16bit[0] );
//  Serial.print(", ");
//  fillZeros16b( agmt.mag.i16bit[1] );
//  Serial.print( agmt.mag.i16bit[1] );
//  Serial.print(", ");
//  fillZeros16b( agmt.mag.i16bit[2] );
//  Serial.print( agmt.mag.i16bit[2] );
//  Serial.print(" ], Tmp [ ");
//  fillZeros16b( agmt.tmp.i16bit );
//  Serial.print( agmt.tmp.i16bit );
//  Serial.print(" ]");
//  Serial.println();
//}
//
//uint16_t fifoAvailable( void ){
//  uint16_t retval = 0;
//  myread(AGB0_REG_FIFO_COUNT_H, (uint8_t*)&retval, 2);
//  return retval;
//}
//
//float getAccMG( int16_t raw, uint8_t fss ){
//  switch(fss){
//    case 0 : return (((float)raw)/16.384); break;
//    case 1 : return (((float)raw)/8.192); break;
//    case 2 : return (((float)raw)/4.096); break;
//    case 3 : return (((float)raw)/2.048); break;
//    default : return 0; break;
//  }
//}
//
//float getGyrDPS( int16_t raw, uint8_t fss ){
//  switch(fss){
//    case 0 : return (((float)raw)/131); break;
//    case 1 : return (((float)raw)/65.5); break;
//    case 2 : return (((float)raw)/32.8); break;
//    case 3 : return (((float)raw)/16.4); break;
//    default : return 0; break;
//  }
//}
//
//float getMagUT( int16_t raw ){
//  return (((float)raw)*0.15);
//}
//
//float getTmpC( int16_t raw ){
//  return (((float)raw)/333.87);
//}
//
//void printScaledAGMT( ICM_20948_AGMT_t agmt){
//  Serial.print("Scaled. Acc (mg) [ ");
//  fillZeros16b( agmt.acc.i16bit[0] );
//  Serial.print( getAccMG(agmt.acc.i16bit[0], agmt.fss.a ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[1] );
//  Serial.print( getAccMG(agmt.acc.i16bit[1], agmt.fss.a ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[2] );
//  Serial.print( getAccMG(agmt.acc.i16bit[2], agmt.fss.a ) );
//  Serial.print(" ], Gyr (DPS) [ ");
//  fillZeros16b( agmt.gyr.i16bit[0] );
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[0], agmt.fss.g ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.gyr.i16bit[1] );
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[1], agmt.fss.g ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.gyr.i16bit[2] );
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[2], agmt.fss.g ) );
//  Serial.print(" ], Mag (uT) [ ");
//  fillZeros16b( agmt.mag.i16bit[0] );
//  Serial.print( getMagUT(agmt.mag.i16bit[0]) );
//  Serial.print(", ");
//  fillZeros16b( agmt.mag.i16bit[1] );
//  Serial.print( getMagUT(agmt.mag.i16bit[1]) );
//  Serial.print(", ");
//  fillZeros16b( agmt.mag.i16bit[2] );
//  Serial.print( getMagUT(agmt.mag.i16bit[2]) );
//  Serial.print(" ], Tmp (C) [ ");
//  fillZeros16b( agmt.tmp.i16bit );
//  Serial.print( getTmpC(agmt.tmp.i16bit) );
//  Serial.print(" ]");
//  Serial.println();
//}
