#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h" // Temporary


ICM_20948_Status_e write(uint8_t reg, uint8_t* data, uint32_t len, void* user);
ICM_20948_Status_e read(uint8_t reg, uint8_t* buff, uint32_t len, void* user);
#define I2C_ADDR 0x69


ICM_20948_Device_t myICM;
const ICM_20948_Serif_t mySerif = {
  write, // write
  read, // read
  NULL,
};

void setup() {
  // put your setup code here, to run once:


  // Perform platform initialization
  Serial.begin(115200);

  Wire.begin();
//  Wire.setClock(400000);

  // Link the serif
  ICM_20948_link_serif(&myICM, &mySerif);


  while(1){
      Wire.beginTransmission(I2C_ADDR);
      Wire.write(0x00); // read from whoami reg assuming that we're in bank 0
      Wire.endTransmission(false); // Send repeated start

      uint32_t offset = 0;
      Serial.println(Wire.requestFrom(I2C_ADDR, 1));
      if(Wire.available()){
        Serial.print("Got: 0x");
        Serial.println(Wire.read());
      }else{
        Serial.println("Nothing available");
      }

      delay(1000);
  }

  

  while(ICM_20948_check_id( &myICM ) != ICM_20948_Stat_Ok){
    Serial.println("whoami does not match. Halting...");
     delay(1000);
  }

ICM_20948_Status_e stat = ICM_20948_Stat_Err;
uint8_t whoami = 0x00;
  while( (stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI) ) {
    whoami = 0x00;
    stat = ICM_20948_get_who_am_i(&myICM, &whoami);
    Serial.print("whoami does not match (0x");
    Serial.print(whoami, HEX);
    Serial.print("). Halting...");
    Serial.println();
     delay(1000);
  }
  
//  uint32_t count = 0;
//  while(1){
//    uint8_t whoami = 0x00;
//    ICM_20948_get_who_am_i(&myICM, &whoami);
//    Serial.print(whoami, HEX);
//    Serial.print(", ");
//    Serial.print(count++);
//    Serial.println();
//    delay(5);
//  }

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

  delay(1000);

//  if(ICM_20948_data_ready( &myICM ) == ICM_20948_Stat_Ok){
//    Serial.println("1");
//  }else{
//    Serial.println("0");
//  }

//  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
//  ICM_20948_INT_STATUS_1_t reg;
//  retval = ICM_20948_set_bank( &myICM, 0); // Must be in the right bank
//  retval = read( AGB0_REG_INT_STATUS_1, (uint8_t*)&reg, sizeof(ICM_20948_INT_STATUS_1_t));
//
//  if(retval == ICM_20948_Stat_Ok){
//    Serial.println(*((uint8_t*)&reg), BIN);
//  }else{
//    Serial.println("Oh no");
//  }

    
//    ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
//    if(getAGMT( &agmt ) == ICM_20948_Stat_Ok){
////    printRawAGMT( agmt );
//    printScaledAGMT(agmt);
//    }else{
//      Serial.println("Uh oh");
//    }

  

}

//ICM_20948_Status_e write(uint8_t reg, uint8_t* data, uint32_t len, void* user){
//  Wire.beginTransmission(I2C_ADDR);
//  Wire.write(reg);
//  Wire.write(data, len);
//  Wire.endTransmission();
//
//  return ICM_20948_Stat_Ok;
//}
//
//ICM_20948_Status_e read(uint8_t reg, uint8_t* buff, uint32_t len, void* user){
//  Wire.beginTransmission(I2C_ADDR);
//  Wire.write(reg);
//  Wire.endTransmission(false); // Send repeated start
//
//  Serial.print("length: ");
//  Serial.print(len);
//  Serial.println();
//
//  uint32_t offset = 0;
//  Serial.println(Wire.requestFrom(I2C_ADDR, len));
//  while(Wire.available()){
//    if(len--){
//      *(buff + offset) = Wire.read();
//      Serial.print(*(buff+offset));
//      Serial.print(", ");
//    }
//  }
//  Serial.println();
//  Wire.endTransmission();
//  
//  return ICM_20948_Stat_Ok;
//}
//
//void setBank(uint8_t bank){
//  bank &= 0x03; // mask off values above 3
//  bank = bank << 4; // Shift into the right place
//  write((uint8_t)REG_BANK_SEL, &bank, 1, NULL);
//}
//
//void getAGMT( ICM_20948_AGMT_t* p ){
//  const uint8_t numbytes = 14;
//  uint8_t buff[numbytes];
//  
//  setBank(0); 
//  read( (uint8_t)AGB0_REG_ACCEL_XOUT_H, buff, numbytes, NULL);
//
//  p->acc.i16bit[0] = ((buff[0] << 8) | (buff[1] & 0xFF));
//  p->acc.i16bit[1] = ((buff[2] << 8) | (buff[3] & 0xFF));
//  p->acc.i16bit[2] = ((buff[4] << 8) | (buff[5] & 0xFF));
//
//  p->gyr.i16bit[0] = ((buff[6] << 8) | (buff[7] & 0xFF));
//  p->gyr.i16bit[1] = ((buff[8] << 8) | (buff[9] & 0xFF));
//  p->gyr.i16bit[2] = ((buff[10] << 8) | (buff[11] & 0xFF));
//
//  // ToDo: get magnetometer readings
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
//  read( (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, 1*sizeof(acfg), NULL);
//  p->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range 
//                                // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
//  setBank(2);
//  ICM_20948_GYRO_CONFIG_1_t gcfg1;
//  read( (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg1, 1*sizeof(gcfg1), NULL);
//  p->fss.g = gcfg1.GYRO_FS_SEL;
//
////  Serial.print("acc cfg: 0x"); Serial.print(*((uint8_t*)&acfg), HEX); Serial.println();
//
//  ICM_20948_ACCEL_CONFIG_2_t acfg2;
//  read( (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t*)&acfg2, 1*sizeof(acfg2), NULL);
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
//  read((uint8_t)AGB0_REG_FIFO_COUNT_H, (uint8_t*)&retval, 2, NULL);
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
