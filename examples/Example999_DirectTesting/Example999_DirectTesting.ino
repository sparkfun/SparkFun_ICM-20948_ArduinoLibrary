#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h" // Temporary

#include <SPI.h>

#define CS_PIN 0
#define SPI_CLK 10000
SPISettings mySettings(SPI_CLK, MSBFIRST, SPI_MODE3);

ICM_20948_Status_e write(uint8_t reg, uint8_t* data, uint32_t len);
ICM_20948_Status_e read(uint8_t reg, uint8_t* buff, uint32_t len);


ICM_20948_Device_t myICM;
const ICM_20948_Serif_t mySerif = {
  write, // write
  read, // read
};

void setup() {
  // put your setup code here, to run once:


  // Perform platform initialization
  Serial.begin(115200);
  SPI.begin();

  while(!Serial){};

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Aha! The SPI initialization monster bytes again! Let's send one byte out to 'set' the pins into a good starting state
  SPI.beginTransaction(mySettings);
  SPI.transfer( 0x00 );
  SPI.endTransaction();
  // Note to self - this should be performed any time that the SPI_MODE changes, just to be safe
  
  // Link the serif
  ICM_20948_link_serif(&myICM, &mySerif);

  uint8_t whoami = 0x00;
  setBank(0); 
  read( AGB0_REG_WHO_AM_I, (uint8_t*)&whoami, sizeof(whoami));
  while(whoami != ICM_20948_WHOAMI){
    Serial.println("whoami does not match. Halting...");
     delay(1000);
     read( AGB0_REG_WHO_AM_I, (uint8_t*)&whoami, sizeof(whoami));
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  ICM_20948_sw_reset( &myICM );
  delay(250);

//    delay(1000);


//    setBank(0);
//    digitalWrite(CS_PIN, LOW);
//    delayMicroseconds(5);
//    SPI.beginTransaction(mySettings);
//    SPI.transfer( ((0x00 & 0x7F) | 0x80) ); // Start reading from register 0
//    
//    for(uint16_t indi = 0; indi < 10; indi++){
//      for(uint8_t indj = 0; indj < 16; indj++){
//        for(uint8_t indk = 0; indk < 16; indk++){
//          uint8_t val = SPI.transfer(0x00);
//          if(val < 100){ Serial.print(" "); }
//          if(val < 10) { Serial.print(" "); }
//          Serial.print(val);
//          Serial.print(", ");
//        }
//        Serial.println();
//      }
//      Serial.println();
//    }
//
//    SPI.endTransaction();
//    delayMicroseconds(5);
//    digitalWrite(CS_PIN, HIGH);



    // Try starting the accelerometer in ODR mode
    setBank(0);
    ICM_20948_LP_CONFIG_t reg;
    read( AGB0_REG_LP_CONFIG, (uint8_t*)&reg, sizeof(reg));
    reg.ACCEL_CYCLE = 1; // Set Accel to duty cycled mode
    reg.GYRO_CYCLE = 1; // Set Gyro to duty cycled mode
    write( AGB0_REG_LP_CONFIG, (uint8_t*)&reg, sizeof(reg));

    // Oh, you'll also need to make sure you arent in sleep mode!
    setBank(0);
    ICM_20948_PWR_MGMT_1_t pwr1;
    read( AGB0_REG_PWR_MGMT_1, (uint8_t*)&pwr1, sizeof(pwr1));
    pwr1.SLEEP = 0;
    pwr1.LP_EN = 0; // Hmm, well the LP_EN bit doesn't seem to affect the strange averaging problem...
    write( AGB0_REG_PWR_MGMT_1, (uint8_t*)&pwr1, sizeof(pwr1));

    // Let's set the full-scale selection for the accelerometer
    setBank(2);
    ICM_20948_ACCEL_CONFIG_t acfg;
    read( AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, sizeof(acfg));
    acfg.ACCEL_FS_SEL = 0;
    acfg.ACCEL_FCHOICE = 1; // Why does the DLPF affect the reported value? (the DC value, that is)
    acfg.ACCEL_DLPFCFG = 0;
    write( AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, sizeof(acfg));


    // Let's set the full-scale selection for the gyroscope
    setBank(2);
    ICM_20948_GYRO_CONFIG_1_t gcfg;
    read( AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg, sizeof(gcfg));
    gcfg.GYRO_FS_SEL = 0;
    gcfg.GYRO_FCHOICE = 0; // Why does the FCHOICE bit make the gyro stop working? (1 works, 0 doesn't)
    gcfg.GYRO_DLPFCFG = 6;
    write( AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg, sizeof(gcfg));

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(200);

//    uint16_t fifoBytes = fifoAvailable();
//    if(fifoBytes > 0){
//      Serial.print(fifoBytes);
//      Serial.print(" bytes in buffer. They are: {");
//      while(fifoAvailable()){
//        uint8_t bite = 0;
//        read(AGB0_REG_FIFO_R_W, &bite, 1);
//        Serial.print(bite);
//        Serial.print(", ");
//      }
//      Serial.println("}");
//    }
    
    
    ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
    getAGMT( &agmt );
//    printRawAGMT( agmt );
    printScaledAGMT(agmt);

//  Serial.print( agmt.acc.i16bit[0] );
//  Serial.print(", ");
//  Serial.print( agmt.acc.i16bit[1] );
//  Serial.print(", ");
//  Serial.print( agmt.acc.i16bit[2] );
//  Serial.println();
  

//  Serial.print( getAccMG(agmt.acc.i16bit[0], agmt.fss.a ) );
//  Serial.print(", ");
//  Serial.print( getAccMG(agmt.acc.i16bit[1], agmt.fss.a ) );
//  Serial.print(", ");
//  Serial.print( getAccMG(agmt.acc.i16bit[2], agmt.fss.a ) );
//  Serial.println();
    
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[0], agmt.fss.g ) );
//  Serial.print(", ");
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[1], agmt.fss.g ) );
//  Serial.print(", ");
//  Serial.print( getGyrDPS(agmt.gyr.i16bit[2], agmt.fss.g ) );
//  Serial.println();
  

}

ICM_20948_Status_e write(uint8_t reg, uint8_t* data, uint32_t len){
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.beginTransaction(mySettings);
  SPI.transfer( ((reg & 0x7F) | 0x00) );
//  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
  for(uint32_t indi = 0; indi < len; indi++){
    SPI.transfer(*(data + indi));
  }
  SPI.endTransaction();
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);
}

ICM_20948_Status_e read(uint8_t reg, uint8_t* buff, uint32_t len){
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.beginTransaction(mySettings);
  SPI.transfer( ((reg & 0x7F) | 0x80) );
//  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
  for(uint32_t indi = 0; indi < len; indi++){
    *(buff + indi) = SPI.transfer(0x00);
  }
  SPI.endTransaction();
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);
}

void setBank(uint8_t bank){
  bank &= 0x03; // mask off values above 3
  bank = bank << 4; // Shift into the right place
  write(REG_BANK_SEL, &bank, 1);
}

void getAGMT( ICM_20948_AGMT_t* p ){
  const uint8_t numbytes = 14;
  uint8_t buff[numbytes];
  
  setBank(0); 
  read( AGB0_REG_ACCEL_XOUT_H, buff, numbytes);

  p->acc.i16bit[0] = ((buff[0] << 8) | (buff[1] & 0xFF));
  p->acc.i16bit[1] = ((buff[2] << 8) | (buff[3] & 0xFF));
  p->acc.i16bit[2] = ((buff[4] << 8) | (buff[5] & 0xFF));

  p->gyr.i16bit[0] = ((buff[6] << 8) | (buff[7] & 0xFF));
  p->gyr.i16bit[1] = ((buff[8] << 8) | (buff[9] & 0xFF));
  p->gyr.i16bit[2] = ((buff[10] << 8) | (buff[11] & 0xFF));

  // ToDo: get magnetometer readings
//  p->mag.i16bit[0] = ((buff[] << 8) | (buff[] & 0xFF));
//  p->mag.i16bit[1] = ((buff[] << 8) | (buff[] & 0xFF));
//  p->mag.i16bit[2] = ((buff[] << 8) | (buff[] & 0xFF));

  p->tmp.i16bit = ((buff[12] << 8) | (buff[13] & 0xFF));



  setBank(2);
  ICM_20948_ACCEL_CONFIG_t acfg; 
  read( AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, 1*sizeof(acfg));
  p->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range 
                                // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
  setBank(2);
  ICM_20948_GYRO_CONFIG_1_t gcfg1;
  read( AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg1, 1*sizeof(gcfg1));
  p->fss.g = gcfg1.GYRO_FS_SEL;

//  Serial.print("acc cfg: 0x"); Serial.print(*((uint8_t*)&acfg), HEX); Serial.println();

  ICM_20948_ACCEL_CONFIG_2_t acfg2;
  read( AGB2_REG_ACCEL_CONFIG_2, (uint8_t*)&acfg2, 1*sizeof(acfg2));
//  Serial.print("acc cfg2: 0x"); Serial.print(*((uint8_t*)&acfg2), HEX); Serial.println();
  
}

void fillZeros16b( int16_t val ){
//  if(val > 0){
//    Serial.print(" ");
//    if(val > 10000){ Serial.print("0"); }
//    if(val > 1000 ){ Serial.print("0"); }
//    if(val > 100  ){ Serial.print("0"); }
//    if(val > 10   ){ Serial.print("0"); }
//  }else{
//    if(val < 10000){ Serial.print("0"); }
//    if(val < 1000 ){ Serial.print("0"); }
//    if(val < 100  ){ Serial.print("0"); }
//    if(val < 10   ){ Serial.print("0"); }
//  }
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("RAW. Acc [ ");
  fillZeros16b( agmt.acc.i16bit[0] );
  Serial.print( agmt.acc.i16bit[0] );
  Serial.print(", ");
  fillZeros16b( agmt.acc.i16bit[1] );
  Serial.print( agmt.acc.i16bit[1] );
  Serial.print(", ");
  fillZeros16b( agmt.acc.i16bit[2] );
  Serial.print( agmt.acc.i16bit[2] );
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
  Serial.print(" ]");
  Serial.println();
}

uint16_t fifoAvailable( void ){
  uint16_t retval = 0;
  read(AGB0_REG_FIFO_COUNT_H, (uint8_t*)&retval, 2);
  return retval;
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
  return (((float)raw)*0.15);
}

float getTmpC( int16_t raw ){
  return (((float)raw)/333.87);
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
//  Serial.print("Scaled. Acc (mg) [ ");
//  fillZeros16b( agmt.acc.i16bit[0] );
//  Serial.print( getAccMG(agmt.acc.i16bit[0], agmt.fss.a ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[1] );
//  Serial.print( getAccMG(agmt.acc.i16bit[1], agmt.fss.a ) );
//  Serial.print(", ");
//  fillZeros16b( agmt.acc.i16bit[2] );
  Serial.print( getAccMG(agmt.acc.i16bit[2], agmt.fss.a ) );
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
  Serial.println();
}
