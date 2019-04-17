#include "ICM_20948.h"

//#define USE_SPI       // Uncomment this to use SPI

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

  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){}; // NOTE: make sure while(!SERIAL_PORT) does not accidentally call Wire.begin a bunch of times

  bool initialized = false;
  while( !initialized ){

#ifdef USE_SPI
    SPI_PORT.begin( CS_PIN, SPI_PORT ); 
    myICM.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
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
}

void loop() {

  if( myICM.dataReady() ){
    myICM.getAGMT();
//    printRawAGMT( myICM.agmt );
    printScaledAGMT( myICM.agmt);
    delay(30);
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
  return (((float)raw)*0.15);
 //todo: this is a much more complicated formula, atually
}

float getTmpC( int16_t raw ){
  return (((float)raw)/333.87) + 21;
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
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( getAccMG(agmt.acc.axes.x, agmt.fss.a ), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getAccMG(agmt.acc.axes.y, agmt.fss.a ), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getAccMG(agmt.acc.axes.z, agmt.fss.a ), 5, 2 );
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( getGyrDPS(agmt.gyr.axes.x, agmt.fss.g ), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getGyrDPS(agmt.gyr.axes.y, agmt.fss.g ), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getGyrDPS(agmt.gyr.axes.z, agmt.fss.g ), 5, 2 );
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat( getMagUT(agmt.mag.axes.x), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getMagUT(agmt.mag.axes.y), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( getMagUT(agmt.mag.axes.z), 5, 2 );
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat( getTmpC(agmt.tmp.val), 5, 2 );
  Serial.print(" ]");
  Serial.println();
}
