#include "ICM_20948.h"

#define I2C_BUS 5
#define IMU_ADDRESS 0x68

enum class CommMode { SPI, I2C };
CommMode currentMode = CommMode::I2C; // Default to I2C

ICM_20948_I2C icm_i2c;
ICM_20948_SPI icm_spi(ICM_20948_SPI_DEFAULT_FREQ, 8);

// Functions used to print the IMU data on stdout
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading);
void printScaledAGMT(ICM_20948_I2C *sensor);
void printScaledAGMT(ICM_20948_SPI *sensor);

int main(int argc, char **argv) {
  bool initialized = false;

  // Parse command-line arguments to set the mode (I2C/SPI)
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--spi") == 0) {
      currentMode = CommMode::SPI;
    } else if (strcmp(argv[i], "--i2c") == 0) {
      currentMode = CommMode::I2C;
    }
  }

  if (currentMode == CommMode::SPI) {
    // SPI initialization
    while (!initialized) {
      icm_spi.begin("/dev/spidev2.0");
      std::cout << "Initialization of the sensor over SPI returned: "
                << icm_spi.statusString() << std::endl;
      if (icm_spi.status != ICM_20948_Stat_Ok) {
        printf("Trying again...\r\n");
        delay(500);
      } else {
        initialized = true;
      }
    }
  } else {
    // I2C initialization
    while (!initialized) {
      icm_i2c.begin(I2C_BUS, IMU_ADDRESS);
      std::cout << "Initialization of the sensor over I2C returned: "
                << icm_i2c.statusString() << std::endl;
      if (icm_i2c.status != ICM_20948_Stat_Ok) {
        printf("Trying again...\r\n");
        delay(500);
      } else {
        initialized = true;
      }
    }
  }

  printf("Device connected!\r\n");

  if (currentMode == CommMode::SPI) {
    icm_spi.swReset();
    if (icm_spi.status != ICM_20948_Stat_Ok) {
      printf("Software Reset returned: ");
      printf("%s\r\n", icm_spi.statusString());
    }
    delay(250);

    // Now wake the sensor up
    icm_spi.sleep(false);
    icm_spi.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for
    // which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    icm_spi.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                          ICM_20948_Sample_Mode_Continuous);
    if (icm_spi.status != ICM_20948_Stat_Ok) {
      printf("setSampleMode returned: ");
      printf("%s\r\n", icm_spi.statusString());
    }
    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that
                           // can contain values for all configurable sensors

    myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps1000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                       // dps250
                       // dps500
                       // dps1000
                       // dps2000

    icm_spi.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                         myFSS);
    if (icm_spi.status != ICM_20948_Stat_Ok) {
      printf("setFullScale returned: ");
      printf("%s\r\n", icm_spi.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg; // Similar to FSS, this uses a configuration
                                 // structure for the desired sensors
    myDLPcfg.a =
        acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                           // acc_d246bw_n265bw      - means 3db bandwidth is
                           // 246 hz and nyquist bandwidth is 265 hz
                           // acc_d111bw4_n136bw acc_d50bw4_n68bw8
                           // acc_d23bw9_n34bw4 acc_d11bw5_n17bw acc_d5bw7_n8bw3
                           // - means 3 db bandwidth is 5.7 hz and nyquist
                           // bandwidth is 8.3 hz acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    icm_spi.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                       myDLPcfg);
    if (icm_spi.status != ICM_20948_Stat_Ok) {
      printf("setDLPcfg returned: ");
      printf("%s\r\n", icm_spi.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that
    // it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat =
        icm_spi.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat =
        icm_spi.enableDLPF(ICM_20948_Internal_Gyr, false);
    printf("Enable DLPF for Accelerometer returned: ");
    printf("%s\r\n", icm_spi.statusString(accDLPEnableStat));
    printf("Enable DLPF for Gyroscope returned: ");
    printf("%s\r\n", icm_spi.statusString(gyrDLPEnableStat));

    printf("Configuration complete!\r\n");

    while (1) {
      if (icm_spi.dataReady()) {
        auto start = std::chrono::system_clock::now();
        icm_spi
            .getAGMT(); // The values are only updated when you call 'getAGMT'
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        printf(" Hz: %f", 1 / elapsed.count());
        printf("\r\n");

        // printRawAGMT( icm_spi.agmt ); // Uncomment this to see the raw
        // values, taken directly from the agmt structure
        printScaledAGMT(&icm_spi); // This function takes into account the scale
                                   // settings from when the measurement was
                                   // made to calculate the values with units
        delay(10);
      } else {
        printf("Waiting for data\r\n");
        delay(500);
      }
    }
  } else {
    icm_i2c.swReset();
    if (icm_i2c.status != ICM_20948_Stat_Ok) {
      printf("Software Reset returned: ");
      printf("%s\r\n", icm_i2c.statusString());
    }
    delay(250);

    // Now wake the sensor up
    icm_i2c.sleep(false);
    icm_i2c.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for
    // which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    icm_i2c.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                          ICM_20948_Sample_Mode_Continuous);
    if (icm_i2c.status != ICM_20948_Stat_Ok) {
      printf("setSampleMode returned: ");
      printf("%s\r\n", icm_i2c.statusString());
    }
    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that
                           // can contain values for all configurable sensors

    myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps1000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                       // dps250
                       // dps500
                       // dps1000
                       // dps2000

    icm_i2c.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                         myFSS);
    if (icm_i2c.status != ICM_20948_Stat_Ok) {
      printf("setFullScale returned: ");
      printf("%s\r\n", icm_i2c.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg; // Similar to FSS, this uses a configuration
                                 // structure for the desired sensors
    myDLPcfg.a =
        acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                           // acc_d246bw_n265bw      - means 3db bandwidth is
                           // 246 hz and nyquist bandwidth is 265 hz
                           // acc_d111bw4_n136bw acc_d50bw4_n68bw8
                           // acc_d23bw9_n34bw4 acc_d11bw5_n17bw acc_d5bw7_n8bw3
                           // - means 3 db bandwidth is 5.7 hz and nyquist
                           // bandwidth is 8.3 hz acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    icm_i2c.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                       myDLPcfg);
    if (icm_i2c.status != ICM_20948_Stat_Ok) {
      printf("setDLPcfg returned: ");
      printf("%s\r\n", icm_i2c.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that
    // it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat =
        icm_i2c.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat =
        icm_i2c.enableDLPF(ICM_20948_Internal_Gyr, false);
    printf("Enable DLPF for Accelerometer returned: ");
    printf("%s\r\n", icm_i2c.statusString(accDLPEnableStat));
    printf("Enable DLPF for Gyroscope returned: ");
    printf("%s\r\n", icm_i2c.statusString(gyrDLPEnableStat));

    printf("Configuration complete!\r\n");

    while (1) {
      if (icm_i2c.dataReady()) {
        auto start = std::chrono::system_clock::now();
        icm_i2c
            .getAGMT(); // The values are only updated when you call 'getAGMT'
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        printf(" Hz: %f", 1 / elapsed.count());
        printf("\r\n");

        // printRawAGMT( icm_i2c.agmt ); // Uncomment this to see the raw
        // values, taken directly from the agmt structure
        printScaledAGMT(&icm_i2c); // This function takes into account the scale
                                   // settings from when the measurement was
                                   // made to calculate the values with units
        delay(10);
      } else {
        printf("Waiting for data\r\n");
        delay(500);
      }
    }
  }
}

void printPaddedInt16b(int16_t val) {
  if (val > 0) {
    printf(" ");
    if (val < 10000) {
      printf("0");
    }
    if (val < 1000) {
      printf("0");
    }
    if (val < 100) {
      printf("0");
    }
    if (val < 10) {
      printf("0");
    }
  } else {
    printf("-");
    if (abs(val) < 10000) {
      printf("0");
    }
    if (abs(val) < 1000) {
      printf("0");
    }
    if (abs(val) < 100) {
      printf("0");
    }
    if (abs(val) < 10) {
      printf("0");
    }
  }
  printf("%d", abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
  printf("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  printf(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  printf(" ]");
  printf(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  printf(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  printf(" ]");
  printf(" ]");
  printf("\r\n");
}

void printFormattedFloat(float val, uint8_t leading) {
  float aval = abs(val);
  if (val < 0) {
    printf("-");
  } else {
    printf(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      printf("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    printf("%f", -val);
  } else {
    printf("%f", val);
  }
}

void printScaledAGMT(ICM_20948_SPI *sensor) {
  printf("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5);
  printf(", ");
  printFormattedFloat(sensor->accY(), 5);
  printf(", ");
  printFormattedFloat(sensor->accZ(), 5);
  printf(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5);
  printf(", ");
  printFormattedFloat(sensor->gyrY(), 5);
  printf(", ");
  printFormattedFloat(sensor->gyrZ(), 5);
  /*
  printf(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  */
  printf(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5);
  printf(" ]\r\n");
}

void printScaledAGMT(ICM_20948_I2C *sensor) {
  printf("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5);
  printf(", ");
  printFormattedFloat(sensor->accY(), 5);
  printf(", ");
  printFormattedFloat(sensor->accZ(), 5);
  printf(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5);
  printf(", ");
  printFormattedFloat(sensor->gyrY(), 5);
  printf(", ");
  printFormattedFloat(sensor->gyrZ(), 5);
  /*
  printf(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  */
  printf(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5);
  printf(" ]\r\n");
}
