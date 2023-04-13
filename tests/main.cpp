#include "ICM_20948.h"
#define I2C_BUS 1

ICM_20948_I2C myICM;  // Create an ICM_20948_I2C object

// Functions used to print the IMU data on stdout
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading);
void printScaledAGMT(ICM_20948_I2C* sensor);

int main() {
    uint8_t i2caddr = ICM_20948_I2C_ADDR_AD1;
    uint8_t i2cbus = I2C_BUS;
    bool initialized = false;
    while (!initialized) {
        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We
        // need to configure the sample mode etc. manually.
        myICM.begin(myICM._i2c, i2caddr, i2cbus);
        printf("Initialization of the sensor returned: ");
        std::cout << myICM.statusString() << std::endl;
        if (myICM.status != ICM_20948_Stat_Ok) {
            printf("Trying again...\r\n");
            delay(500);
        } else {
            initialized = true;
        }
    }

    printf("Device connected!\r\n");
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok) {
        printf("Software Reset returned: ");
        printf("%s\r\n", myICM.statusString());
    }
    delay(250);

    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok) {
        printf("setSampleMode returned: ");
        printf("%s\r\n", myICM.statusString());
    }
    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t
            myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm2;  // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                     // gpm2
                     // gpm4
                     // gpm8
                     // gpm16

    myFSS.g = dps1000;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                        // dps250
                        // dps500
                        // dps1000
                        // dps2000

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok) {
        printf("setFullScale returned: ");
        printf("%s\r\n", myICM.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;     // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw;  // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                     // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is
                                     // 265 hz acc_d111bw4_n136bw acc_d50bw4_n68bw8 acc_d23bw9_n34bw4 acc_d11bw5_n17bw
                                     // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth
                                     // is 8.3 hz acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5;  // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                       // gyr_d196bw6_n229bw8
                                       // gyr_d151bw8_n187bw6
                                       // gyr_d119bw5_n154bw3
                                       // gyr_d51bw2_n73bw3
                                       // gyr_d23bw9_n35bw9
                                       // gyr_d11bw6_n17bw8
                                       // gyr_d5bw7_n8bw9
                                       // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok) {
        printf("setDLPcfg returned: ");
        printf("%s\r\n", myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor
    // masks to these functions
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
    printf("Enable DLPF for Accelerometer returned: ");
    printf("%s\r\n", myICM.statusString(accDLPEnableStat));
    printf("Enable DLPF for Gyroscope returned: ");
    printf("%s\r\n", myICM.statusString(gyrDLPEnableStat));

    printf("Configuration complete!\r\n");
    while (1) {
        if (myICM.dataReady()) {
            auto start = std::chrono::system_clock::now();
            myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            printf(" Hz: %f", 1 / elapsed.count());
            printf("\r\n");

            // printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt
            // structure
            printScaledAGMT(&myICM);  // This function takes into account the scale settings from when the measurement
                                      // was made to calculate the values with units
            delay(10);
        } else {
            printf("Waiting for data\r\n");
            delay(500);
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

void printScaledAGMT(ICM_20948_I2C* sensor) {
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
