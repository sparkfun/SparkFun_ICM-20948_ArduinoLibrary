#include "ICM_20948.h"
#define I2C_BUS 5
#define IMU_ADDRESS 0x68

ICM_20948_I2C myICM;  // Create an ICM_20948_I2C object

// Functions used to print the IMU data on stdout
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading);
void printScaledAGMT(ICM_20948_I2C* sensor);

int main() {
    uint8_t i2caddr = IMU_ADDRESS;
    uint8_t i2cbus = I2C_BUS;
    bool initialized = false;
    while (!initialized) {
        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We
        // need to configure the sample mode etc. manually.
        myICM.begin(i2cbus, i2caddr);
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

    bool success = true;  // Use success to show if the DMP configuration
                          // was successful

    // Initialize the DMP. initializeDMP is a weak function. You can
    // overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro +
    //    32-bit calibrated gyro) INV_ICM20948_SENSOR_RAW_ACCELEROMETER
    //    (16-bit accel) INV_ICM20948_SENSOR_RAW_GYROSCOPE (16-bit gyro +
    //    32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step
    //    Detector) INV_ICM20948_SENSOR_STEP_COUNTER (Pedometer Step
    //    Detector) INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit
    //    6-axis quaternion) INV_ICM20948_SENSOR_ROTATION_VECTOR (32-bit
    //    9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV
    //    + heading accuracy) INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD (32-bit
    //    calibrated compass) INV_ICM20948_SENSOR_GRAVITY (32-bit 6-axis
    //    quaternion) INV_ICM20948_SENSOR_LINEAR_ACCELERATION (16-bit accel
    //    + 32-bit 6-axis quaternion) INV_ICM20948_SENSOR_ORIENTATION
    //    (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP Game Rotation Vector sensor (Quat6)
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Enable additional sensors / features
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates
    // to FIFO. Setting value can be calculated as follows: Value = (DMP
    // running rate / ODR ) - 1 E.g. For a 225Hz ODR rate when DMP is
    // running at 255Hz, value = (225/225) - 1 = 0.
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) ==
    // ICM_20948_Stat_Ok);  // Set to 1Hz
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) ==
    // ICM_20948_Stat_Ok);  // Set to 1Hz

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success) {
        printf("DMP enabled!\r\n");
    } else {
        printf("Enable DMP failed!\r\n");
        printf("Please check that you have uncommented line 29 (#define "
               "ICM_20948_USE_DMP) in ICM_20948_C.h...\r\n");
        while (1)
            ;  // Do nothing more
    }
    while (1) {
        auto start = std::chrono::system_clock::now();
        // Read any DMP data waiting in the FIFO
        // Note:
        //    readDMPdataFromFIFO will return
        //    ICM_20948_Stat_FIFONoDataAvail if no data is available. If
        //    data is available, readDMPdataFromFIFO will attempt to
        //    read _one_ frame of DMP data. readDMPdataFromFIFO will
        //    return ICM_20948_Stat_FIFOIncompleteData if a frame was
        //    present but was incomplete readDMPdataFromFIFO will return
        //    ICM_20948_Stat_Ok if a valid frame was read.
        //    readDMPdataFromFIFO will return
        //    ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read
        //    _and_ the FIFO contains more (unread) data.
        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);

        if ((myICM.status == ICM_20948_Stat_Ok) ||
                (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data
                                                                     // available?
        {
            // printf("Received data! Header: 0x")); // Print the
            // header in HEX so we can see what data is arriving in
            // the FIFO if ( data.header < 0x1000)
            // SERIAL_PORT.print( "0" ); // Pad the zeros if (
            // data.header < 0x100) SERIAL_PORT.print( "0" ); if (
            // data.header < 0x10) SERIAL_PORT.print( "0" );
            // SERIAL_PORT.println( data.header, HEX );

            if ((data.header & DMP_header_bitmap_Quat6) > 0)  // Check for orientation data (Quat9)
            {
                // Q0 value is computed from this equation: Q0^2
                // + Q1^2 + Q2^2 + Q3^2 = 1. In case of drift,
                // the sum will not add to 1, therefore,
                // quaternion data need to be corrected with
                // right bias values. The quaternion data is
                // scaled by 2^30.

                // SERIAL_PORT.printf("Quat6 data is: Q1:%ld
                // Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1,
                // data.Quat6.Data.Q2, data.Quat6.Data.Q3);

                // Scale to +/- 1
                double q1 = ((double) data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double.
                                                                           // Divide by 2^30
                double q2 = ((double) data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double.
                                                                           // Divide by 2^30
                double q3 = ((double) data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double.
                                                                           // Divide by 2^30
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                printf(" Q0:");
                printf("%f", q0);
                printf(" Q1:");
                printf("%f", q1);
                printf(" Q2:");
                printf("%f", q2);
                printf(" Q3:");
                printf("%f", q3);
            }

            if ((data.header & DMP_header_bitmap_Accel) > 0)  // Check for Accel
            {
                float acc_x = (float) data.Raw_Accel.Data.X;  // Extract the raw
                                                              // accelerometer data
                float acc_y = (float) data.Raw_Accel.Data.Y;
                float acc_z = (float) data.Raw_Accel.Data.Z;

                printf(" Accel: X:");
                printf("%f", acc_x / 1000);
                printf(" Y:");
                printf("%f", acc_y / 1000);
                printf(" Z:");
                printf("%f", acc_z / 1000);
            }

            if ((data.header & DMP_header_bitmap_Gyro) > 0)  // Check for Gyro
            {
                float x = (float) data.Raw_Gyro.Data.X;  // Extract the raw gyro data
                float y = (float) data.Raw_Gyro.Data.Y;
                float z = (float) data.Raw_Gyro.Data.Z;

                printf(" Gyro: X:");
                printf("%f", x / 1000);
                printf(" Y:");
                printf("%f", y / 1000);
                printf(" Z:");
                printf("%f", z / 1000);
            }

            // if ((data.header & DMP_header_bitmap_Compass) > 0)
            ////Check for Compass
            //{
            //  float x = (float)data.Compass.Data.X;
            //  float y = (float)data.Compass.Data.Y;
            //  float z = (float)data.Compass.Data.Z;

            //  printf(" Compass: X:");
            //  printf("%f", x);
            //  printf(" Y:");
            //  printf("%f", y);
            //  printf(" Z:");
            //  printf("%f", z);
            //}
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            printf(" Hz: %f", 1 / elapsed.count());
            printf("\r\n");
        }

        if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is
                                                               // available then we
                                                               // should read it right
                                                               // away - and not delay
        {
            delay(10);
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
