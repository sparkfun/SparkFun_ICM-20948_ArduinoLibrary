# InvenSense Digital Motion Processor (DMP™)

## What is the Digital Motion Processor (DMP™)?

In version 1.2 of this library, we added _partial_support for the InvenSense Digital Motion Processor (DMP™). The DMP is firmware which runs on the
ICM-20948 and which "offloads computation of motion processing algorithms from the host processor, improving system power performance".

"The DMP enables ultra-low power run-time and background calibration of the accelerometer, gyroscope, and compass, maintaining optimal performance of
the sensor data for both physical and virtual sensors generated through sensor fusion."

The DMP allows the accelerometer, gyro and magnetometer data to be combined (fused) so that Quaternion data can be produced.

The DMP firmware binary has been available for quite some time. It is included in InvenSense's "MotionLink" and "Enbedded Motion Driver (eMD)" examples
which can be downloaded from the InvenSense Developers Corner. However, the code is opaque and difficult to follow.

Users like @ericalbers have [ported the InvenSense example code to the Arduino environment](https://github.com/ericalbers/ICM20948_DMP_Arduino) previously. We are
grateful to Eric as his code allowed us to reverse-engineer some of the ICM-20948 configuration steps.

We are also grateful to InvenSense themselves for sharing with us a _confidential & proprietary_ document called "_Application Note: Programming Sequence for
ICM-20648 DMP Hardware Function_". InvenSense admit that the document is not complete and have asked us not to share it openly.

The InvenSense document and the bus traffic we captured using Eric's port have allowed us to add _partial_ support for the DMP to this library, using our
own functions. We say _partial_ because, at the time of writing, our library does not support: activity recognition, step counting, pick-up and tap-detection.
It does however support:
- raw and calibrated accelerometer, gyro and compass data and accuracy
- 6-axis and 9-axis Quaternion data (including Game Rotation Vector data)
- Geomagnetic Rotation Vector data

We have added [three new examples](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino) to show how to configure the DMP and read:
9-axis Quaternion data; 6-axis Quaternion converted to Euler angles (roll, pitch & yaw); raw accelerometer data.

## Is DMP support enabled by default?

No. The DMP occupies 14kBytes of program memory and so, to allow the library to continue to run on processors with limited memory, DMP support is disabled by default.

You can enable it by editing the file called ```ICM_20948_C.h``` and uncommenting [line 29](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/src/util/ICM_20948_C.h#L29):

Change:

```
//#define ICM_20948_USE_DMP
```

to:

```
#define ICM_20948_USE_DMP
```

You will find ```ICM_20948_C.h``` in the library _src\util_ folder. If you are using Windows, you will find it in _Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util_.

## How is the DMP loaded and started?

The DMP firmware is loaded into the ICM-20948's processor memory space via three special Bank 0 registers:
- **AGB0_REG_MEM_START_ADDR** (0x7C) - the address which AGB0_REG_MEM_R_W reads from or writes to (it auto-increments after each read or write)
- **AGB0_REG_MEM_R_W** (0x7D) - the memory read/write register
- **AGB0_REG_MEM_BANK_SEL** (0x7E) - the memory bank select. The complete read/write address is: (AGB0_REG_MEM_BANK_SEL * 256) + AGB0_REG_MEM_START_ADDR

The firmware binary (14301 Bytes) is written into processor memory starting at address 0x90. ```loadDMPFirmware``` automatically breaks the code up into 256 byte blocks and increments **AGB0_REG_MEM_BANK_SEL** during the writing.

Before the DMP is enabled, the 16-bit register **AGB2_REG_PRGM_START_ADDRH** (Bank 2, 0x50) needs to be loaded with the program start address. ```setDMPstartAddress``` does this for you.

The DMP is enabled or reset by setting bits in the Bank 0 register **AGB0_REG_USER_CTRL** (0x03). ```enableDMP``` and ```resetDMP``` do this for you.

The helper functions ```readDMPmems``` and ```writeDMPmems``` will let you read and write data directly from the DMP memory space.

## How do I access the DMP data?

The DMP data is returned via the FIFO (First In First Out). ```readDMPdataFromFIFO``` checks if any data is present in the FIFO (by calling ```getFIFOcount``` which reads the 16-bit register **AGB0_REG_FIFO_COUNT_H** (0x70)). If data is present, it is copied into a ```icm_20948_DMP_data_t``` struct.

```readDMPdataFromFIFO``` will return:
- ```ICM_20948_Stat_FIFONoDataAvail``` if no data or incomplete data is available
- ```ICM_20948_Stat_Ok``` if a valid frame was read
- ```ICM_20948_Stat_FIFOMoreDataAvail``` if a valid frame was read _and_ the FIFO contains more (unread) data

You can examine the ```icm_20948_DMP_data_t data.header``` to see what data the frame contained. ```data.header``` is a bit field; each bit indicates what data is present:
- **DMP_header_bitmap_Compass_Calibr** (0x0020)
- **DMP_header_bitmap_Gyro_Calibr** (0x0040)
- **DMP_header_bitmap_Geomag** (0x0100)
- **DMP_header_bitmap_PQuat6** (0x0200)
- **DMP_header_bitmap_Quat9** (0x0400)
- **DMP_header_bitmap_Quat6** (0x0800)
- **DMP_header_bitmap_ALS** (0x1000)
- **DMP_header_bitmap_Compass** (0x2000)
- **DMP_header_bitmap_Gyro** (0x4000)
- **DMP_header_bitmap_Accel** (0x8000)

**DMP_header_bitmap_Header2** (0x0008) indicates if any secondary data was included. If the **DMP_header_bitmap_Header2** bit is set, the frame also contained one or more of:
- **DMP_header2_bitmap_Compass_Accuracy** (0x1000)
- **DMP_header2_bitmap_Gyro_Accuracy** (0x2000)
- **DMP_header2_bitmap_Accel_Accuracy** (0x4000)

## The DMP examples are verbose. Is that deliberate?

Yes, it certainly is! As you can tell from the examples, we are still gaining experience with the DMP. We have _deliberately_ written the examples so you can follow each step as the DMP is configured.

At some point in the future, we may hide the DMP configuration in a separate function. But, for now, you get to see the full configuration, warts and all!

## Where are the DMP registers defined?

You will find the definitions in ```ICM_20948_DMP.h```.

That file also includes the definition for the ```icm_20948_DMP_data_t``` struct which is loaded with DMP data from the FIFO.

## Can the DMP generate interrupts?

Yes it can, but you might find that they are not fully supported as we have not tested them. The main functions you will need to experiment with are ```intEnableDMP``` and ```enableDMPSensorInt```.

## How is the DMP data rate set?

We don't know the complete answer to this. As we understand it, it is a _combination_ of the raw sensor rate (set by ```setSampleRate```) and the multiple DMP Output Data Rate (ODR) registers (set by ```setDMPODRrate```). The documentation says that the "DMP is capable of outputting multiple sensor data at different rates to FIFO". So, in theory, you can have (e.g.) raw accelerometer data and Quaternion data arriving at different rates, but we have not tested that.

## Can I contribute to this library?

Absolutely! Please see [CONTRIBUTING.md](./CONTRIBUTING.md) for further details.
