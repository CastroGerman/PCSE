# MPU9250

This is a driver designed for [InvenSense MPU-9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) Inertial Measurement Unit (IMU).

# Description

The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: 
the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 
three-axis magnetometer. The MPU-9250 supports I2C, up to 400 kHz, and SPI communication, 
up to 1 MHz for register setup and 20 MHz for data reading. 
The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range |
| --- | --- | ---  |
| +/- 250 deg/s  | +/- 2g  | +/- 4800 uT |
| +/- 500 deg/s  | +/- 4g  | |
| +/- 1000 deg/s | +/- 8g  | |
| +/- 2000 deg/s | +/- 16g | |

The MPU-9250 samples the gyros, accelerometers, and magnetometers with 16 bit analog to 
digital converters. It also features programmable digital filters, a precision clock, 
an embedded temperature sensor, programmable interrupts (including wake on motion), 
and a 512 byte FIFO buffer.

# Installation

Compile the driver alongside your project source files. This driver is added as:
```C
  #include "MPU9250.h"
```
To communicate with the MPU9250 sensor, you first have to set the I2C interface. 
The I2C interface is required to instantiate `I2CPort` class and then `MPU9250` class as follows:
```C
    I2CPort i2c;
    I2CPort_ctor(&i2c, &hi2c1);
    MPU9250 imu;
    MPU9250_ctor(&imu, (uint8_t)MPU9250_ADDRESS_AD0_LOW, &i2c);
```
**Note**: the previous code is an example for any STM32 board port. Each board port will
define their own I2C interface for `I2CPort`.

## I2C

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output. Connect to interruptable pin on microcontroller.
   * SDA / SDI: connect to SDA.
   * SCL / SCLK: connect to SCL.
   * AD0 / SDO: ground to select I2C address 0x68. Pull high to VDD to select I2C address 0x69.
   * nCS: no connect.
   * AUXDA: not used.
   * AUXCL: not used.

2.2 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.
