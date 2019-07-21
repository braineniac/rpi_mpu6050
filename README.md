# MPU6050 Node for the Raspberry Pi
A ROS node for the MPU6050 connected to the Raspberry Pi via I2C.

A self contained catkinized packge. Automatically installs:  
https://www.airspayce.com/mikem/bcm2835/  

There is a "warm up" period until the IMU settles, then offsets are calculated from the still position and published.

The library used in this code is old and somewhat unstable, but it works.

MPU6050 library used:  
https://github.com/jrowberg/i2cdevlib

## Installation
In your workspace:
```
$ catkin_make rpi_mpu6050
```

## Use
Launch raw output without DMP:
```
# roslaunch rpi_mpu6050 mpu6050_raw.launch
```
Launch output with DMP:
```
# roslaunch rpi_mpu6050 mpu6050_dmp.launch
```

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details
