# rpi_mpu6050
A ROS node for the MPU6050 connected to the raspberry pi

A self contained catkinized packge. Automatically installs:  
https://www.airspayce.com/mikem/bcm2835/  

Currently there is a "warm up" period until the IMU settles, then offsets are calculated from the stationary 
position and published.  

This project was inspired by:  
https://github.com/chrisspen/ros_mpu6050_node  

MPU6050 library used:  
https://github.com/jrowberg/i2cdevlib  
