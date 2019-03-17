#!/usr/bin/env python

import rospy
from sensor_msgs import Imu
import numpy as np

from mpu6050 import mpu6050

class MPU6050Node:
    def __init__(self, i2c_addr=0x68):
        self.mpu6050 = mpu6050(i2c_addr)
        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)

        self.offset_done = False

        self.linear_acceleration = np.zeros(1,3)
        self.angular_velocity = np.zeros(1,3)
        self.temp = 0.0

        self.la_offset = 0.0
        self.av_offset = 0.0

        self.offset_array = np.zeros(6,1000)
        self.offset_array[:] = np.nan

        self.linear_acceleration_covariance = np.zeros(1,9)
        self.angular_velocity_covariance = np.zeros(1,9)
        self.orientation_covariance = np.zeros(1,9)

        rospy.loginfo("Setting covariances")
        self.set_covariances()

    def poll(self):
        self.linear_acceleration[0] = self.mpu6050.get_accel_data()['x']
        self.linear_acceleration[1] = self.mpu6050.get_accel_data()['y']
        self.linear_acceleration[2] = self.mpu6050.get_accel_data()['z']

        self.angular_velocity[0] = self.mpu6050.get_gyro_data()['x']
        self.angular_velocity[1] = self.mpu6050.get_gyro_data()['y']
        self.angular_velocity[2] = self.mpu6050.get_gyro_data()['z']

        self.temp = self.mpu6050.get_temp()

        if not self.offset_done:
            self.get_offsets()

    def get_offsets(self):
        for accel_x,accel_y,accel_z in [self.offset_array[0:],self.offset_array[1:],self.offset_array[2:]]:
            if accel_x == np.nan:
                accel_x = self.linear_acceleration[0]
            if accel_y == np.nan:
                accel_y = self.linear_acceleration[1]
            if accel_z == np.nan:
                accel_z = self.linear_acceleration[2]

        for vel_x,vel_y,vel_z in [self.offset_array[3:], self.offset_array[4:],self.offset_array[5:]]:
            if vel_x == np.nan:
                vel_x = self.angular_velocity[0]
            if vel_y == np.nan:
                vel_y = self.angular_velocity[1]
            if vel_z == np.nan:
                vel_z = self.angular_velocity[2]

        if self.offset_array[0:-1] != np.nan:
            self.set_offsets()
            rospy.loginfo("Offsets set")

            rospy.loginfo("Offset linear x={}".format(self.la_offset[0]))
            rospy.loginfo("Offset linear y={}".format(self.la_offset[1]))
            rospy.loginfo("Offset linear z={}".format(self.la_offset[2]))
            rospy.loginfo("Offset angular x={}".format(self.av_offset[0]))
            rospy.loginfo("Offset angular y={}".format(self.av_offset[1]))
            rospy.loginfo("Offset angular z={}".format(self.av_offset[2]))

            self.offset_done = true

    def set_offsets(self):
        self.la_offset[0] = np.average(self.offset_array[0:])
        self.la_offset[1] = np.average(self.offset_array[1:])
        self.la_offset[2] = np.average(self.offset_array[2:])

        self.av_offset[0] = np.average(self.offset_array[3:])
        self.av_offset[1] = np.average(self.offset_array[4:])
        self.av_offset[2] = np.average(self.offset_array[5:])

    def set_covariances(self):
        linear_acceleration_stdev = (400/1000000) * 9.80655
        angular_velocity_stdev = 0.05 * (np.pi / 180.0)
        pitch_roll_stdev = 1.0 * (np.pi / 180.0)
        yaw_stdev = 5.0 * (np.pi / 180.0)

        linear_acceleration_covariance = linear_acceleration_stdev * linear_acceleration_stdev
        angular_velocity_covariance = angular_velocity_stdev * angular_velocity_stdev
        pitch_roll_covariance = pitch_roll_covariance * pitch_roll_covariance
        yaw_covariance = yaw_stdev * yaw_stdev

        self.linear_acceleration_covariance[0] = linear_acceleration_covariance
        self.linear_acceleration_covariance[4] = linear_acceleration_covariance
        self.linear_acceleration_covariance[8] = linear_acceleration_covariance

        self.angular_velocity_covariance[0] = angular_velocity_covariance
        self.angular_velocity_covariance[4] = angular_velocity_covariance
        self.angular_velocity_covariance[8] = angular_velocity_covariance

        self.orientation_covariance[0] = pitch_roll_covariance
        self.orientation_covariance[4] = pitch_roll_covariance
        self.orientation_covariance[8] = yaw_covariance

    def publish(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        #imu_msg.orientation_covariance = self.orientation_covariance

        imu_msg.linear_acceleration.x = self.linear_acceleration[0] - self.la_offset[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1] - self.la_offset[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2] - self.la_offset[2]

        imu_msg.angular_velocity.x = self.angular_velocity[0] - self.av_offset[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1] - self.av_offset[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2] - self.av_offset[2]

        self.imu_pub.publish(imu_msg)


if __name__ == '__main__':
    rospy.loginfo("Initiallising mpu6050 node");
    rospy.init_node("mpu6050_node")

    mpu6050_node = MPU6050Node()

    rospy.loginfo("Warming up")
    rospy.sleep(30)

    rospy.loginfo("Gathering data for offsets")

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        mpu6050_node.poll()
        if mpu6050_node.offset_done:
            mpu6050.publish()
        rate.sleep()
