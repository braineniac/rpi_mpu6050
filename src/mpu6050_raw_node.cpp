#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include <vector>
#include <array>

#define AVG_BUF_SIZE 200 // offset buffer size after warmup

class MPU6050Node {
  public:
    MPU6050Node();
    size_t getSampleRate() { return sample_rate; }
    void   poll();

    void signal_f() { signal(SIGINT, MPU6050Node::static_signal_handler); }
    void signal_handler(int signum);

    static void static_signal_handler(int signum) { instance.signal_handler(signum); }

  private:
    static MPU6050Node instance;
    ros::NodeHandle    nh;

    int         sample_rate;
    std::string frame_id;

    ros::Publisher data_pub;
    ros::Publisher euler_pub;
    ros::Publisher mag_pub;

    // Time
    ros::Time     begin;
    ros::Time     now;
    ros::Duration warmup_time{90.0};

    // flags
    bool warmup_done{false};
    bool avg_done{false};
    bool DMP_ready{false};

    // MPU6050
    MPU6050  mpu;
    bool     ado{false};
    uint8_t  mpuIntStatus;    // holds actual interrupt status byte from MPU
    uint8_t  devStatus;       // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;       // count of all bytes currently in FIFO
    uint8_t  fifoBuffer[64];    // FIFO storage buffer

    // covariances
    double angular_velocity_covariance{0.0};
    double linear_acceleration_covariance{0.0};
    double pitch_roll_covariance{0.0};
    double yaw_covariance{0.0};

    // deviations
    double linear_acceleration_stdev{0.0};
    double angular_velocity_stdev{0.0};
    double pitch_roll_stdev{0.0};
    double yaw_stdev{0.0};

    // offsets
    std::array<std::vector<double>, AVG_BUF_SIZE> a_offset_array;
    std::array<std::vector<double>, AVG_BUF_SIZE> g_offset_array;

    std::array<double, 6> avg;
    std::vector<double>   a_offset;
    std::vector<double>   g_offset;

    // orientation/motion vars
    //Quaternion  q;           // [w, x, y, z]         quaternion container
    //VectorInt16 aa;          // [x, y, z]            accel sensor measurements
    //VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor measurements
    //VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor measurements
    //VectorFloat gravity;     // [x, y, z]            gravity vector
    float       euler[3];    // [psi, theta, phi]    Euler angle container
    float       ypr[3];      // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float       la[3];       // [x, y, z]
    float       av[3];
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // methods
    bool mpu_setup();
    bool wait_for_warmup();
    bool set_offsets();
    void publish_imu();
};
MPU6050Node::MPU6050Node() {

    begin = ros::Time::now();

    /* setting parameters from launch file */
    nh.param<std::string>("frame_id", frame_id, "imu_link");
    nh.param("sample_rate", sample_rate, 1000);
    nh.param("ado", ado, false);
    // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz
    nh.param("linear_acceleration_stdev", linear_acceleration_stdev, (400 / 1000000.0) * 9.807);
    // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    nh.param("angular_velocity_stdev", angular_velocity_stdev, 0.05 * (M_PI / 180.0));
    // 1 degree for pitch and roll
    nh.param("pitch_roll_stdev", pitch_roll_stdev, 1.0 * (M_PI / 180.0));
    // 5 degrees for yaw
    nh.param("yaw_stdev", yaw_stdev, 5.0 * (M_PI / 180.0));

    /* setting publishers */
    data_pub  = nh.advertise<sensor_msgs::Imu>("imu/data_raw", sample_rate);
    euler_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/euler", sample_rate);
    mag_pub   = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", sample_rate);

    /* setting covariances */
    angular_velocity_covariance    = angular_velocity_stdev * angular_velocity_stdev;
    linear_acceleration_covariance = linear_acceleration_stdev * linear_acceleration_stdev;
    pitch_roll_covariance          = pitch_roll_stdev * pitch_roll_stdev;
    yaw_covariance                 = yaw_stdev * yaw_stdev;

    /* Initialising vectors */
    a_offset_array.fill(std::vector<double>(3, 0.0));
    g_offset_array.fill(std::vector<double>(3, 0.0));
    std::fill(std::begin(avg), std::end(avg), 0.0);
    a_offset = std::vector<double>(3, 0.0);
    g_offset = std::vector<double>(3, 0.0);

    /* Initialising the MPU */
    if(!mpu_setup()) {
        ROS_ERROR("MPU6050 setup failed, exiting...\n");
        ros::shutdown();
    }
}

void MPU6050Node::signal_handler(int signum) {

        ROS_INFO("Shutting down mpu6050_node");
        mpu.reset();

        ros::shutdown();
    }

bool MPU6050Node::mpu_setup() {

    I2Cdev::initialize();

    mpu = MPU6050(ado ? 0x69 : 0x68);
    if(!mpu.testConnection()) {
        std::cout << "MPU6050 connection failed" << std::endl << std::flush;
        return DMP_ready;
    }

    mpu.initialize();


    DMP_ready = true;
    return DMP_ready;
}

void MPU6050Node::poll() {

    now = ros::Time::now();

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    la[0] = ax * 1 / 8192. * 9.80655 + a_offset[0];
    la[1] = ay * 1 / 8192. * 9.80655 + a_offset[1];
    la[2] = az * 1 / 8192. * 9.80655 + a_offset[2];

    av[0] = gx * M_PI/180.0 + g_offset[0];
    av[1] = gy * M_PI/180.0 + g_offset[1];
    av[2] = gz * M_PI/180.0 + g_offset[2];

    if(wait_for_warmup()) {
        if(set_offsets()) { publish_imu(); }
    }
}

bool MPU6050Node::wait_for_warmup() {

    if((now - begin - warmup_time).toSec() > 0 && !warmup_done) {

        ROS_INFO("Warmup finished");
        ROS_INFO("Collecting data for offsets");
        warmup_done = true;
    }

    return warmup_done;
}

bool MPU6050Node::set_offsets() {

    // fill offset vector arrays
    if((now - begin - warmup_time).toSec() - int(AVG_BUF_SIZE / sample_rate + 1) < 0) {

        for(int i = 0; i < a_offset_array.size(); i++) {
            if(fabs(la[0]) < 30) { a_offset_array[i][0] = la[0]; }
            if(fabs(la[1]) < 30) { a_offset_array[i][1] = la[1]; }
            if(fabs(la[2]) < 30) { a_offset_array[i][2] = la[2]; }
            if(fabs(av[0]) < 30) { g_offset_array[i][0] = av[0]; }
            if(fabs(av[1]) < 30) { g_offset_array[i][1] = av[1]; }
            if(fabs(av[2]) < 30) { g_offset_array[i][2] = av[2]; }

        }
        return avg_done;
    }

    if(!avg_done) {
        // calculate average values from offset vector arrays
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < a_offset_array.size(); j++) {
                avg[i] += a_offset_array[j][i];
                avg[i + 3] += g_offset_array[j][i];
            }
            avg[i]     = avg[i] / a_offset_array.size();
            avg[i + 3] = avg[i + 3] / g_offset_array.size();
        }

        // set offset vectors
        a_offset[0] = -avg[0];
        a_offset[1] = -avg[1];
        a_offset[2] = -avg[2];
        g_offset[0] = -avg[3];
        g_offset[1] = -avg[4];
        g_offset[2] = -avg[5];

        ROS_INFO_STREAM("Offset linear accel x: " + std::to_string(avg[0]));
        ROS_INFO_STREAM("Offset linear accel y: " + std::to_string(avg[1]));
        ROS_INFO_STREAM("Offset linear accel z: " + std::to_string(avg[2]));
        ROS_INFO_STREAM("Offset angular vel x: " + std::to_string(avg[3]));
        ROS_INFO_STREAM("Offset angular vel y: " + std::to_string(avg[4]));
        ROS_INFO_STREAM("Offset angular vel z: " + std::to_string(avg[5]));

        ROS_INFO("Setup finished");

        avg_done = true;
    }

    return avg_done;
}

void MPU6050Node::publish_imu() {

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = now;
    imu_msg.header.frame_id = frame_id;

    geometry_msgs::Vector3Stamped imu_euler_msg;
    imu_euler_msg.header.stamp    = now;
    imu_euler_msg.header.frame_id = frame_id;

    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header.stamp    = now;
    mag_msg.header.frame_id = frame_id;

    //imu_msg.orientation.x = q.x;
    //imu_msg.orientation.y = q.y;
    //imu_msg.orientation.z = q.z;
    //imu_msg.orientation.w = q.w;

    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
    imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
    imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;

    imu_msg.orientation_covariance[0] = pitch_roll_covariance;
    imu_msg.orientation_covariance[4] = pitch_roll_covariance;
    imu_msg.orientation_covariance[8] = yaw_covariance;

    imu_msg.angular_velocity.x = av[0];
    imu_msg.angular_velocity.y = av[1];
    imu_msg.angular_velocity.z = av[2];

    imu_msg.linear_acceleration.x = la[0];
    imu_msg.linear_acceleration.y = la[1];
    imu_msg.linear_acceleration.z = la[2];

		if((fabs(la[0])<1)&&(fabs(la[1])<1)&&(fabs(la[2])<1)) {
						data_pub.publish(imu_msg);
		}
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mpu6050_node");
    ROS_INFO("Starting mpu6050_node");
    ROS_INFO("Warming up");

    MPU6050Node mpu6050_node;

    ros::Rate r(mpu6050_node.getSampleRate());

    while(ros::ok()) {
        try {
            mpu6050_node.poll();
        }
        catch (const std::exception& e) {
            ROS_INFO("Exception caught");
        }
        ros::spinOnce();
        r.sleep();
    }
}
