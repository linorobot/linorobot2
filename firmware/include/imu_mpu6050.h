#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

// MPU6050 over I2C (Teensy 4.1: SDA = pin 18, SCL = pin 19, 3.3V).
// Publishes raw accel/gyro; orientation is left to the EKF / madgwick filter
// on the robot computer, so orientation_covariance[0] = -1 marks it as not
// provided. The IMU is OPTIONAL: if none is connected, ok() returns false and
// the firmware simply publishes odometry only.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>

class ImuMPU6050
{
public:
    bool init()
    {
        sensor_msgs__msg__Imu__init(&imu_msg_);
        rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, "imu_link");
        imu_msg_.orientation.w = 1.0;
        imu_msg_.orientation_covariance[0] = -1.0; // no orientation estimate
        imu_msg_.angular_velocity_covariance[0] = 0.00001;
        imu_msg_.angular_velocity_covariance[4] = 0.00001;
        imu_msg_.angular_velocity_covariance[8] = 0.00001;
        imu_msg_.linear_acceleration_covariance[0] = 0.00001;
        imu_msg_.linear_acceleration_covariance[4] = 0.00001;
        imu_msg_.linear_acceleration_covariance[8] = 0.00001;

        ok_ = mpu_.begin();
        if (ok_)
        {
            mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu_.setFilterBandwidth(MPU6050_BAND_44_HZ);
        }
        return ok_;
    }

    bool ok() { return ok_; }

    sensor_msgs__msg__Imu *getData()
    {
        if (ok_)
        {
            sensors_event_t a, g, t;
            mpu_.getEvent(&a, &g, &t);
            imu_msg_.linear_acceleration.x = a.acceleration.x; // m/s^2
            imu_msg_.linear_acceleration.y = a.acceleration.y;
            imu_msg_.linear_acceleration.z = a.acceleration.z;
            imu_msg_.angular_velocity.x = g.gyro.x; // rad/s
            imu_msg_.angular_velocity.y = g.gyro.y;
            imu_msg_.angular_velocity.z = g.gyro.z;
        }
        return &imu_msg_;
    }

private:
    Adafruit_MPU6050 mpu_;
    sensor_msgs__msg__Imu imu_msg_;
    bool ok_ = false;
};

#endif
