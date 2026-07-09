#ifndef ODOMETRY_H
#define ODOMETRY_H

// Integrates body velocities into a pose and fills the nav_msgs/Odometry
// message published on odom/unfiltered (fused with the IMU by linorobot2's
// EKF on the robot computer). Modeled on linorobot2_hardware's Odometry class.

#include <Arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

class Odometry
{
public:
    Odometry() : x_(0.0), y_(0.0), heading_(0.0)
    {
        nav_msgs__msg__Odometry__init(&odom_msg_);
        rosidl_runtime_c__String__assign(&odom_msg_.header.frame_id, "odom");
        rosidl_runtime_c__String__assign(&odom_msg_.child_frame_id, "base_footprint");
    }

    void update(float vel_dt, float linear_x, float angular_z)
    {
        float delta_heading = angular_z * vel_dt;
        float cos_h = cos(heading_);
        float sin_h = sin(heading_);
        float delta_x = (linear_x * cos_h) * vel_dt;
        float delta_y = (linear_x * sin_h) * vel_dt;

        x_ += delta_x;
        y_ += delta_y;
        heading_ += delta_heading;

        // yaw -> quaternion
        float q[4];
        eulerToQuat(0, 0, heading_, q);

        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation.x = (double)q[1];
        odom_msg_.pose.pose.orientation.y = (double)q[2];
        odom_msg_.pose.pose.orientation.z = (double)q[3];
        odom_msg_.pose.pose.orientation.w = (double)q[0];
        odom_msg_.pose.covariance[0] = 0.001;
        odom_msg_.pose.covariance[7] = 0.001;
        odom_msg_.pose.covariance[35] = 0.001;

        odom_msg_.twist.twist.linear.x = linear_x;
        odom_msg_.twist.twist.linear.y = 0.0;
        odom_msg_.twist.twist.angular.z = angular_z;
        // Honest wheel-odom uncertainty: ERPM feedback is quantized
        // (~0.0004 m/s per count) and jittery at crawl speeds, and a diff
        // drive cannot measure vy at all. The old 0.0001 told the EKF to
        // treat that jitter as near-ground-truth.
        odom_msg_.twist.covariance[0] = 0.001;
        odom_msg_.twist.covariance[7] = 0.01;
        odom_msg_.twist.covariance[35] = 0.001;
    }

    nav_msgs__msg__Odometry *getData() { return &odom_msg_; }

private:
    void eulerToQuat(float roll, float pitch, float yaw, float *q)
    {
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);

        q[0] = cy * cp * cr + sy * sp * sr;
        q[1] = cy * cp * sr - sy * sp * cr;
        q[2] = sy * cp * sr + cy * sp * cr;
        q[3] = sy * cp * cr - cy * sp * sr;
    }

    nav_msgs__msg__Odometry odom_msg_;
    float x_;
    float y_;
    float heading_;
};

#endif
