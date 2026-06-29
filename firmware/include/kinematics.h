#ifndef KINEMATICS_H
#define KINEMATICS_H

// Differential drive (2WD) kinematics in wheel ANGULAR VELOCITY (rad/s), which
// is the natural unit for AK10-9 MIT mode. linorobot2 conventions: x forward,
// z up, positive angular_z = counter-clockwise.

#include <Arduino.h>
#include "config.h"

class Kinematics2WD
{
public:
    struct WheelOmega
    {
        float left;   // rad/s
        float right;  // rad/s
    };

    struct Velocities
    {
        float linear_x;  // m/s
        float angular_z; // rad/s
    };

    Kinematics2WD()
        : wheel_radius_(WHEEL_DIAMETER / 2.0f),
          track_(LR_WHEELS_DISTANCE) {}

    // cmd_vel (m/s, rad/s) -> per-wheel angular velocity (rad/s).
    WheelOmega getWheelOmega(float linear_x, float angular_z)
    {
        float v_left  = linear_x - (angular_z * track_ / 2.0f);
        float v_right = linear_x + (angular_z * track_ / 2.0f);

        WheelOmega w;
        w.left  = v_left  / wheel_radius_;
        w.right = v_right / wheel_radius_;
        return w;
    }

    // measured per-wheel angular velocity (rad/s) -> body velocities.
    Velocities getVelocities(float left_omega, float right_omega)
    {
        float v_left  = left_omega  * wheel_radius_;
        float v_right = right_omega * wheel_radius_;

        Velocities vel;
        vel.linear_x  = (v_left + v_right) / 2.0f;
        vel.angular_z = (v_right - v_left) / track_;
        return vel;
    }

private:
    float wheel_radius_;
    float track_;
};

#endif
