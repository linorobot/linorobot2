#ifndef AK10_MIT_H
#define AK10_MIT_H

// CubeMars AK10-9 driver, MIT mode over CAN (Teensy 4.1 CAN2 = pin 0 CRX2 /
// pin 1 CTX2). The command packing is identical to the known-good bench-test
// sketch (teensy41_can1_can3_test.ino); this file adds feedback decoding so the
// measured wheel velocity can be turned into odometry for SLAM.
//
// MIT command frame (8 bytes), sent to the motor's command ID:
//   [kp(12) | kd(12) | p_des(16) | v_des(12) | t_ff(12)]
// MIT feedback frame (8 bytes), reported by the motor (byte 0 = controller id):
//   [id | pos(16) | vel(12) | cur(12)]
// Position/velocity are at the OUTPUT shaft (rad / rad/s), so wheel angular
// velocity maps 1:1 to v_des and to the decoded feedback velocity.

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"

// CAN2 on Teensy 4.1 (pins 0/1), matching the bench-test sketch.
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ak10_can;

inline uint32_t ak10_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    float span = x_max - x_min;
    uint32_t max_int = (1UL << bits) - 1;
    return (uint32_t)((x - x_min) * ((float)max_int / span));
}

inline float ak10_uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    uint32_t max_int = (1UL << bits) - 1;
    return ((float)x_int) * (span / (float)max_int) + x_min;
}

class AK10
{
public:
    AK10(uint8_t id, uint32_t cmd_id, float dir)
        : id_(id), cmd_id_(cmd_id), dir_(dir),
          feedback_vel_(0.0f), feedback_pos_(0.0f), last_feedback_ms_(0) {}

    // Command a wheel angular velocity (rad/s, robot-forward convention).
    // KP=0 => velocity control; torque feedforward overcomes stiction.
    void setWheelAngularVelocity(float wheel_omega)
    {
        float v_des = dir_ * wheel_omega;
        v_des = constrain(v_des, -MAX_WHEEL_OMEGA, MAX_WHEEL_OMEGA);

        float t_ff = 0.0f;
        if (v_des > 0.0f)       t_ff =  SPIN_TORQUE_FF;
        else if (v_des < 0.0f)  t_ff = -SPIN_TORQUE_FF;

        sendMIT(0.0f, v_des, SPIN_KP, SPIN_KD, t_ff);
    }

    // Active brake: hold velocity at zero with damping only.
    void stop()
    {
        sendMIT(0.0f, 0.0f, 0.0f, SPIN_KD, 0.0f);
    }

    // Measured wheel angular velocity (rad/s, robot-forward convention).
    // Returns 0 if no fresh feedback, so stale data cannot poison odometry.
    float getWheelAngularVelocity()
    {
        if (millis() - last_feedback_ms_ > FEEDBACK_STALE_MS)
            return 0.0f;
        return dir_ * feedback_vel_;
    }

    bool feedbackFresh() { return millis() - last_feedback_ms_ <= FEEDBACK_STALE_MS; }
    uint8_t id() const { return id_; }

    // Decode a MIT feedback frame for this motor.
    void handleFrame(const CAN_message_t &msg)
    {
        uint32_t p_int = ((uint32_t)msg.buf[1] << 8) | msg.buf[2];          // 16-bit
        uint32_t v_int = ((uint32_t)msg.buf[3] << 4) | (msg.buf[4] >> 4);   // 12-bit
        feedback_pos_ = ak10_uint_to_float(p_int, P_MIN, P_MAX, 16);
        feedback_vel_ = ak10_uint_to_float(v_int, V_MIN, V_MAX, 12);
        last_feedback_ms_ = millis();
    }

private:
    void sendMIT(float p_des, float v_des, float kp, float kd, float t_ff)
    {
        p_des = constrain(p_des, P_MIN, P_MAX);
        v_des = constrain(v_des, V_MIN, V_MAX);
        kp    = constrain(kp,    KP_MIN, KP_MAX);
        kd    = constrain(kd,    KD_MIN, KD_MAX);
        t_ff  = constrain(t_ff,  T_MIN, T_MAX);

        uint32_t kp_int = ak10_float_to_uint(kp,    KP_MIN, KP_MAX, 12);
        uint32_t kd_int = ak10_float_to_uint(kd,    KD_MIN, KD_MAX, 12);
        uint32_t p_int  = ak10_float_to_uint(p_des, P_MIN,  P_MAX,  16);
        uint32_t v_int  = ak10_float_to_uint(v_des, V_MIN,  V_MAX,  12);
        uint32_t t_int  = ak10_float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

        CAN_message_t msg;
        msg.id = cmd_id_;
        msg.len = 8;
        msg.flags.extended = 1;
        msg.buf[0] = kp_int >> 4;
        msg.buf[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);
        msg.buf[2] = kd_int & 0xFF;
        msg.buf[3] = p_int >> 8;
        msg.buf[4] = p_int & 0xFF;
        msg.buf[5] = v_int >> 4;
        msg.buf[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
        msg.buf[7] = t_int & 0xFF;
        ak10_can.write(msg);
    }

    uint8_t id_;
    uint32_t cmd_id_;
    float dir_;
    volatile float feedback_vel_;       // rad/s, motor frame
    volatile float feedback_pos_;       // rad, motor frame
    volatile uint32_t last_feedback_ms_;
};

inline void ak10Begin()
{
    ak10_can.begin();
    ak10_can.setBaudRate(CAN_BITRATE);
    ak10_can.setMaxMB(16);
    ak10_can.enableFIFO();
}

// OPTIONAL: send the MIT "enter motor mode" command (FF..FC). The bench-test
// sketch does NOT use this, so it is off by default. If the motors do not
// respond after flashing, call this once per motor right after ak10Begin().
inline void ak10EnterMotorMode(uint32_t cmd_id)
{
    CAN_message_t msg;
    msg.id = cmd_id;
    msg.len = 8;
    msg.flags.extended = 1;
    for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
    msg.buf[7] = 0xFC;
    ak10_can.write(msg);
}

// Drain the RX FIFO and route each MIT feedback frame to its motor by the
// controller id in byte 0.
inline void ak10Poll(AK10 &left, AK10 &right)
{
    CAN_message_t msg;
    while (ak10_can.read(msg))
    {
        if (msg.len < 6)
            continue;
        uint8_t controller_id = msg.buf[0];
        if (controller_id == left.id())
            left.handleFrame(msg);
        else if (controller_id == right.id())
            right.handleFrame(msg);
    }
}

#endif
