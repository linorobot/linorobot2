#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Robot geometry -- MOCK PLACEHOLDER VALUES so the firmware builds and runs.
// Replace with real measurements before trusting odometry / SLAM.
// ---------------------------------------------------------------------------
#define WHEEL_DIAMETER       0.150   // meters (mock). Measure your wheel.
#define LR_WHEELS_DISTANCE   0.350   // meters, center-to-center (mock). Measure.

// ---------------------------------------------------------------------------
// AK10-9 actuators (CubeMars, MIT mode over CAN). Values copied from the
// known-good bench-test sketch (teensy41_can1_can3_test.ino).
//
// MOTOR1 = LEFT wheel, MOTOR2 = RIGHT wheel.
// Command IDs are the MIT command frame IDs (extended), one per motor.
// The low byte of the controller ID is what the motor reports in feedback
// byte 0, used to route incoming frames to the right motor.
// ---------------------------------------------------------------------------
#define LEFT_MOTOR_ID        0x68    // controller ID (feedback byte 0)
#define RIGHT_MOTOR_ID       0x69
#define LEFT_MOTOR_CMD_ID    0x868   // MIT command frame ID (extended)
#define RIGHT_MOTOR_CMD_ID   0x869

// Direction: +1.0 means "positive wheel velocity pushes the robot forward".
// On a mirrored left/right drivetrain one side is usually negated.
#define LEFT_MOTOR_DIR        1.0f
#define RIGHT_MOTOR_DIR      -1.0f

// MIT-mode control gains (from the bench-test sketch).
// torque = KP*(p_des-p) + KD*(v_des-v) + t_ff.  KP=0 => pure velocity control.
#define SPIN_KP               0.0f   // position gain (0 = velocity control)
#define SPIN_KD               3.0f   // velocity damping gain
#define SPIN_TORQUE_FF        4.0f   // Nm feedforward to overcome stiction
                                     // (applied with the sign of the command).
                                     // TUNE DOWN if low-speed motion is jerky.

// AK10-9 V3 MIT ranges (must match the motor firmware).
#define P_MIN  (-12.56f)
#define P_MAX  ( 12.56f)
#define V_MIN  (-28.0f)             // output-shaft angular velocity, rad/s
#define V_MAX  ( 28.0f)
#define T_MIN  (-54.0f)
#define T_MAX  ( 54.0f)
#define KP_MIN ( 0.0f)
#define KP_MAX ( 500.0f)
#define KD_MIN ( 0.0f)
#define KD_MAX ( 5.0f)

// In MIT mode the commanded/reported velocity is at the OUTPUT shaft in rad/s,
// so NO gear ratio / pole-pair conversion is needed (unlike servo/ERPM mode).
// Wheel angular velocity (rad/s) maps 1:1 to MIT v_des/feedback, assuming the
// wheel is mounted directly on the output shaft.
#define MAX_WHEEL_OMEGA      V_MAX   // rad/s safety clamp

#define CAN_BITRATE          1000000 // CubeMars default 1 Mbps

// ---------------------------------------------------------------------------
// Behavior
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_MS    20      // 50 Hz control/odometry loop
#define CMD_VEL_TIMEOUT_MS   200     // stop if no cmd_vel for this long
#define FEEDBACK_STALE_MS    250     // treat motor feedback older than this as 0

#endif
