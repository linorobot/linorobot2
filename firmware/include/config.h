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
// Feedback decode. The motors run in SERVO mode and stream a status frame whose
// CAN-id low byte is the motor id (0x68/0x69). Layout (8 bytes, big-endian):
//   buf[0:1]=marker(const 0x7D00)  buf[2:3]=speed ERPM (int16)
//   buf[4:5]=current x0.01A         buf[6]=temp C   buf[7]=error
// Convert reported ERPM to wheel angular velocity (rad/s). The starting value is
// empirical (steady 0.05 m/s drive -> ~99 ERPM); REFINE by driving a measured
// distance and matching integrated odometry, together with WHEEL_DIAMETER.
// ---------------------------------------------------------------------------
#define ERPM_TO_WHEEL_RADPS  0.00677f   // wheel rad/s per ERPM count (TUNE)

// Servo-mode status frame reports motor phase current at buf[4:5] as a signed
// int16 in units of 0.01 A (per the CubeMars manual). CONFIRM against a clamp
// meter / datasheet the same way ERPM_TO_WHEEL_RADPS still needs calibration.
#define CURRENT_LSB_TO_AMP   0.01f      // amps per current LSB (CONFIRM)

// ---------------------------------------------------------------------------
// Behavior
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_MS    20      // 50 Hz control/odometry loop
#define CMD_VEL_TIMEOUT_MS   200     // stop if no cmd_vel for this long
#define FEEDBACK_STALE_MS    250     // treat motor feedback older than this as 0

#endif
