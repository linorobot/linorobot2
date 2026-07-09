#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Robot geometry -- measured on the real robot (chassis is 550mm x 550mm).
// ---------------------------------------------------------------------------
#define WHEEL_DIAMETER       0.120     // meters, measured
#define LR_WHEELS_DISTANCE   0.37921   // meters, center-to-center, measured

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

// Body longitudinal-axis sign. With the motor DIR constants above this
// drivetrain drives physically BACKWARD for +cmd_vel.x and reports real
// forward motion as -odom.x -- i.e. its forward axis is reversed vs the ROS
// convention (REP-103: +x = forward). TURNING is already correct. Flipping this
// sign corrects BOTH the command and the odometry linear.x together, so +x =
// forward everywhere (teleop, Nav2, SLAM). Set to 1.0f if your drivetrain is
// already ROS-correct. After changing this, remove teleop's LINEAR_SIGN flip.
#define BASE_LINEAR_DIR      -1.0f

// MIT-mode control gains (from the bench-test sketch).
// torque = KP*(p_des-p) + KD*(v_des-v) + t_ff.  KP=0 => pure velocity control.
#define SPIN_KP               0.0f   // position gain (0 = velocity control)
#define SPIN_KD               3.0f   // velocity damping gain

// Friction/stiction feedforward. With KP=0 the torque law is
// KD*(v_des - v) + t_ff, so any t_ff ABOVE the true friction torque acts as a
// steady-state speed offset of (excess/KD) that the damping then fights -- at
// the 4.0 Nm it used to be, 0.05 m/s commands lurched and oscillated. The FF
// now ramps linearly with |v_des| up to SPIN_FF_FULL_RADPS (see ak10_mit.h),
// removing the old +/-4 Nm bang-bang step across zero.
// TUNING: if wheels stall on 0.05 m/s commands, raise SPIN_TORQUE_FF in
// 0.25 Nm steps until they reliably start, then stop -- the ceiling should sit
// just above measured breakaway torque, no higher.
#define SPIN_TORQUE_FF        1.0f   // Nm feedforward ceiling (was 4.0)
#define SPIN_FF_FULL_RADPS    1.0f   // |v_des| (rad/s) at which full FF applies

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
// 0.00842 = (0.05 m/s / 0.06 m wheel radius) / 99 ERPM, i.e. the value the
// empirical note above actually implies. The previous 0.00677 under-reported
// wheel speed by ~20%, which skewed odometry (and anything fusing it).
#define ERPM_TO_WHEEL_RADPS  0.00842f   // wheel rad/s per ERPM count (TUNE)

// Servo-mode status frame reports motor phase current at buf[4:5] as a signed
// int16 in units of 0.01 A (per the CubeMars manual). CONFIRM against a clamp
// meter / datasheet the same way ERPM_TO_WHEEL_RADPS still needs calibration.
#define CURRENT_LSB_TO_AMP   0.01f      // amps per current LSB (CONFIRM)

// ---------------------------------------------------------------------------
// Behavior
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_MS    20      // 50 Hz control/odometry loop
#define CMD_VEL_TIMEOUT_MS   400     // stop if no cmd_vel for this long.
                                     // 400 rides through Nav2 BT/controller
                                     // transitions (>200ms publish gaps) that
                                     // used to slam the brake mid-drive; still
                                     // stops within ~2 cm at the 0.05 m/s cap.
#define FEEDBACK_STALE_MS    250     // treat motor feedback older than this as 0

// Overcurrent/stall protection. If EITHER motor's |phase current| stays above
// OVERCURRENT_AMPS for OVERCURRENT_MS continuously (a jammed wheel, a pinned
// robot), the firmware latches a stop: motors brake and ignore cmd_vel until
// the commanded twist returns to zero (teleop hard-stop, Nav2 cancel, or the
// CMD_VEL_TIMEOUT failsafe). Driving at the 0.05 m/s cap on flat ground draws
// well under 8 A, and a stalled AK10-9 will blow past it within the window.
// TUNE: log /motor_current during normal driving and set the threshold ~2x the
// worst normal draw. Depends on CURRENT_LSB_TO_AMP being confirmed.
#define OVERCURRENT_AMPS     8.0f    // per-motor stall threshold, amps (TUNE)
#define OVERCURRENT_MS       1000    // sustained this long -> latch a stop

// Re-sync the micro-ROS epoch this often while connected. The Teensy clock
// drifts relative to the agent; a once-per-connection sync (the old behavior)
// let odom/IMU stamps walk away from ROS time over long sessions.
#define TIME_SYNC_PERIOD_MS  60000

#endif
