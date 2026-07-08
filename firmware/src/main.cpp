// linorobot2-compatible base firmware for Teensy 4.1
//   2x CubeMars AK10-9 (MIT mode over CAN2) + optional MPU6050 + micro-ROS.
//
// Talks to the robot computer over USB serial via micro-ROS. The agent is
// launched by linorobot2_bringup:
//   ros2 launch linorobot2_bringup bringup.launch.py
//
//   subscribes: cmd_vel         (geometry_msgs/Twist)
//   publishes:  odom/unfiltered (nav_msgs/Odometry)   <- wheel odometry for SLAM
//               imu/data        (sensor_msgs/Imu)      <- if an MPU6050 is present
//
// Structure follows linorobot2_hardware's firmware: a connection state machine
// that keeps retrying the agent, and a 50 Hz control loop that stops the motors
// whenever cmd_vel goes quiet or the agent drops.

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "config.h"
#include "ak10_mit.h"
#include "kinematics.h"
#include "odometry.h"
#include "imu_mpu6050.h"

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }
#define RCSOFTCHECK(fn)         \
    {                           \
        rcl_ret_t temp_rc = fn; \
        (void)temp_rc;          \
    }
#define EXECUTE_EVERY_N_MS(MS, X)              \
    do                                         \
    {                                          \
        static volatile int64_t init = -1;     \
        if (init == -1)                        \
        {                                      \
            init = millis();                   \
        }                                      \
        if (millis() - init > MS)              \
        {                                      \
            X;                                 \
            init = millis();                   \
        }                                      \
    } while (0)

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rclc_executor_t executor;

rcl_subscription_t twist_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t motor_current_publisher;

geometry_msgs__msg__Twist twist_msg;
// /motor_current : [left, right] motor current draw in amps. Backed by a static
// array so no dynamic allocation is needed for the micro-ROS sequence.
std_msgs__msg__Float32MultiArray motor_current_msg;
static float motor_current_data[2];

AK10 left_motor(LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID, LEFT_MOTOR_DIR);
AK10 right_motor(RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID, RIGHT_MOTOR_DIR);
Kinematics2WD kinematics;
Odometry odometry;
ImuMPU6050 imu;

enum AgentState
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
AgentState agent_state = WAITING_AGENT;

unsigned long prev_cmd_time = 0;
unsigned long prev_odom_time = 0;
int64_t time_offset_ms = 0;

void stopMotors()
{
    left_motor.stop();
    right_motor.stop();
}

// Sync epoch with the agent so message stamps are in ROS time, not millis().
void syncTime()
{
    unsigned long now = millis();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    int64_t ros_time_ms = rmw_uros_epoch_millis();
    time_offset_ms = ros_time_ms - (int64_t)now;
}

void stampNow(builtin_interfaces__msg__Time *stamp)
{
    int64_t now_ms = (int64_t)millis() + time_offset_ms;
    stamp->sec = now_ms / 1000;
    stamp->nanosec = (now_ms % 1000) * 1000000;
}

void twistCallback(const void *msgin)
{
    (void)msgin; // data already lands in twist_msg
    prev_cmd_time = millis();
}

void moveBase()
{
    // Failsafe: zero the command if cmd_vel went quiet.
    if (millis() - prev_cmd_time >= CMD_VEL_TIMEOUT_MS)
    {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
    }

    // BASE_LINEAR_DIR flips the body forward axis to the ROS convention
    // (+x = forward). Apply it to the command here and to the odometry below so
    // both stay consistent; angular_z is already correct and is left untouched.
    Kinematics2WD::WheelOmega req =
        kinematics.getWheelOmega(BASE_LINEAR_DIR * twist_msg.linear.x, twist_msg.angular.z);
    left_motor.setWheelAngularVelocity(req.left);
    right_motor.setWheelAngularVelocity(req.right);

    // Odometry from the motors' measured (encoder) velocity feedback.
    Kinematics2WD::Velocities vel = kinematics.getVelocities(
        left_motor.getWheelAngularVelocity(),
        right_motor.getWheelAngularVelocity());

    unsigned long now = millis();
    float dt = (now - prev_odom_time) / 1000.0;
    prev_odom_time = now;
    odometry.update(dt, BASE_LINEAR_DIR * vel.linear_x, vel.angular_z);
}

void publishData()
{
    nav_msgs__msg__Odometry *odom_msg = odometry.getData();
    stampNow(&odom_msg->header.stamp);
    RCSOFTCHECK(rcl_publish(&odom_publisher, odom_msg, NULL));

    if (imu.ok())
    {
        sensor_msgs__msg__Imu *imu_msg = imu.getData();
        stampNow(&imu_msg->header.stamp);
        RCSOFTCHECK(rcl_publish(&imu_publisher, imu_msg, NULL));
    }

    motor_current_data[0] = left_motor.getCurrentAmps();
    motor_current_data[1] = right_motor.getCurrentAmps();
    RCSOFTCHECK(rcl_publish(&motor_current_publisher, &motor_current_msg, NULL));
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL)
        return;
    ak10Poll(left_motor, right_motor);
    moveBase();
    publishData();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));
    RCCHECK(rclc_publisher_init_default(
        &motor_current_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_current"));
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(CONTROL_PERIOD_MS), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    prev_odom_time = millis();
    return true;
}

void destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&motor_current_publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    rclc_executor_fini(&executor);
    RCSOFTCHECK(rcl_node_fini(&node));
    rclc_support_fini(&support);
}

void setup()
{
    Serial.begin(921600); // USB CDC: baud is nominal, must just match the agent
    set_microros_serial_transports(Serial);

    ak10Begin();
    // If the Teensy rebooted while the motors stayed powered (firmware upload,
    // USB replug, watchdog), they are still executing the last MIT command.
    // Brake right away; motors not yet in motor mode ignore the frame.
    stopMotors();

    // AK10-9 only streams feedback frames while in motor mode, so the mode is
    // required for encoder odometry, not just commanding. But the "enter motor
    // mode" handshake twitches each shaft the instant it lands -- on a mirrored
    // drivetrain that reads as the robot jerking on every boot, and braking
    // afterwards cannot undo it. So probe for feedback first and send the
    // handshake only to motors that stay silent (a real motor power-on); on a
    // Teensy-only reboot the motors are already in motor mode and are skipped.
    delay(200); // let the CAN bus and motor controllers settle
    ak10ProbeFeedback(left_motor, right_motor, 500);
    if (!left_motor.feedbackFresh())
        ak10EnterMotorMode(LEFT_MOTOR_CMD_ID);
    if (!right_motor.feedbackFresh())
        ak10EnterMotorMode(RIGHT_MOTOR_CMD_ID);
    // Latch a damped zero-velocity hold so the motors are braked before the
    // control loop takes over.
    delay(50);
    stopMotors();
    imu.init();

    geometry_msgs__msg__Twist__init(&twist_msg);

    // Point the Float32MultiArray at its static backing store: [left, right].
    std_msgs__msg__Float32MultiArray__init(&motor_current_msg);
    motor_current_msg.data.data = motor_current_data;
    motor_current_msg.data.size = 2;
    motor_current_msg.data.capacity = 2;

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    // Drain motor feedback even while disconnected so feedbackFresh() stays
    // accurate for the recovery check below (the control timer only polls
    // while the agent is connected).
    ak10Poll(left_motor, right_motor);

    // Recovery: a motor that streams no feedback is not in motor mode (powered
    // on after the Teensy, or power-cycled), so it would ignore commands
    // forever. Re-send the enter handshake, braked immediately. The feedback
    // gate means a healthy motor is never re-twitched by this.
    EXECUTE_EVERY_N_MS(
        3000,
        if (!left_motor.feedbackFresh())
        {
            ak10EnterMotorMode(LEFT_MOTOR_CMD_ID);
            left_motor.stop();
        }
        if (!right_motor.feedbackFresh())
        {
            ak10EnterMotorMode(RIGHT_MOTOR_CMD_ID);
            right_motor.stop();
        });

    switch (agent_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(
            500,
            agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                              ? AGENT_AVAILABLE
                              : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        agent_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (agent_state == WAITING_AGENT)
            destroyEntities();
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(
            200,
            agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                              ? AGENT_CONNECTED
                              : AGENT_DISCONNECTED;);
        if (agent_state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        }
        break;
    case AGENT_DISCONNECTED:
        stopMotors();
        destroyEntities();
        agent_state = WAITING_AGENT;
        break;
    }

    // Failsafe + status LED while not connected: motors held at zero, LED off.
    if (agent_state != AGENT_CONNECTED)
    {
        EXECUTE_EVERY_N_MS(100, stopMotors(););
        digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}
