// Host-side simulation of the Teensy boot sequence against the REAL
// ak10_mit.h + config.h, with the setup()/loop() motor logic copied verbatim
// from main.cpp. Models the AK10-9s on the other end of the CAN bus:
//   - a powered motor in motor mode streams a servo status frame every 20 ms
//     (CAN id 0x2900 | motor_id), and JERKS when it receives the MIT
//     "enter motor mode" handshake (FF..FC)
//   - a motor not in motor mode streams nothing and ignores MIT commands
//   - an unpowered motor ignores everything
//
// Scenarios:
//   A. Teensy gets power while motors are already up (the reported problem)
//      -> must send ZERO handshakes: no jerk.
//   B. Whole robot cold boot (motors not yet in motor mode)
//      -> exactly one handshake per motor, each followed by a brake.
//   C. Motor power arrives AFTER the Teensy booted
//      -> recovery enables it within ~3 s, braked immediately, and never
//         re-sends the handshake once the motor streams.
//   D. Teensy reboots mid-drive (motors executing an old velocity command)
//      -> a brake frame goes out within a few ms of boot, before the 200 ms
//         settle delay.
//   E. A wheel stalls mid-drive (sustained overcurrent)
//      -> the firmware latches a stop after OVERCURRENT_MS, refuses further
//         drive commands, and releases only on a zero cmd_vel. A brief spike
//         shorter than OVERCURRENT_MS must NOT latch.
//   F. One wheel runs slower than commanded (friction mismatch -> veer)
//      -> the per-wheel trim ramps that wheel's command up (clamped at
//         WHEEL_TRIM_MAX), leaves the on-speed wheel alone, and resets on a
//         zero command so a stale trim can't lurch the robot from rest.
#include "ak10_mit.h"
#include "kinematics.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

uint32_t fake_now = 0;
bool (*flexcan_read_hook)(CAN_message_t &) = nullptr;
void (*flexcan_write_hook)(const CAN_message_t &) = nullptr;

// ---------------------------------------------------------------- fake motors
struct FakeMotor
{
    uint8_t id;
    uint32_t cmd_id;
    bool powered = false;
    bool in_motor_mode = false;
    uint32_t last_stream_ms = 0;
    int jerks = 0;             // handshakes received while powered
    float current_amps = 0.0f; // reported in the status frame (scenario E)
    int16_t erpm = 0;          // reported in the status frame (scenario F)
};
FakeMotor fm_left{LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID};
FakeMotor fm_right{RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID};

struct SentFrame
{
    uint32_t t;
    uint32_t id;
    bool is_handshake;
    bool is_brake; // MIT frame with kp=0, v_des=0 (mid-scale), kd>0
    bool is_drive; // MIT frame with kp=0, kd>0, v_des != 0
    float v_des;   // decoded MIT velocity command (motor frame, rad/s)
};
std::vector<SentFrame> sent;

static bool isHandshake(const CAN_message_t &m)
{
    if (m.len != 8 || m.buf[7] != 0xFC) return false;
    for (int i = 0; i < 7; i++)
        if (m.buf[i] != 0xFF) return false;
    return true;
}

static bool isBrake(const CAN_message_t &m)
{
    if (m.len != 8 || isHandshake(m)) return false;
    uint32_t kp = ((uint32_t)m.buf[0] << 4) | (m.buf[1] >> 4);
    uint32_t kd = (((uint32_t)m.buf[1] & 0xF) << 8) | m.buf[2];
    uint32_t v = ((uint32_t)m.buf[5] << 4) | (m.buf[6] >> 4);
    return kp == 0 && kd > 0 && v == 2047; // 2047 = zero velocity mid-scale
}

static bool isDrive(const CAN_message_t &m)
{
    if (m.len != 8 || isHandshake(m)) return false;
    uint32_t kp = ((uint32_t)m.buf[0] << 4) | (m.buf[1] >> 4);
    uint32_t kd = (((uint32_t)m.buf[1] & 0xF) << 8) | m.buf[2];
    uint32_t v = ((uint32_t)m.buf[5] << 4) | (m.buf[6] >> 4);
    return kp == 0 && kd > 0 && v != 2047; // nonzero velocity command
}

static float mitVdes(const CAN_message_t &m)
{
    if (m.len != 8 || isHandshake(m)) return 0.0f;
    uint32_t v = ((uint32_t)m.buf[5] << 4) | (m.buf[6] >> 4);
    return ak10_uint_to_float(v, V_MIN, V_MAX, 12);
}

static void busWrite(const CAN_message_t &m)
{
    sent.push_back({fake_now, m.id, isHandshake(m), isBrake(m), isDrive(m),
                    mitVdes(m)});
    for (FakeMotor *fm : {&fm_left, &fm_right})
    {
        if (m.id != fm->cmd_id || !fm->powered) continue;
        if (isHandshake(m))
        {
            if (fm->in_motor_mode) fm->jerks++; // re-enable transient
            else { fm->in_motor_mode = true; fm->jerks++; } // enable transient
        }
    }
}

static bool busRead(CAN_message_t &m)
{
    for (FakeMotor *fm : {&fm_left, &fm_right})
    {
        if (!fm->powered || !fm->in_motor_mode) continue;
        if (fake_now - fm->last_stream_ms >= 20)
        {
            fm->last_stream_ms = fake_now;
            m = CAN_message_t{};
            m.id = 0x2900 | fm->id; // servo status frame, low byte = motor id
            m.len = 8;
            m.buf[0] = 0x7D; m.buf[1] = 0x00; // marker
            m.buf[2] = (uint8_t)(((uint16_t)fm->erpm) >> 8);   // ERPM hi
            m.buf[3] = (uint8_t)(((uint16_t)fm->erpm) & 0xFF); // ERPM lo
            int16_t cur_raw = (int16_t)(fm->current_amps / CURRENT_LSB_TO_AMP);
            m.buf[4] = (uint8_t)(((uint16_t)cur_raw) >> 8);
            m.buf[5] = (uint8_t)(((uint16_t)cur_raw) & 0xFF);
            return true;
        }
    }
    return false;
}

// ------------------------------------------------- firmware logic under test
AK10 *left_motor;
AK10 *right_motor;

void stopMotors()
{
    left_motor->stop();
    right_motor->stop();
}

// Copied verbatim from main.cpp
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

void setup_motor_fragment() // motor-related part of setup(), verbatim
{
    ak10Begin();
    stopMotors();
    // 2026-07-09 hardware finding: real AK10-9s stream their status frame
    // whether or not MIT mode is enabled, so "is it streaming?" cannot gate
    // the handshake -- the old probe-and-skip left the motors ignoring every
    // command after a battery power-cycle. Handshake unconditionally: the
    // small enable transient is the price of guaranteed drivability.
    delay(200);
    // Brake each motor in the SAME millisecond as its enable: the handshake
    // makes the motor execute whatever command it has latched, and the old
    // 50 ms gap before the first brake let that run unopposed -- the robot
    // visibly moved at every boot.
    ak10EnterMotorMode(LEFT_MOTOR_CMD_ID);
    left_motor->stop();
    ak10EnterMotorMode(RIGHT_MOTOR_CMD_ID);
    right_motor->stop();
    // Hold the brake while the enable transient damps out instead of trusting
    // a single frame the controller may drop mid mode-switch.
    for (int i = 0; i < BOOT_BRAKE_HOLD_MS / BOOT_BRAKE_PERIOD_MS; i++)
    {
        delay(BOOT_BRAKE_PERIOD_MS);
        stopMotors();
    }
}

void loop_motor_fragment() // motor-related part of loop(), verbatim
{
    ak10Poll(*left_motor, *right_motor);

    EXECUTE_EVERY_N_MS(
        3000,
        if (!left_motor->feedbackFresh())
        {
            ak10EnterMotorMode(LEFT_MOTOR_CMD_ID);
            left_motor->stop();
        }
        if (!right_motor->feedbackFresh())
        {
            ak10EnterMotorMode(RIGHT_MOTOR_CMD_ID);
            right_motor->stop();
        });
}

// Overcurrent latch + drive gating, mirrored from main.cpp (checkOvercurrent
// and the moveBase command path; odometry/micro-ROS parts omitted -- they
// don't compile on host and don't affect the latch).
Kinematics2WD kinematics;
struct { struct { double x = 0; } linear; struct { double z = 0; } angular; } twist_msg;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_time = 0;

bool overcurrent_latched = false;
bool left_over = false, right_over = false;
unsigned long left_over_since = 0, right_over_since = 0;

void checkOvercurrent()
{
    unsigned long now = millis();

    float amps = fabsf(left_motor->getCurrentAmps());
    if (amps > OVERCURRENT_AMPS)
    {
        if (!left_over) { left_over = true; left_over_since = now; }
    }
    else
        left_over = false;

    amps = fabsf(right_motor->getCurrentAmps());
    if (amps > OVERCURRENT_AMPS)
    {
        if (!right_over) { right_over = true; right_over_since = now; }
    }
    else
        right_over = false;

    if ((left_over && now - left_over_since >= OVERCURRENT_MS) ||
        (right_over && now - right_over_since >= OVERCURRENT_MS))
        overcurrent_latched = true;
}

// Per-wheel closed-loop speed trim, mirrored from main.cpp (see config.h).
float left_trim = 0.0f, right_trim = 0.0f;

void updateWheelTrim(float &trim, float req, AK10 &motor, float dt)
{
    if (fabsf(req) < WHEEL_TRIM_MIN_CMD)
        trim = 0.0f;
    else if (motor.feedbackFresh() && dt > 0.0f && dt < 0.1f)
    {
        trim += WHEEL_TRIM_KI * (req - motor.getWheelAngularVelocity()) * dt;
        trim = constrain(trim, -WHEEL_TRIM_MAX, WHEEL_TRIM_MAX);
    }
}

void moveBase_motor_fragment() // command path of moveBase(), verbatim
{
    if (millis() - prev_cmd_time >= CMD_VEL_TIMEOUT_MS)
    {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
    }

    checkOvercurrent();
    if (overcurrent_latched &&
        twist_msg.linear.x == 0.0 && twist_msg.angular.z == 0.0)
    {
        overcurrent_latched = false;
        left_over = right_over = false;
    }

    unsigned long now = millis();
    float dt = (now - prev_odom_time) / 1000.0;
    prev_odom_time = now;

    if (overcurrent_latched)
    {
        left_trim = right_trim = 0.0f;
        stopMotors();
    }
    else
    {
        Kinematics2WD::WheelOmega req =
            kinematics.getWheelOmega(BASE_LINEAR_DIR * twist_msg.linear.x, twist_msg.angular.z);
        updateWheelTrim(left_trim, req.left, *left_motor, dt);
        updateWheelTrim(right_trim, req.right, *right_motor, dt);
        left_motor->setWheelAngularVelocity(req.left + left_trim);
        right_motor->setWheelAngularVelocity(req.right + right_trim);
    }
}

// One 50 Hz control tick with an active cmd_vel publisher: refresh the command
// (as twistCallback would), poll feedback, run the moveBase command path.
static void driveTick(double lin, double ang)
{
    twist_msg.linear.x = lin;
    twist_msg.angular.z = ang;
    prev_cmd_time = millis();
    ak10Poll(*left_motor, *right_motor);
    moveBase_motor_fragment();
    delay(CONTROL_PERIOD_MS);
}

// -------------------------------------------------------------------- runner
int failures = 0;
#define CHECK(cond, msg)                                        \
    do {                                                        \
        if (cond) printf("  PASS: %s\n", msg);                  \
        else { printf("  FAIL: %s\n", msg); failures++; }       \
    } while (0)

static void teensyPowerOn() // fresh Teensy: new driver objects, clock reset
{
    fake_now = 0;
    sent.clear();
    fm_left.jerks = fm_right.jerks = 0;
    fm_left.last_stream_ms = fm_right.last_stream_ms = 0;
    static AK10 l1(LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID, LEFT_MOTOR_DIR),
        r1(RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID, RIGHT_MOTOR_DIR),
        l2(LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID, LEFT_MOTOR_DIR),
        r2(RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID, RIGHT_MOTOR_DIR),
        l3(LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID, LEFT_MOTOR_DIR),
        r3(RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID, RIGHT_MOTOR_DIR),
        l4(LEFT_MOTOR_ID, LEFT_MOTOR_CMD_ID, LEFT_MOTOR_DIR),
        r4(RIGHT_MOTOR_ID, RIGHT_MOTOR_CMD_ID, RIGHT_MOTOR_DIR);
    static AK10 *pool_l[] = {&l1, &l2, &l3, &l4};
    static AK10 *pool_r[] = {&r1, &r2, &r3, &r4};
    static int n = 0;
    left_motor = pool_l[n];
    right_motor = pool_r[n];
    n++;
}

static int handshakesSent()
{
    int c = 0;
    for (auto &f : sent) c += f.is_handshake;
    return c;
}

// Every handshake must be followed by a brake to the SAME cmd_id in the SAME
// millisecond -- any gap is time the motor spends running its latched command
// unopposed (the boot-movement bug).
static bool everyHandshakeBrakedSameMs()
{
    for (size_t i = 0; i < sent.size(); i++)
    {
        if (!sent[i].is_handshake) continue;
        bool braked = false;
        for (size_t j = i + 1; j < sent.size(); j++)
            if (sent[j].id == sent[i].id && sent[j].is_brake)
            {
                braked = sent[j].t == sent[i].t;
                break;
            }
        if (!braked) return false;
    }
    return true;
}

// The brake must be actively held (re-sent) for the full hold window after
// the last handshake, not fired once and forgotten.
static bool brakeHeldAfterLastHandshake()
{
    uint32_t last_hs = 0, last_brake = 0;
    for (auto &f : sent)
    {
        if (f.is_handshake) last_hs = f.t;
        if (f.is_brake) last_brake = f.t;
    }
    return last_brake >= last_hs + BOOT_BRAKE_HOLD_MS - BOOT_BRAKE_PERIOD_MS;
}

int main()
{
    flexcan_read_hook = busRead;
    flexcan_write_hook = busWrite;

    printf("Scenario A: Teensy gets power, motors already up & in motor mode\n");
    fm_left.powered = fm_right.powered = true;
    fm_left.in_motor_mode = fm_right.in_motor_mode = true;
    teensyPowerOn();
    setup_motor_fragment();
    // The handshake is unconditional (see setup_motor_fragment), so a
    // re-enable transient per motor is ACCEPTED here -- the alternative
    // (probe-gating) left real motors undriveable after a battery cycle.
    CHECK(handshakesSent() == 2, "handshake sent to both motors (by design)");
    CHECK(fm_left.jerks == 1 && fm_right.jerks == 1,
          "at most one enable transient per motor, immediately braked");
    CHECK(everyHandshakeBrakedSameMs(),
          "each enable braked in the same millisecond (no unopposed window)");
    CHECK(brakeHeldAfterLastHandshake(),
          "brake actively held for the full boot hold window");
    CHECK(!sent.empty() && sent.back().is_brake, "ends holding a brake");
    // setup() no longer polls; feedback lands on the first loop() passes.
    for (int i = 0; i < 100; i++) loop_motor_fragment();
    CHECK(left_motor->feedbackFresh() && right_motor->feedbackFresh(),
          "feedback flowing once loop() runs (odometry alive)");

    printf("Scenario B: whole-robot cold boot (motors powered, not in motor mode)\n");
    fm_left.in_motor_mode = fm_right.in_motor_mode = false;
    teensyPowerOn();
    setup_motor_fragment();
    CHECK(handshakesSent() == 2, "exactly one handshake per motor");
    CHECK(fm_left.jerks == 1 && fm_right.jerks == 1,
          "one unavoidable enable transient per motor (cold power-on only)");
    CHECK(fm_left.in_motor_mode && fm_right.in_motor_mode, "motors enabled");
    CHECK(everyHandshakeBrakedSameMs(),
          "each enable braked in the same millisecond (no unopposed window)");
    CHECK(brakeHeldAfterLastHandshake(),
          "brake actively held for the full boot hold window");
    CHECK(!sent.empty() && sent.back().is_brake, "ends holding a brake");

    printf("Scenario C: motor power arrives after the Teensy booted\n");
    fm_left.powered = fm_right.powered = false;
    fm_left.in_motor_mode = fm_right.in_motor_mode = false;
    teensyPowerOn();
    setup_motor_fragment(); // handshake goes to dead bus, motors ignore it
    fm_left.powered = fm_right.powered = true; // battery switched on now
    uint32_t t_on = fake_now;
    while (fake_now - t_on < 8000 && !(fm_left.in_motor_mode && fm_right.in_motor_mode))
        loop_motor_fragment();
    CHECK(fm_left.in_motor_mode && fm_right.in_motor_mode,
          "recovery enabled both motors");
    CHECK(fake_now - t_on <= 3500, "enabled within ~3 s of motor power-on");
    int hs_after_enable = handshakesSent();
    for (int i = 0; i < 20000; i++) loop_motor_fragment(); // run on
    CHECK(handshakesSent() == hs_after_enable,
          "no handshake re-sent once motors stream (no periodic twitching)");

    printf("Scenario D: Teensy reboots mid-drive\n");
    fm_left.powered = fm_right.powered = true;
    fm_left.in_motor_mode = fm_right.in_motor_mode = true;
    teensyPowerOn();
    setup_motor_fragment();
    uint32_t first_brake = UINT32_MAX;
    uint32_t first_handshake = UINT32_MAX;
    for (auto &f : sent)
    {
        if (f.is_brake && first_brake == UINT32_MAX) first_brake = f.t;
        if (f.is_handshake && first_handshake == UINT32_MAX) first_handshake = f.t;
    }
    CHECK(first_brake < 50, "brake frame sent within 50 ms of boot");
    CHECK(first_brake < first_handshake,
          "brake goes out before the enable handshake");

    printf("Scenario E: wheel stall -> overcurrent latch\n");
    // Continue from scenario D's healthy state: both motors up and streaming.
    size_t mark = sent.size();
    fm_left.current_amps = fm_right.current_amps = 0.5f; // normal draw
    for (int i = 0; i < 100; i++) driveTick(0.05, 0.0);
    CHECK(!overcurrent_latched && sent.back().is_drive,
          "normal driving passes velocity commands through");

    // Brief spike shorter than OVERCURRENT_MS must not latch.
    fm_left.current_amps = OVERCURRENT_AMPS + 4.0f;
    uint32_t t0 = fake_now;
    while (fake_now - t0 < OVERCURRENT_MS / 2) driveTick(0.05, 0.0);
    fm_left.current_amps = 0.5f;
    for (int i = 0; i < 50; i++) driveTick(0.05, 0.0);
    CHECK(!overcurrent_latched, "sub-threshold-duration spike does not latch");

    // Sustained stall: left wheel jams while a drive command is active.
    fm_left.current_amps = OVERCURRENT_AMPS + 4.0f;
    t0 = fake_now;
    while (fake_now - t0 < OVERCURRENT_MS + 500) driveTick(0.05, 0.0);
    CHECK(overcurrent_latched, "sustained overcurrent latches a stop");
    mark = sent.size();
    for (int i = 0; i < 50; i++) driveTick(0.05, 0.0); // keep commanding
    bool all_brakes = true;
    for (size_t i = mark; i < sent.size(); i++)
        if (!sent[i].is_brake) all_brakes = false;
    CHECK(all_brakes && sent.size() > mark,
          "latched: drive commands are refused, only brakes go out");

    // Zero command releases the latch; driving then resumes.
    fm_left.current_amps = 0.5f; // jam cleared
    for (int i = 0; i < 5; i++) driveTick(0.0, 0.0);
    CHECK(!overcurrent_latched, "zero cmd_vel releases the latch");
    for (int i = 0; i < 50; i++) driveTick(0.05, 0.0);
    CHECK(!overcurrent_latched && sent.back().is_drive,
          "driving resumes after release");

    printf("Scenario F: left wheel runs slow -> per-wheel trim compensates\n");
    // Reset the trims with zero commands, then have the fake motors report
    // measured speed: left at 50% of the commanded wheel speed, right dead on.
    for (int i = 0; i < 5; i++) driveTick(0.0, 0.0);
    const float req_wheel =
        (float)(BASE_LINEAR_DIR * 0.05 / (WHEEL_DIAMETER / 2.0)); // robot conv
    fm_left.erpm =
        (int16_t)lrintf(0.5f * req_wheel / (LEFT_MOTOR_DIR * ERPM_TO_WHEEL_RADPS));
    fm_right.erpm =
        (int16_t)lrintf(req_wheel / (RIGHT_MOTOR_DIR * ERPM_TO_WHEEL_RADPS));
    size_t f_mark = sent.size();
    for (int i = 0; i < 400; i++) driveTick(0.05, 0.0); // 8 s of driving
    float l_first = 0, l_last = 0, r_first = 0, r_last = 0;
    bool l_seen = false, r_seen = false;
    for (size_t i = f_mark; i < sent.size(); i++)
    {
        if (!sent[i].is_drive) continue;
        if (sent[i].id == LEFT_MOTOR_CMD_ID)
        {
            if (!l_seen) { l_first = sent[i].v_des; l_seen = true; }
            l_last = sent[i].v_des;
        }
        if (sent[i].id == RIGHT_MOTOR_CMD_ID)
        {
            if (!r_seen) { r_first = sent[i].v_des; r_seen = true; }
            r_last = sent[i].v_des;
        }
    }
    CHECK(l_seen && r_seen, "drive frames reached both motors");
    CHECK(fabsf(l_last) > fabsf(l_first) + 0.5f,
          "slow wheel's command ramps up (trim integrating)");
    CHECK(fabsf(l_last) <= fabsf(req_wheel) + WHEEL_TRIM_MAX + 0.05f,
          "trim respects its clamp");
    CHECK(fabsf(r_last - r_first) < 0.15f,
          "on-speed wheel's command stays put");
    // A stop must reset the trim: the first drive frame after restarting from
    // rest must not carry the stale trim.
    for (int i = 0; i < 5; i++) driveTick(0.0, 0.0);
    f_mark = sent.size();
    driveTick(0.05, 0.0);
    float l_resume = 0; bool resume_seen = false;
    for (size_t i = f_mark; i < sent.size(); i++)
        if (sent[i].is_drive && sent[i].id == LEFT_MOTOR_CMD_ID)
        {
            l_resume = sent[i].v_des; resume_seen = true; break;
        }
    CHECK(resume_seen && fabsf(fabsf(l_resume) - fabsf(req_wheel)) < 0.1f,
          "trim resets at zero command (no lurch on restart)");

    printf(failures ? "\n%d FAILURE(S)\n" : "\nALL SCENARIOS PASS\n", failures);
    return failures ? 1 : 0;
}
