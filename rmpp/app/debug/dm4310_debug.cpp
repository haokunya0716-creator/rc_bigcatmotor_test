#include "motor/DM4310.hpp"

static constexpr float TARGET_ANGLE_DEG = 0.0f;
static constexpr float SPEED_FF_DEG_S = 0.0f;

static PID::config_t dm_angle_pid = {
    .kp = (80.0f * rpm) / (10.0f * deg),
    .ki = 0 * default_unit,
    .kd = 0 * default_unit,
    .max_out = 100.0f * rpm,
};

static PID::config_t dm_speed_pid = {
    .kp = (2.0f * Nm) / (100.0f * rpm),
    .ki = 0 * default_unit,
    .kd = 0 * default_unit,
    .max_out = 6.0f * Nm,
};

DM4310 dm_motor({
    .can_port = 1,
    .master_id = 0x10,
    .slave_id = 0x01,

    .reduction = DM4310::REDUCTION,
    .Kt = DM4310::Kt,
    .R = DM4310::R,

    .is_invert = false,
    .offset = 0 * deg,
    .is_limit = true,
    .limit_min = -90 * deg,
    .limit_max = 90 * deg,

    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &dm_speed_pid,
    .angle_pid_config = &dm_angle_pid,
});

void setup()
{
    BSP::Init();
    dm_motor.SetEnable(true);
}

void loop()
{
    dm_motor.SetAngle(TARGET_ANGLE_DEG * deg, SPEED_FF_DEG_S * deg_s);
    dm_motor.OnLoop();
    dm_motor.SendCanCmd();
}

extern "C" void rmpp_main()
{
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}
