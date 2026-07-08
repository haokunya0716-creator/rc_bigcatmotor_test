#include <cstdio>
#include "usart.h" // 包含usart头文件以调用 huart6

#include "motor/DM4310.hpp"

static constexpr float TARGET_ANGLE_DEG = 0.0f;
static constexpr float SPEED_FF_DEG_S = 0.0f;

float angle_ref = 0;

static PID::config_t dm_angle_pid = {
    .kp = (15.0f * rpm) / (10.0f * deg),
    .ki = 0.0 * default_unit,
    .kd = 0 * default_unit,
    .max_i = 50.0f * rpm,
    .max_out = 100.0f * rpm,

};

static PID::config_t dm_speed_pid = {
    .kp = (13.0f * Nm) / (100.0f * rpm),
    .ki = 0.0 * default_unit,
    .kd = 0 * default_unit,
    .max_i = 2.0f * Nm,
    .max_out = 2.0f * Nm,
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
    .limit_min = -60 * deg,
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
    //dm_motor.SetAngle(angle_ref * deg, SPEED_FF_DEG_S * deg_s);
    dm_motor.OnLoop();
    dm_motor.SendCanCmd();
}

// ---------------- 重定向 printf 到 UART6 ----------------
extern "C" int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
// --------------------------------------------------------

extern "C" void rmpp_main()
{
    setup();

    BSP::Dwt dwt_loop;  // 给控制循环用的定时器
    BSP::Dwt dwt_print; // 给串口打印用的定时器

    while (true) {
        if (dwt_loop.PollTimeout(1 * ms)) {
            loop();
        }

        if (dwt_print.PollTimeout(50 * ms)) {
            printf("%f,%f\n", dm_motor.angle.measure.value,dm_motor.angle.ref.value);
            //printf("%f,%f\n", dm_motor.speed.ref.value, dm_motor.speed.measure.value);
        }
    }
}