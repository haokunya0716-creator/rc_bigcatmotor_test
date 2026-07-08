#include <cstdio>
#include "usart.h" // 包含usart头文件以调用 huart6
#include <cmath>   // 包含 cos() 需要的数学库

#include "motor/DM4310.hpp"

// ---------------- 核心控制变量 ----------------
// 在 Ozone 调试器里，双击修改这个变量，机械臂就会指哪打哪！
// 初始设为 -40.36，正好是你标定的水平位置，一上电稳如泰山
float target_angle_deg = -40.36f;

// ---------------- 重力补偿参数 ----------------
float max_gravity_torque = 1.05f;      // 水平最大补偿扭矩
float horizontal_angle_deg = -40.36f;  // 水平基准角度
// ----------------------------------------------

static PID::config_t dm_angle_pid = {
    .kp = (10.0f * rpm) / (10.0f * deg),
    .ki = 0.0 * default_unit,
    .kd = 0 * default_unit,
    .max_i = 50.0f * rpm,
    .max_out = 100.0f * rpm,
};

static PID::config_t dm_speed_pid = {
    .kp = (10.0f * Nm) / (100.0f * rpm),
    .ki = 0.0 * default_unit,
    .kd = 0 * default_unit,
    .max_i = 2.0f * Nm,
    .max_out = 2.0f * Nm, // PID 动态输出限幅 2Nm
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

    // 使用 角度-速度 双闭环模式
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

            // 1. 软件目标限位保护 (防止调试器手抖输入错误数值)
            if (target_angle_deg < -60.0f) target_angle_deg = -60.0f;
            if (target_angle_deg > 90.0f)  target_angle_deg = 90.0f;

            // 2. 将目标角度下发给电机
            dm_motor.SetAngle(target_angle_deg * deg);

            // 3. 运行串级 PID 核心算法 (执行完后 torque.ref 已经是 PID 算出来的值)
            dm_motor.OnLoop();

            // =========================================================
            // 4. 重力前馈补偿计算
            float current_angle_deg = dm_motor.angle.measure.value;
            float angle_diff_rad = (current_angle_deg - horizontal_angle_deg) * (3.14159265f / 180.0f);
            float gravity_torque_nm = max_gravity_torque * std::cos(angle_diff_rad);

            // 5. 将“重力补偿力矩”叠加到“PID输出力矩”上！
            // 使用规范的 SetTorque 接口，带上单位 Nm，保证底层同步更新
            dm_motor.SetTorque(dm_motor.torque.ref + gravity_torque_nm * Nm);
            // =========================================================

            // 6. 发送最终指令给 CAN 硬件
            dm_motor.SendCanCmd();
        }

        if (dwt_print.PollTimeout(50 * ms)) {
            // 打印：目标角度 和 实际角度 (用于看 PID 跟随曲线)
            printf("%f,%f\n", dm_motor.angle.ref.value, dm_motor.angle.measure.value);
        }
    }
}