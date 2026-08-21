#include "chassis.h"

// ==================== 机械参数（按实际底盘修改） ====================
// 轮径(cm) 和电机转一圈对应的脉冲数，决定“距离 -> 脉冲”的换算
#define CHASSIS_WHEEL_DIAM_CM        6.5f       // 麦轮直径
#define CHASSIS_MOTOR_PULSE_PER_REV  3200       // 多少脉冲一圈
#define CHASSIS_CM_PER_PULSE         ((3.14159265f * CHASSIS_WHEEL_DIAM_CM) / CHASSIS_MOTOR_PULSE_PER_REV)  // cm/脉冲

// 偏航 PID：输出是电机速度(RPM)，正负方向按实际转向调整
#define CHASSIS_YAW_KP       0.7f
#define CHASSIS_YAW_KD       0.1f
#define CHASSIS_YAW_MAX_I    50.0f
#define CHASSIS_YAW_MAX_OUT  300
#define CHASSIS_YAW_THRESH   1.0f
#define CHASSIS_YAW_SETTLE   50        // 50 个 10ms = 稳定 500ms 后才算转到位
#define CHASSIS_MOVE_MARGIN_MS 500     // 位置移动结束的额外余量

// 底盘四台电机速度（保留，便于查看）
int16_t vel[4] = {0};

// 底盘运动状态
Chassis_Data my_chassis;

// 底盘当前模式：空闲 / 旋转 / 移动
typedef enum {
    CHASSIS_IDLE,
    CHASSIS_ROTATE,
    CHASSIS_MOVE
} Chassis_Mode;

static volatile Chassis_Mode chassis_mode = CHASSIS_IDLE;
static volatile float chassis_target_yaw = 0.0f;
static volatile uint8_t chassis_settle = 0;     // 角度稳定帧数
static volatile uint32_t chassis_deadline = 0;
static uint8_t chassis_inited = 0;

// float 绝对值
static float my_absf(float data)
{
    return data < 0.0f ? -data : data;
}

// 底盘初始化：配置偏航 PID
void chassis_init(void)
{
    Positional_PID_Init(&yaw_pid,
                        CHASSIS_YAW_KP,
                        0.0f,
                        CHASSIS_YAW_KD,
                        CHASSIS_YAW_MAX_I,
                        -CHASSIS_YAW_MAX_OUT,
                        CHASSIS_YAW_MAX_OUT);
    PID_Reset(&yaw_pid);
    chassis_mode = CHASSIS_IDLE;
    chassis_inited = 1;
}

// 小车旋转 只旋转不位移
static void chassis_rotate(int16_t omega)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = omega;
}

// 清空底盘（停车）
static void chassis_clear(void)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = 0;
}

// 麦克纳姆轮运动学解算：
// wheel[i] 的正负通过 motor_set_speed 转换成方向，绝对值是速度
static void chassis_solve(void)
{
    int wheel[4];   // 四个轮子的速度（正负表示方向）

    wheel[0] = my_chassis.y + my_chassis.x - my_chassis.omega;
    wheel[1] = -my_chassis.y + my_chassis.x - my_chassis.omega;
    wheel[2] = my_chassis.y - my_chassis.x - my_chassis.omega;
    wheel[3] = -my_chassis.y - my_chassis.x - my_chassis.omega;

    for (int i = 0; i < 4; i++) {
        vel[i] = (int16_t)wheel[i];
        motor_set_speed((Motor_Num)i, (int16_t)wheel[i]);
    }
}

// 停车
void car_stop(void)
{
    chassis_clear();
    chassis_solve();
    chassis_mode = CHASSIS_IDLE;
}

// 原地旋转
void car_rotate(int16_t omega)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙（旋转/移动中）时不接受新目标，避免互相覆盖
    }

    chassis_rotate(omega);
    chassis_solve();
    chassis_mode = CHASSIS_ROTATE;
}

// 一次调用：设置目标角度
// 非阻塞：只设置目标，实际由 car_rotate_ctrl() 每 10ms 驱动闭环
void car_rotate_angle(float target_angle)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙（旋转/移动中）时不接受新目标，避免互相覆盖
    }

    if (!chassis_inited) {
        chassis_init();
    }

    chassis_target_yaw = target_angle;
    chassis_settle = 0;
    PID_Reset(&yaw_pid);
    chassis_mode = CHASSIS_ROTATE;
}

// 10ms 周期调用（放 TIM2 中断或主循环）：
// 用偏航 PID 闭环转到目标角度，到位后调用 hwt101_yaw_reset() 复位偏航角
void car_rotate_ctrl(void)
{
    // 未初始化则初始化
    if (!chassis_inited) {
        chassis_init();
    }

    // 不处于旋转状态，不执行
    if (chassis_mode != CHASSIS_ROTATE) {
        return;
    }

    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    float err = chassis_target_yaw - my_yaw;   // 不归一化，直接比较

    if (err > CHASSIS_YAW_THRESH || err < -CHASSIS_YAW_THRESH) {
        chassis_settle = 0;
        Positional_PID_Cal(&yaw_pid, chassis_target_yaw, my_yaw);
        car_rotate((int16_t)yaw_pid.output);
    }
    else {
        // 误差小：先停住，连续稳定 50 帧（500ms）才算到位
        car_rotate(0);
        chassis_settle++;
        if (chassis_settle >= CHASSIS_YAW_SETTLE) {
            car_stop();
            hwt101_yaw_reset();         // 到位后把偏航角复位为 0
            chassis_target_yaw = 0.0f;  // 底盘目标角度也复位
        }
    }
}

// 位置模式：沿 (x_cm, y_cm) 走一段相对距离
static void car_move_xy(int16_t speed, float x_cm, float y_cm)
{
    int32_t px, py;
    int32_t w[4];
    float max_cm, speed_cm_s;

    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }

    if (!chassis_inited) {
        chassis_init();
    }

    // 距离 -> 脉冲数（四舍五入）
    px = (int32_t)(x_cm / CHASSIS_CM_PER_PULSE + (x_cm >= 0.0f ? 0.5f : -0.5f));
    py = (int32_t)(y_cm / CHASSIS_CM_PER_PULSE + (y_cm >= 0.0f ? 0.5f : -0.5f));

    // 麦克纳姆轮运动学
    w[0] =  py + px;
    w[1] = -py + px;
    w[2] =  py - px;
    w[3] = -py - px;

    // 发给四个电机（位置模式），脉冲数限制在 int32 范围内
    for (int i = 0; i < 4; i++) {
        if (w[i] > 2147483647) {
            w[i] = 2147483647;
        }
        if (w[i] < -2147483647) {
            w[i] = -2147483647;
        }
        motor_set_speedPos((Motor_Num)i, speed, (int32_t)w[i]);   // pulse 正负决定方向
    }

    // 按最长轴估算移动耗时，作为 busy 结束时间
    max_cm = my_absf(x_cm) > my_absf(y_cm) ? my_absf(x_cm) : my_absf(y_cm);
    speed_cm_s = 3.14159265f * CHASSIS_WHEEL_DIAM_CM * my_absf((float)speed) / 60.0f;
    if (speed_cm_s < 0.1f) {
        speed_cm_s = 0.1f;
    }

    chassis_deadline = HAL_GetTick()
                       + (uint32_t)(max_cm / speed_cm_s * 1000.0f)
                       + CHASSIS_MOVE_MARGIN_MS;
    chassis_mode = CHASSIS_MOVE;
}

// 小车沿 X/Y 轴走固定距离（cm）
void car_run_distance(int16_t speed, float x_cm, float y_cm)
{
    car_move_xy(speed, x_cm, y_cm);
}

// 1ms 周期调用（可放在 1ms 定时器里）：
// 只负责移动的非阻塞计时，到时间后清除 busy，不做旋转控制
void car_run_tick(void)
{
    if (!chassis_inited) {
        chassis_init();
    }

    if (chassis_mode == CHASSIS_MOVE) {
        // 位置模式由驱动器自己走完，这里只负责结束 busy 状态
        if ((int32_t)(HAL_GetTick() - chassis_deadline) >= 0) {
            chassis_mode = CHASSIS_IDLE;
        }
    }
}

// 是否还在执行旋转/移动
uint8_t car_is_busy(void)
{
    return chassis_mode != CHASSIS_IDLE;
}
