#include "headfile.h"

// ==================== 机械参数（按实际底盘修改） ====================
// 轮径(cm) 和电机转一圈对应的脉冲数，决定“距离 -> 脉冲”的换算
#define CHASSIS_WHEEL_DIAM_CM        7.9f       // 麦轮直径
#define CHASSIS_MOTOR_PULSE_PER_REV  3200       // 多少脉冲一圈
#define CHASSIS_CM_PER_PULSE         ((3.14159265f * CHASSIS_WHEEL_DIAM_CM) / CHASSIS_MOTOR_PULSE_PER_REV)  // cm/脉冲

// 偏航 PID：输出是电机速度(RPM)，正负方向按实际转向调整
#define CHASSIS_YAW_KP         3.8f
#define CHASSIS_YAW_KD         0.2f
#define CHASSIS_YAW_MAX_I      50.0f
#define CHASSIS_YAW_MAX_OUT    120
#define CHASSIS_YAW_SETTLE     50        // 50 个 10ms = 稳定 500ms 后才算转到位
#define CHASSIS_MOVE_MARGIN_MS 500     // 位置移动结束的额外余量ms

// 转向电机死区速度
#define ROTATE_DEAD_SPEED  0
// 横移补偿：指令距离 / 实测距离，先标定再填
#define CHASSIS_STRAFE_K  1.10f     // 原1.21f

// 底盘四台电机速度（保留，便于查看）
int16_t vel[4] = {0};

// 全局位姿使用变量
static uint8_t odom_inited = 0;         // 是否已经完成第一次采样(第一次只记录基准，不算位移)
static float   odom_last_dist[4] = {0}; // 上一次采样时四个轮子的累计位移(cm)，用来算增量
float cur_wheel_dist[4] = {0.0f};       // 当前底盘四个电机的编码器状态 cm
static uint32_t odom_hold_tick = 0;     // 最后一次"要求暂停航迹推算"的时间戳(ms)
#define ODOM_RESUME_DELAY_MS   500      // 跟随停止后，延迟多久恢复航迹推算(ms)

// 底盘运动状态
Chassis_Data my_chassis;

static volatile Chassis_Mode chassis_mode = CHASSIS_IDLE;
static volatile float chassis_target_yaw = 0.0f;
static volatile uint8_t chassis_settle = 0;     // 角度稳定帧数
static volatile uint32_t chassis_deadline = 0;
static uint8_t chassis_inited = 0;

// 分时发送状态：一次移动 = 4 条电机命令
static int32_t chassis_wheel_cmd[4];     // 位置模式=脉冲，速度模式=速度
static uint8_t chassis_send_mode = 0;    // 0=速度模式，1=位置模式
static int16_t chassis_move_speed = 0;   // 位置模式使用
static uint8_t chassis_send_index = 0;   // 下一个要发送的电机下标
static uint8_t chassis_sending = 0;      // 1 = 还在逐条发送命令

// ======== 初始化 ======== //
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

    chassis_odom_hold();    // 未发车 关闭全局位姿解算
}

// ======== 底盘麦轮解算 ======== //
// 清空底盘（停车）
static void chassis_clear(void)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = 0;
}
// 麦克纳姆轮运动学解算
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
        if (wheel[i] > 0) {
            chassis_wheel_cmd[i] = wheel[i] + ROTATE_DEAD_SPEED;
        }
        else if (wheel[i] < 0) {
            chassis_wheel_cmd[i] = wheel[i] - ROTATE_DEAD_SPEED;
        }
        else {
            chassis_wheel_cmd[i] = wheel[i];
        }
    }
    chassis_send_mode = 0;     // 速度模式
    // 只有当前没在发送时才重置序列 让car_move_send_tick把 4 个轮子依次发完
    if (!chassis_sending) {
        chassis_send_index = 0;
        chassis_sending = 1;
    }
}
// 停车
void car_stop(void)
{
    chassis_clear();
    chassis_solve();
    chassis_send_index = 0; // 强制从 0 号轮重新发送四轮停止命令，避免旧速度命令残留
    chassis_sending = 1;
    chassis_mode = CHASSIS_IDLE;
}

// ======== 底盘移动旋转控制 ======== //
// 速度模式：小车平移和旋转
void car_speed_move (int16_t x_speed, int16_t y_speed, int16_t omega_speed)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;
    }

    my_chassis.x = x_speed;
    my_chassis.y = y_speed;
    my_chassis.omega = omega_speed;
    chassis_solve();
}

// 小车旋转 只旋转不位移
static void chassis_rotate(int16_t omega)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = omega;
}

// 原地旋转
void car_rotate(int16_t omega)
{
    chassis_rotate(omega);
    chassis_solve();
}

// 一次调用：设置目标角度（复位角度版）
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
// 用偏航 PID 闭环转到目标角度，到位后调用 hwt101_yaw_reset() 复位偏航角，用于转完后复位
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

    float err = chassis_target_yaw - now_yaw;   // 不归一化，直接比较

    if (err > YAW_THRESH || err < -YAW_THRESH) {
        chassis_settle = 0;
        Positional_PID_Cal(&yaw_pid, chassis_target_yaw, now_yaw);
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

// 小车旋转目标角度（不复位角度版）
void car_rotate_yaw(float target_yaw)
{
    // 未初始化则初始化
    if (!chassis_inited) {
        chassis_init();
    }

    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    // 底盘状态为旋转
    chassis_mode = CHASSIS_ROTATE;

    float err = target_yaw - my_yaw;   // 不归一化，直接比较

    if (err > YAW_THRESH || err < -YAW_THRESH) {
        chassis_settle = 0;     // 等待时间清零
        Positional_PID_Cal(&yaw_pid, target_yaw, my_yaw);
        car_rotate((int16_t)(yaw_pid.output));
    }
    else {
        // 误差小：先停住，连续稳定 100 帧（1000ms）才算到位
        car_rotate(0);   // 只发 0 速度，保持 CHASSIS_ROTATE，直到稳定计数满
        chassis_settle++;
        if (chassis_settle >= CHASSIS_YAW_SETTLE) {
            car_stop();
            chassis_mode = CHASSIS_IDLE;    // 底盘空闲
        }
    }
}

// 位置模式：沿 (x_cm, y_cm) 走一段相对距离
static void car_move_xy(int16_t speed, float x_cm, float y_cm)
{
    int32_t px, py;
    int32_t w[4];
    float max_cm, speed_cm_s;
    static float acc = (float)STEP_MOTOR_CHASSIS_ACC;
    uint32_t t_ms = 0;

    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }

    if (!chassis_inited) {
        chassis_init();
    }

    // 距离 -> 脉冲数（四舍五入）
    px = (int32_t)((x_cm * CHASSIS_STRAFE_K) / CHASSIS_CM_PER_PULSE + (x_cm >= 0.0f ? 0.5f : -0.5f));
    py = (int32_t)(y_cm / CHASSIS_CM_PER_PULSE + (y_cm >= 0.0f ? 0.5f : -0.5f));

    // 麦克纳姆轮运动学
    w[0] =  py + px;
    w[1] = -py + px;
    w[2] =  py - px;
    w[3] = -py - px;

    // 不立即发送：先存下 4 个轮子的脉冲数，
    // 由 car_move_send_tick() 在 1ms 里分时逐条发送，避免驱动器丢帧
    for (int i = 0; i < 4; i++) {
        if (w[i] > 2147483647) {
            w[i] = 2147483647;
        }
        if (w[i] < -2147483647) {
            w[i] = -2147483647;
        }
        chassis_wheel_cmd[i] = w[i];
    }
    chassis_send_mode = 1;     // 位置模式
    chassis_move_speed = speed;
    chassis_send_index = 0;
    chassis_sending = 1;

    // 按最长轴估算移动耗时，作为 busy 结束时间
    max_cm = my_absf(x_cm) > my_absf(y_cm) ? my_absf(x_cm) : my_absf(y_cm);
    speed_cm_s = 3.14159265f * CHASSIS_WHEEL_DIAM_CM * my_absf((float)speed) / 60.0f;
    if (speed_cm_s < 0.1f) {
        speed_cm_s = 0.1f;
    }

    if (acc < 1.0f) {
        t_ms = (uint32_t)(max_cm / speed_cm_s * 1000.0f);
    }
    else {
        float t_ramp_s = my_absf((float)speed)
                     * (256.0f - (float)STEP_MOTOR_CHASSIS_ACC) * 50e-6f; // ZDT官方资料，曲线加减速时间计算公式
        t_ms = (uint32_t)(((max_cm / speed_cm_s) + t_ramp_s) * 1000.0f);  // 梯形公式
    }

    chassis_deadline = HAL_GetTick()
                       + (uint32_t)(t_ms)
                       + CHASSIS_MOVE_MARGIN_MS;
    chassis_mode = CHASSIS_MOVE;
}

// 小车沿 X/Y 轴走固定距离（cm）
void car_run_distance(int16_t speed, float x_cm, float y_cm)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }
    car_move_xy(speed, x_cm, y_cm);
}

// 底盘四个电机回到编码器零点位置
void car_back_zero(void)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }

    chassis_send_mode = 2;     // 回零模式
    chassis_send_index = 0;
    chassis_sending = 1;
}

// ======== 底盘状态 ======== //
// 是否还在执行旋转/移动
uint8_t car_is_busy(void)
{
    return chassis_mode != CHASSIS_IDLE;
}
// 获取底盘状态
Chassis_Mode car_get_state(void)
{
    return chassis_mode;
}

// ======== 全局位姿解析 ======== //
// 暂停底盘航迹推算（跟随模式下调用）
void chassis_odom_hold(void)
{
    odom_hold_tick = HAL_GetTick();
    chassis_odom_reset();      // 暂停前重置基准，避免恢复时把暂停期间的位移跳进去
    task_state.odom_on = 0;
}
// 1ms 调用：轮流给四个电机发读位置请求
void chassis_odom_poll(void)
{
    static uint8_t idx = 0;
    static uint32_t last_tick = 0;

    // 只在底盘位置移动、发送队列完全空闲时读，避免和运动命令抢串口
    if (Emm_V5_Tx_Count() != 0 || chassis_sending || chassis_mode != CHASSIS_MOVE) {
        return;
    }

    if ((int32_t)(HAL_GetTick() - last_tick) < 100) return;
    last_tick = HAL_GetTick();

    Emm_V5_Read_Sys_Params(idx + 1, S_CPOS);   // 地址 1~4
    idx = (idx + 1) & 0x03;
}
// 1ms 调用：把回复进来的位置取走，换算成累计 cm
void chassis_odom_fetch(void)
{
    for (int i = 0; i < 4; i++) {
        int32_t pos;
        if (Emm_V5_Get_Pos(i + 1, &pos)) {
            cur_wheel_dist[i] = (float)pos * CHASSIS_CM_PER_PULSE;
        }
    }
}
// 20ms 调用：用四个轮子的增量做航迹推算
void chassis_odom_update(void)
{
    static uint32_t last_tick   = 0;    // 上一次进入本函数的时间戳(ms)，用于 20ms 节流

    // 80ms 节流：没到 80ms 直接返回，避免高频重复积分
    if ((int32_t)(HAL_GetTick() - last_tick) < 400) return;
    last_tick = HAL_GetTick();

    // 第一次调用：只把当前轮子位移记成基准，不算位移
    // 否则第一次会把"从零点到现在的总位移"当成一帧增量，跳一大步
    if (!odom_inited) {
        for (int i = 0; i < 4; i++) odom_last_dist[i] = cur_wheel_dist[i];
        odom_inited = 1;
        return;
    }

    // 每个轮子的增量
    float d[4];
    for (int i = 0; i < 4; i++) {
        d[i] = cur_wheel_dist[i] - odom_last_dist[i];
        odom_last_dist[i] = cur_wheel_dist[i];
    }

    // 正运动学：轮子增量 → 车体位移
    // 逆运动学是 w0=y+x-ω, w1=-y+x-ω, w2=y-x-ω, w3=-y-x-ω
    float car_dy = ( d[0] - d[1] + d[2] - d[3]) / 4.0f;
    float car_dx = ((d[0] + d[1] - d[2] - d[3]) / 4.0f) / CHASSIS_STRAFE_K;

    // 车体系 → 全局（用陀螺仪 yaw）
    // 你的约定：yaw=0 时车头朝 (1,0)，顺时针 yaw 减小
    float rad = my_yaw * 3.14159265f / 180.0f;  // 陀螺仪偏航角 → 弧度
    float hx =  cosf(rad);       // 车头方向的全局 x 分量
    float hy = -sinf(rad);       // 车头方向的全局 y 分量
    float rx = -hy;              // 车右方向的全局 x 分量
    float ry =  hx;              // 车右方向的全局 y 分量

    // 累加到全局位姿
    task_state.x_cm += car_dy * hx + car_dx * rx;
    task_state.y_cm += car_dy * hy + car_dx * ry;
}
// 编码器清零后调用：把航迹推算的累计值和基准一起归零
void chassis_odom_reset(void)
{
    for (int i = 0; i < 4; i++) {
        cur_wheel_dist[i] = 0.0f;
    }
    odom_inited = 0;   // 让 update 下次重新初始化 last_dist
}
// 底盘全局位姿实时解算tick
void chassis_odom_tick(void)
{
    // 航迹推算
    chassis_odom_poll();     // 轮流发读位置请求（每 1ms 一个电机）
    chassis_odom_fetch();    // 取回已到达的回复，更新 cur_wheel_dist[]
    chassis_odom_update();   // 内部 20ms 节流，算增量 + 位姿
}

// ======== 非阻塞计时发送 ======== //
// 1ms 周期调用：每个周期只发一条电机命令，保证串口上有足够帧间隔
void car_move_send_tick(void)
{
    if (!chassis_sending) {
        return;
    }

    // 上一帧还没发完就等下一拍，保证串口上有真正的帧间隔
    if (Emm_V5_Tx_Count() != 0) {
        return;
    }

    static uint16_t move_time_gap = 0;

    if (chassis_send_index < 4) {
        if (chassis_send_mode == 1) {
            // 位置模式下每条指令隔3ms
            move_time_gap ++;
            if (move_time_gap >= 3) {
                move_time_gap = 0;
                motor_set_speedPos((Motor_Num)chassis_send_index,
                               chassis_move_speed,
                               chassis_wheel_cmd[chassis_send_index]);
                chassis_send_index++;
            }
        }
        else if (chassis_send_mode == 0) {
            // 速度模式下每条指令隔3ms
            move_time_gap ++;
            if (move_time_gap >= 3) {
                move_time_gap = 0;
                motor_set_speed((Motor_Num)chassis_send_index,
                            (int16_t)chassis_wheel_cmd[chassis_send_index]);
                chassis_send_index++;
            }
        }
        else if (chassis_send_mode == 2) {
            // 回零模式下每条指令隔4ms
            move_time_gap ++;
            if (move_time_gap >= 4) {
                move_time_gap = 0;
                motor_set_speed_Pos((Motor_Num)chassis_send_index,
                            (int16_t)40,(int32_t)0);
                chassis_send_index++;
            }
        }
    }
    else {
        chassis_sending = 0;     // 4 条都入队了，剩下的交给 car_run_tick 计时
    }
}

// 1ms 周期调用（可放在 1ms 定时器里）
// 只负责移动的非阻塞计时，到时间后清除busy 不做旋转控制
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

    // 1ms周期负责发送指令
    car_move_send_tick();

    // // 航迹推算开关
    // if (task_state.odom_on == 1) {
    //     chassis_odom_tick();
    // }
    // else {
    //     // 超过 500ms 没有新的跟随请求 → 认为跟随真正结束
    //     if ((int32_t)(HAL_GetTick() - odom_hold_tick) >= ODOM_RESUME_DELAY_MS) {
    //         chassis_odom_reset();
    //         task_state.odom_on = 1;
    //     }
    // }
}


