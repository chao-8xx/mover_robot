#include "headfile.h"

// 电机参数
#define PULSE_PER_CIRCLE    3200      // 一圈的脉冲数

// 丝杆控制相关参数
#define S_MM_PER_CIRCLE     2         // 一圈伸缩的距离mm
#define MM_PER_CIRCLE       8         // 一圈升降的距离mm

// 齿轮齿条控制相关参数
#define GEAR_DIAMETER       30.0f      // 齿轮直径cm

// 电机速度
#define STEP_MOVE_SPEED     1500      // 丝杆电机位置模式移动速度
#define STEP_SPEED          90        // 齿轮齿条电机位置模式移动速度

// 伸缩齿轮齿条速度模式
uint8_t stretch_speed_on = 0;         // 启动速度模式
int16_t stretch_speed = 0;            // 齿轮齿条运动速度
int8_t  stretch_state = 0;            // 用于替代线程锁 防止伸缩停止功能失灵

// 升降非阻塞计时：预计完成时刻 + busy 标志
static volatile uint8_t lift_busy = 0;      // 升降装置
static volatile uint32_t lift_deadline = 0;
static volatile uint8_t stretch_busy = 0;   // 伸缩装置
static volatile uint32_t stretch_deadline = 0;
static volatile uint8_t servo_busy = 0;      // 夹取装置
static volatile uint32_t servo_deadline = 0;

static volatile int32_t lift_pos_cmd = 0;   // 记录上一次发给驱动器的绝对目标位置

// PID参数
#define X_KP            0.7f    // x轴坐标与麦轮闭环
#define X_KD            0.2f
#define X_MAX_I         50.0f
#define X_MAX_OUT       180
#define Y_KP            0.7f    // y轴坐标与麦轮闭环
#define Y_KD            0.2f
#define Y_MAX_I         50.0f
#define Y_MAX_OUT       180
#define FOLLOW_KP       0.8f    // 物料y轴坐标与齿轮齿条伸缩闭环
#define FOLLOW_KD       0.3f
#define FOLLOW_MAX_I    10.0f
#define FOLLOW_MAX_OUT  50

// float 绝对值
static float my_absf(float data)
{
    return data < 0.0f ? -data : data;
}

// ======== 初始化 ======== //
// 抓取装置位置复位
void catch_init(void)
{
    // 处理升降电机
    motor_encoder_clear(Motor_5);   // 升降，清零点为绝对零点
    HAL_Delay(20);                  // 等清零帧真正发出去，再允许后面的位置命令
    lift_pos_cmd = 0;
    lift_busy = 0;
    lift_deadline = 0;
    motor_pos_set(Motor_5, 0);
    HAL_Delay(20);

    // 夹爪初始高度
    task_state.cur_lift_height = LIFT_ZERO_CM;
    // 夹爪初始伸出距离0.0cm
    task_state.cur_stretch_dist = STRETCH_NORMAL;

    // 处理伸缩电机
    motor_encoder_clear(Motor_6);   // 清零6号电机编码器 齿轮齿条
    HAL_Delay(20);
    motor_pos_set(Motor_6, 0);  // 账本同步归零
    task_state.cur_stretch_dist = 0.0f;

    // 舵机初始化
    servo_init();

    // 舵机复位
    holder_rotate(HOLDER_SAFE);     // 云台舵机
    turntable_rotate(ONE_LOCATION); // 转盘舵机
    claw_ctrl(CLAW_LOOSE);          // 夹爪舵机
    HAL_Delay(500);

    // 伸缩电机停止运动（为了安全）
    catch_stretch(0);   // 伸缩电机停止
    HAL_Delay(10);
    // catch_stretch_distance(STRETCH_SAFE);

    // 初始化PID参数 定位物料
    Positional_PID_Init(&x_pid,X_KP,0.0f,X_KD,
        X_MAX_I,-X_MAX_OUT,X_MAX_OUT);
    Positional_PID_Init(&y_pid,Y_KP,0.0f,Y_KD,
        Y_MAX_I,-Y_MAX_OUT,Y_MAX_OUT);
    Positional_PID_Init(&follow_pid,FOLLOW_KP,0.0f,FOLLOW_KD,
        FOLLOW_MAX_I,-FOLLOW_MAX_OUT,FOLLOW_MAX_OUT);

    // 清除pid误差与积分
    PID_Clear();
}

// ======== 齿轮齿条 ======== //
// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed)
{
    int16_t define_speed = (int16_t)(speed);
    motor_set_speed(Motor_6,define_speed);
}
// 抓取装置夹爪伸缩停止运动
void catch_stretch_stop(void)
{
    stretch_speed_on = 0;   // 关闭伸缩装置速度模式
    stretch_speed = 0;      // 设置速度
    if (stretch_state == 0 || stretch_state == 2) { //  直在等待发指令和刚发完指令期间接停止电机
        stretch_state = 3;  // 打算直接停止
        motor_set_speed(Motor_6,0);
    }
}
// 抓取装置夹爪伸缩状态赋值
void catch_stretch_send(int16_t speed)
{
    stretch_speed_on = 1;   // 启动伸缩装置速度模式
    stretch_speed = speed;  // 设置速度
}
// 伸出距离(cm) → 编码器绝对位置(脉冲)
// 齿轮齿条：一圈走的距离 = π × 齿轮直径
static int32_t stretch_dist_to_pos(float dist_cm)
{
    float cm_per_circle = 3.14159265f * GEAR_DIAMETER / 10.0f;   // mm → cm
    float p = dist_cm * (float)PULSE_PER_CIRCLE / cm_per_circle; // 伸出为正
    return (int32_t)(p + (p >= 0.0f ? 0.5f : -0.5f));
}
// 抓取装置夹爪伸缩一定距离 齿轮齿条 pos:编码器绝对位置
// 控制编码器绝对位置
void catch_stretch_distance(float distance)
{
    int32_t pos;        // 脉冲数 即电机绝对位置用脉冲数表示(ZDT官方设计)
    uint32_t pulses;    // 输出脉冲数
    float pulses_per_ms;
    uint32_t t_ms = 0;  // 时间
    static float acc = (float)STEP_MOTOR_CATCH_ACC; // 加速度

    // 超限和忙保护
    if (stretch_busy || distance < 0.0f || distance > 35.0f) return;

    // 距离 → 绝对位置
    pos = stretch_dist_to_pos(distance);

    // 直接发绝对位置，驱动器自己走过去
    motor_set_speed_Pos(Motor_6, STEP_SPEED, pos);

    // 距离 → 脉冲数
    float cm_per_circle = 3.14159265f * GEAR_DIAMETER / 10.0f;
    float f_pulses = distance * (float)PULSE_PER_CIRCLE / cm_per_circle;
    pulses = (uint32_t)(f_pulses >= 0.0f ? f_pulses + 0.5f : -f_pulses + 0.5f);

    // 时间估算：匀速 + 加减速
    pulses_per_ms = (float)STEP_SPEED * PULSE_PER_CIRCLE / 60000.0f;
    if (pulses_per_ms < 0.01f) pulses_per_ms = 0.01f;

    if (acc < 1.0f) {
        t_ms = (uint32_t)((float)pulses / pulses_per_ms);
    }
    else {
        float t_ramp_s = my_absf((float)STEP_SPEED)
                       * (256.0f - (float)STEP_MOTOR_CATCH_ACC) * 50e-6f;
        t_ms = (uint32_t)((float)pulses / pulses_per_ms + t_ramp_s);
    }

    stretch_deadline = HAL_GetTick() + t_ms + 300;   // 300ms 余量
    stretch_busy = 1;

    task_state.cur_stretch_dist = distance;

    my_control.stretch_state = 1;
}
// 获取当前伸缩距离 编码器值转化
uint8_t catch_stretch_get_dist(float *dist_cm)
{
    int32_t pos;
    if (dist_cm == NULL) return 0;

    // 尝试取上一次读到的位置
    if (!Emm_V5_Get_Pos(6, &pos)) {
        // 没有新数据，发一次读取请求，等下次回调到货
        Emm_V5_Read_Sys_Params(6, S_CPOS);
        return 0;
    }

    // 编码器位置 → 距离（cm）
    float cm_per_circle = 3.14159265f * GEAR_DIAMETER / 10.0f;
    *dist_cm = (float)pos * cm_per_circle / (float)PULSE_PER_CIRCLE;
    return 1;
}

// ======== 伸缩丝杆 ======== //
// 抓取装置夹爪伸缩一定距离 直线模组 单位：cm
// 正：伸出 负：缩回  电机顺时针伸出（pulse为负）
void catch_stretch_dist(float delta_cm)
{
    int32_t pulse = 0;
    int32_t mm = (int32_t)(delta_cm * 10);
    uint32_t pulses;
    float pulses_per_ms;

    // 伸缩装置处于忙状态，不执行
    if (stretch_busy) {
        return;
    }

    // 判断正负 pulse为负：顺时针、伸出
    if (mm >= 0) {
        pulse = -(mm * PULSE_PER_CIRCLE / S_MM_PER_CIRCLE);
    }
    else {
        pulse = (-mm) * PULSE_PER_CIRCLE / S_MM_PER_CIRCLE;
    }
    motor_set_speedPos(Motor_6,STEP_MOVE_SPEED,-pulse);

    // 根据脉冲数和电机速度估算完成时间，1ms 中断里用 catch_tick() 清除 busy
    pulses = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;
    pulses_per_ms = (float)STEP_MOVE_SPEED * PULSE_PER_CIRCLE / 60000.0f;
    if (pulses_per_ms < 0.01f) {
        pulses_per_ms = 0.01f;
    }
    stretch_deadline = HAL_GetTick()
                       + (uint32_t)((float)pulses / pulses_per_ms)
                       + 300;   // 300ms 余量
    stretch_busy = 1;

    // 刷新伸缩装置当前伸出距离
    task_state.cur_stretch_dist += delta_cm;
}
// 抓取装置到达固定位置
void catch_stretch_to_pos(float dist)
{
    // 超限保护
    if (dist > 5.5f || dist < 0.0f) {
        return;
    }

    float delta = dist - task_state.cur_stretch_dist;
    if (delta == 0.0f) {
        return;
    }
    catch_stretch_dist(delta);

    my_control.stretch_state = 1;
}


// ======== 升降丝杆 ======== //
// 绝对式换算函数
static int32_t lift_height_to_pos(float height_cm)
{
    float p = (LIFT_ZERO_CM - height_cm) * 10.0f
              * (float)PULSE_PER_CIRCLE / (float)MM_PER_CIRCLE;
    return (int32_t)(p + (p >= 0.0f ? 0.5f : -0.5f));
}
// 抓取装置夹爪升降 单位：cm
// 正：上升 负：下降  电机顺时针上升（pulse为负） 相对模式
void catch_lift(float delta_cm)
{
    int32_t pulse = 0;
    int32_t mm = (int32_t)(delta_cm * 10);
    uint32_t pulses;
    float pulses_per_ms;
    uint32_t t_ms = 0;
    static float acc = (float)STEP_MOTOR_CATCH_ACC;

    // 升降装置处于忙状态，不执行
    if (lift_busy) {
        return;
    }

    // 判断正负 pulse为负：顺时针、上升
    if (mm >= 0) {
        pulse = -(mm * PULSE_PER_CIRCLE / MM_PER_CIRCLE);
    }
    else {
        pulse = (-mm) * PULSE_PER_CIRCLE / MM_PER_CIRCLE;
    }
    motor_set_speedPos(Motor_5,STEP_MOVE_SPEED,pulse);

    // 根据脉冲数和电机速度估算完成时间，1ms 中断里用 catch_tick() 清除 busy
    pulses = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;
    pulses_per_ms = (float)STEP_MOVE_SPEED * PULSE_PER_CIRCLE / 60000.0f;
    if (pulses_per_ms < 0.01f) {
        pulses_per_ms = 0.01f;
    }

    if (acc < 1.0f) {
        t_ms = (uint32_t)((float)pulses / pulses_per_ms);
    }
    else {
        float t_ramp_s = my_absf((float)STEP_MOVE_SPEED)
                       * (256.0f - (float)STEP_MOTOR_CATCH_ACC) * 50e-6f;
        t_ms = (uint32_t)((float)pulses / pulses_per_ms + t_ramp_s);  // 梯形公式
    }

    lift_deadline = HAL_GetTick()
                     + (uint32_t)(t_ms)
                     + 1400;   // 1400ms 余量
    lift_busy = 1;
    task_state.cur_lift_height += delta_cm;
}
// 抓取装置夹爪升降至固定高度 单位:cm 绝对模式
void catch_lift_to_height(float height)
{
    // 超限保护
    if (height < -2.0f || height > 15.0f) {
        return;
    }

    // 绝对模式
    int32_t pos;
    int32_t diff;
    uint32_t pulses;
    float pulses_per_ms;
    uint32_t t_ms = 0;
    static float acc = (float)STEP_MOTOR_CATCH_ACC;

    pos  = lift_height_to_pos(height);
    diff = pos - lift_pos_cmd;                       // 本次要走的绝对位置差
    pulses = (diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff;

    // 绝对位置命令：覆盖重发同目标也是安全的
    motor_set_speed_Pos(Motor_5, STEP_MOVE_SPEED, pos);
    lift_pos_cmd = pos;

    // ---- 沿用原来的时间估算 ----
    pulses_per_ms = (float)STEP_MOVE_SPEED * PULSE_PER_CIRCLE / 60000.0f;
    if (pulses_per_ms < 0.01f) {
        pulses_per_ms = 0.01f;
    }

    if (acc < 1.0f) {
        t_ms = (uint32_t)((float)pulses / pulses_per_ms);
    }
    else {
        float t_ramp_s = my_absf((float)STEP_MOVE_SPEED)
                       * (256.0f - (float)STEP_MOTOR_CATCH_ACC) * 50e-6f;
        t_ms = (uint32_t)((float)pulses / pulses_per_ms + t_ramp_s);
    }


    if (task_state.cur_lift_height < 5) {
        lift_deadline = HAL_GetTick() + t_ms + 3000;   // 从低到高比较吃力 多留些余量
    }
    else {
        lift_deadline = HAL_GetTick() + t_ms + 1500;   // 余量
    }
    lift_busy = 1;

    task_state.cur_lift_height = height;
    my_control.lift_state = 1;
}

// ======== 舵机 ======== //
// 夹爪舵机控制角度
void claw_ctrl(float angle)
{
    servo_set_angle(Servo_3,angle);   // 夹爪舵机
}
// 云台旋转角度
void holder_rotate(float angle)
{
    servo_set_angle(Servo_1,angle);   // 云台舵机
}
// 转盘旋转角度
void turntable_rotate(float angle)
{
    servo_set_angle(Servo_2,angle);   // 转盘舵机
}

// 启动一次非阻塞等待（舵机动作时间，单位 ms）
void servo_wait_ms(uint16_t ms)
{
    servo_deadline = HAL_GetTick() + ms;
    servo_busy = 1;
}

// 夹爪动作等待是否还在进行
uint8_t servo_is_busy(void)
{
    return servo_busy;
}
// 升降是否还在执行
uint8_t lift_is_busy(void)
{
    return (lift_busy) ? 1 : 0;
}
// 伸缩是否还在执行
uint8_t stretch_is_busy(void)
{
    return stretch_busy;
}

// ======== 非阻塞计时 ======== //
// 1ms 周期调用：升降计时到点后清除 busy
void catch_tick(void)
{
    static uint8_t send_time = 0;   // 发送指令时间间隔
    // 伸缩齿轮齿条速度模式
    if (stretch_speed_on) {
        // 上一帧还没发完就等下一拍
        if (Emm_V5_Tx_Count() == 0) {
            send_time ++;
            stretch_state = 0;  // 0表示等待中
            if (send_time > 4) {
                stretch_state = 1;  // 1表示准备发送指令
                send_time = 0;
                catch_stretch(stretch_speed);
                stretch_state = 2;  // 2表示发送完指令
            }
        }
    }
    else {send_time = 0;}

    // 升降计时
    if (lift_busy && (int32_t)(HAL_GetTick() - lift_deadline) >= 0) {
        lift_busy = 0;
    }

    // 伸缩计时
    if (stretch_busy && (int32_t)(HAL_GetTick() - stretch_deadline) >= 0) {
        stretch_busy = 0;
    }

    // 舵机动作计时
    if (servo_busy && (int32_t)(HAL_GetTick() - servo_deadline) >= 0) {
        servo_busy = 0;
    }

    // hmi可控制夹取装置 和状态接收
    if (!car_run) {
        hmi_control_send_tick();
    }
}
