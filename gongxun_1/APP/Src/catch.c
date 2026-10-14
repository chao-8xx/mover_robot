#include "headfile.h"

// 伸缩机构
#define S_MM_PER_CIRCLE     2         // 一圈伸缩的距离mm

// 丝杆控制夹爪高度变化(闭环步进电机)
#define PULSE_PER_CIRCLE  3200      // 一圈的脉冲数
#define MM_PER_CIRCLE     8         // 一圈升降的距离mm

// 电机速度
#define STEP_MOVE_SPEED     1400     // 丝杆电机位置模式移动速度
#define STEP_SPEED          200      // 齿轮齿条电机位置模式移动速度

// 升降非阻塞计时：预计完成时刻 + busy 标志
static volatile uint8_t lift_busy = 0;      // 升降装置
static volatile uint32_t lift_deadline = 0;
static volatile uint8_t stretch_busy = 0;   // 伸缩装置
static volatile uint32_t stretch_deadline = 0;
static volatile uint8_t servo_busy = 0;      // 夹取装置
static volatile uint32_t servo_deadline = 0;

static volatile int32_t lift_pos_cmd = 0;   // 记录上一次发给驱动器的绝对目标位置

// PID参数
#define X_KP            0.6f    // x轴坐标与麦轮闭环
#define X_KD            0.1f
#define X_MAX_I         50.0f
#define X_MAX_OUT       200
#define Y_KP            0.6f    // y轴坐标与麦轮闭环
#define Y_KD            0.1f
#define Y_MAX_I         50.0f
#define Y_MAX_OUT       200

// float 绝对值
static float my_absf(float data)
{
    return data < 0.0f ? -data : data;
}

// 抓取装置位置复位
void catch_init(void)
{
    motor_encoder_clear(Motor_5);   // 升降，清零点为绝对零点
    HAL_Delay(20);                  // 等清零帧真正发出去，再允许后面的位置命令
    lift_pos_cmd = 0;
    lift_busy = 0;
    lift_deadline = 0;
    motor_pos_set(Motor_5, 0);      // 补这一行

    // 夹爪初始高度
    task_state.cur_lift_height = LIFT_ZERO_CM;
    // 夹爪初始伸出距离0.0cm
    task_state.cur_stretch_dist = STRETCH_NORMAL;

    // 清零6号电机编码器 齿轮齿条
    motor_encoder_clear(Motor_6);

    // 舵机初始化
    servo_init();

    // 舵机复位
    holder_rotate(HOLDER_PUT);      // 云台舵机
    turntable_rotate(ONE_LOCATION); // 转盘舵机
    claw_ctrl(CLAW_LOOSE);          // 夹爪舵机
    HAL_Delay(500);

    // 伸缩电机停止运动（为了安全）
    catch_stretch(0);   // 伸缩电机停止
    HAL_Delay(10);

    // 初始化PID参数 定位物料
    Positional_PID_Init(&x_pid,X_KP,0.0f,X_KD,
        X_MAX_I,-X_MAX_OUT,X_MAX_OUT);
    Positional_PID_Init(&y_pid,Y_KP,0.0f,Y_KD,
        Y_MAX_I,-Y_MAX_OUT,Y_MAX_OUT);

    // 清除pid误差与积分
    PID_Reset(&x_pid);
    PID_Reset(&y_pid);
}

// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed)
{
    int16_t define_speed = (int16_t)(-speed);

    motor_set_speed(Motor_6,define_speed);
}

// 抓取装置夹爪伸缩一定距离 齿轮齿条 pos:编码器绝对位置
// 控制编码器绝对位置
void catch_stretch_ang(float angle)
{
    int32_t pos = 0;

    // 伸缩装置处于忙状态，不执行
    if (stretch_busy) {
        return;
    }

    pos = (int32_t)(angle / 1.8f * STEP_ANGLE_DIV);

    motor_set_speed_Pos(Motor_6,STEP_SPEED,pos);

    task_state.catch_cur_angle = angle;
}

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
        lift_deadline = HAL_GetTick() + t_ms + 1800;   // 余量
    }
    lift_busy = 1;

    task_state.cur_lift_height = height;
    my_control.lift_state = 1;
}

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

// 1ms 周期调用：升降计时到点后清除 busy
void catch_tick(void)
{
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
    // hmi_control_send_tick();    // 串口屏控制发送指令
}
