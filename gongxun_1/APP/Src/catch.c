#include "headfile.h"

// 夹爪舵机角度定义
#define CLAW_CATCH  20.0f    // 夹爪夹取角度
#define CLAW_LOOSE  80.0f    // 夹爪松开角度

// 转盘舵机角度定义
#define ONE_LOCATION   10.0f   // 一号物料存放角度
#define TWO_LOCATION   130.0f  // 二号物料存放角度
#define THREE_LOCATION 250.0f  // 三号物料存放角度

// 云台舵机角度定义
#define HOLDER_PUT    225.0f   // 云台放置物料角度
#define HOLDER_CATCH  90.0f    // 云台抓取中间物料角度
#define HOLDER_RIGHT  60.0f    // 云台抓取右侧物料角度
#define HOLDER_LEFT   120.0f   // 云台抓取左侧物料角度

// 伸缩机构
#define S_MM_PER_CIRCLE     2         // 一圈伸缩的距离mm

// 丝杆控制夹爪高度变化(闭环步进电机)
#define PULSE_PER_CIRCLE  3200      // 一圈的脉冲数
#define MM_PER_CIRCLE     8         // 一圈升降的距离mm
#define PICK_PUT_CM      (-1.5f)    // 拿起到放置
#define PUT_PICK_CM      (1.5f)     // 放置到拿起
#define PICK_CATCH_CM    (-15.0f)   // 拿起到抓取
#define CATCH_PICK_CM    (15.0f)    // 抓取到拿起

// 电机速度
#define STEP_MOVE_SPEED     1700     // 丝杆电机位置模式移动速度

// 升降非阻塞计时：预计完成时刻 + busy 标志
static volatile uint8_t lift_busy = 0;      // 升降装置
static volatile uint32_t lift_deadline = 0;
static volatile uint8_t stretch_busy = 0;   // 伸缩装置
static volatile uint32_t stretch_deadline = 0;
static volatile uint8_t claw_busy = 0;      // 夹取装置
static volatile uint32_t claw_deadline = 0;

// PID参数
#define FOLLOW_KP       0.05f   // y轴坐标与伸缩闭环
#define FOLLOW_KD       0.01f
#define FOLLOW_MAX_I    2.0f
#define FOLLOW_MAX_OUT  2.0f
#define X_KP            3.5f    // x轴坐标与麦轮闭环
#define X_KD            0.5f
#define X_MAX_I         50.0f
#define X_MAX_OUT       200
#define Y_KP            3.5f   // y轴坐标与麦轮闭环
#define Y_KD            0.5f
#define Y_MAX_I         50.0f
#define Y_MAX_OUT       200

// 抓取装置位置复位
void catch_init(void)
{
    // 舵机复位
    holder_rotate(HOLDER_PUT);      // 云台舵机
    turntable_rotate(ONE_LOCATION); // 转盘舵机
    claw_ctrl(CLAW_LOOSE);          // 夹爪舵机
    HAL_Delay(500);

    // 伸缩电机停止运动（为了安全）
    catch_stretch(0);   // 伸缩电机停止
    HAL_Delay(10);

    // 初始化PID参数 定位物料
    Positional_PID_Init(&follow_pid,FOLLOW_KP,0.0f,FOLLOW_KD,
        FOLLOW_MAX_I,-FOLLOW_MAX_OUT,FOLLOW_MAX_OUT);
    Positional_PID_Init(&x_pid,X_KP,0.0f,X_KD,
        X_MAX_I,-X_MAX_OUT,X_MAX_OUT);
    Positional_PID_Init(&y_pid,Y_KP,0.0f,Y_KD,
        Y_MAX_I,-Y_MAX_OUT,Y_MAX_OUT);

    // 清除pid误差与积分
    PID_Reset(&follow_pid);
    PID_Reset(&x_pid);
    PID_Reset(&y_pid);
}

// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed)
{
    int16_t define_speed = (int16_t)(-speed);

    motor_set_speed(Motor_6,define_speed);
}

// 抓取装置夹爪伸缩一定距离 单位：cm
// 正：伸出 负：缩回  电机顺时针伸出（pulse为负）
void catch_stretch_pos(float delta_cm)
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
                       + 100;   // 100ms 余量
    stretch_busy = 1;

    // 刷新伸缩装置当前伸出距离
    task_state.cur_stretch_dist += delta_cm;
}

// 抓取装置夹爪升降 单位：cm
// 正：上升 负：下降  电机顺时针上升（pulse为负）
void catch_lift(float delta_cm)
{
    int32_t pulse = 0;
    int32_t mm = (int32_t)(delta_cm * 10);
    uint32_t pulses;
    float pulses_per_ms;

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
    lift_deadline = HAL_GetTick()
                     + (uint32_t)((float)pulses / pulses_per_ms)
                     + 200;   // 200ms 余量
    lift_busy = 1;
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
void claw_wait_ms(uint16_t ms)
{
    claw_deadline = HAL_GetTick() + ms;
    claw_busy = 1;
}

// 夹爪动作等待是否还在进行
uint8_t claw_is_busy(void)
{
    return claw_busy;
}

// 升降是否还在执行
uint8_t lift_is_busy(void)
{
    return lift_busy;
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

    // 舵机/夹爪动作计时
    if (claw_busy && (int32_t)(HAL_GetTick() - claw_deadline) >= 0) {
        claw_busy = 0;
    }

    // 未发车时，hmi可控制夹取装置
    if (!car_run) {
        hmi_control_send_tick();    // 串口屏控制发送指令
    }
}
