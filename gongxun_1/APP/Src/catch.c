#include "catch.h"

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

// 丝杆控制夹爪高度变化(闭环步进电机)
#define PULSE_PER_CIRCLE   3200     // 一圈的脉冲数
#define MM_PER_CIRCLE      2        // 一圈升降的距离mm
#define HIGH_PICK_CM     (-8.0f)    // 夹爪从最高点到拿起位置cm
#define PICK_HIGH_CM     (8.0f)     // 夹爪从拿起位置到最高点
#define PICK_PUT_CM      (-2.0f)    // 拿起到放置
#define PUT_PICK_CM      (2.0f)     // 放置到拿起
#define PICK_CATCH_CM    (-4.0f)    // 拿起到抓取
#define CATCH_PICK_CM    (4.0f)     // 抓取到拿起

// 电机速度
#define STEP_MOVE_SPEED     1200     // 丝杆电机移动速度

Catch_Mode catch_mode = Free;           // 抓取机构状态机
Claw_State claw_state = LOOSE;          // 夹爪当前状态
Turntable_Pos turn_pos = LOCATION_1;    // 转盘当前位置

// 升降非阻塞计时：预计完成时刻 + busy 标志
static volatile uint8_t catch_busy = 0;     // 升降装置
static volatile uint8_t stretch_busy = 0;   // 伸缩装置
static volatile uint32_t catch_deadline = 0;
static volatile uint32_t stretch_deadline = 0;

// 舵机等无反馈动作的通用非阻塞计时
static volatile uint8_t catch_servo_busy = 0;
static volatile uint32_t catch_servo_deadline = 0;

// 抓取装置位置复位
void catch_init(void)
{
    holder_rotate(HOLDER_PUT);      // 云台舵机
    turntable_rotate(ONE_LOCATION); // 转盘舵机
    claw_ctrl(CLAW_LOOSE);          // 夹爪舵机
    HAL_Delay(500);

    catch_lift(HIGH_PICK_CM);       // 夹爪初始化高度
    // 阻塞等升降真正到位（等2s）
    uint32_t t0 = HAL_GetTick();
    while (catch_is_busy() && (HAL_GetTick() - t0 < 2000)) {
        HAL_Delay(1);
    }
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

    // 升降装置处于忙状态，不执行
    if (stretch_busy) {
        return;
    }

    // 判断正负 pulse为负：顺时针、伸出
    if (mm >= 0) {
        pulse = -(mm * PULSE_PER_CIRCLE / MM_PER_CIRCLE);
    }
    else {
        pulse = (-mm) * PULSE_PER_CIRCLE / MM_PER_CIRCLE;
    }
    motor_set_speedPos(Motor_6,STEP_MOVE_SPEED,pulse);

    // 根据脉冲数和电机速度估算完成时间，1ms 中断里用 catch_tick() 清除 busy
    pulses = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;
    pulses_per_ms = (float)STEP_MOVE_SPEED * PULSE_PER_CIRCLE / 60000.0f;
    if (pulses_per_ms < 0.01f) {
        pulses_per_ms = 0.01f;
    }
    stretch_deadline = HAL_GetTick()
                       + (uint32_t)((float)pulses / pulses_per_ms)
                       + 200;   // 200ms 余量
    stretch_busy = 1;
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
    if (catch_busy) {
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
    catch_deadline = HAL_GetTick()
                     + (uint32_t)((float)pulses / pulses_per_ms)
                     + 200;   // 200ms 余量
    catch_busy = 1;
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

// 夹爪舵机控制角度
void claw_ctrl(float angle)
{
    servo_set_angle(Servo_3,angle);   // 夹爪舵机
}

// 启动一次非阻塞等待（舵机动作时间，单位 ms）
void catch_wait_servo(uint16_t ms)
{
    catch_servo_deadline = HAL_GetTick() + ms;
    catch_servo_busy = 1;
}

// 舵机动作等待是否还在进行
uint8_t catch_servo_waiting(void)
{
    return catch_servo_busy;
}

// 升降是否还在执行
uint8_t catch_is_busy(void)
{
    return catch_busy;
}

// 1ms 周期调用：升降计时到点后清除 busy
void catch_tick(void)
{
    // 升降计时
    if (catch_busy && (int32_t)(HAL_GetTick() - catch_deadline) >= 0) {
        catch_busy = 0;
    }

    // 伸缩计时
    if (stretch_busy && (int32_t)(HAL_GetTick() - stretch_deadline) >= 0) {
        stretch_busy = 0;
    }

    // 舵机/夹爪动作计时
    if (catch_servo_busy && (int32_t)(HAL_GetTick() - catch_servo_deadline) >= 0) {
        catch_servo_busy = 0;
    }
}
