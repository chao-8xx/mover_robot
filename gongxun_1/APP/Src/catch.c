#include "headfile.h"

// 电机参数
#define PULSE_PER_CIRCLE    3200      // 一圈的脉冲数

// 丝杆控制相关参数
#define S_MM_PER_CIRCLE     2         // 一圈伸缩的距离mm
#define MM_PER_CIRCLE       8         // 一圈升降的距离mm
#define LIFT_CMD_PULSE_PER_CM (PULSE_PER_CIRCLE / (MM_PER_CIRCLE / 10.0f))  // 绝对命令：4000 脉冲/cm
#define LIFT_FEEDBACK_PER_CM  320.0f  // S_CPOS 反馈实测：约 320 单位/cm
#define LIFT_TOL_CM           0.5f   // 升降到位容差 cm

// 齿轮齿条控制相关参数
#define GEAR_DIAMETER           31.1f  // 齿轮直径mm，卡尺实测
#define STRETCH_FEEDBACK_PER_CM 26.2f  // S_CPOS 原始读数：伸出 1cm 对应多少单位
#define STRETCH_CMD_PULSE_PER_CM \
    (PULSE_PER_CIRCLE / (3.14159265f * GEAR_DIAMETER / 10.0f))  // 绝对位置命令的脉冲/cm
#define STRETCH_TOL_CM         0.1f  // 伸缩绝对定位容差 cm
#define STRETCH_READ_PERIOD_MS 20    // 绝对定位读反馈周期
#define STRETCH_TIMEOUT_MS     3000  // 绝对定位单次超时
#define STRETCH_RETRY_MAX      3     // 最多重发次数

// 电机速度
#define STEP_MOVE_SPEED     800       // 丝杆电机位置模式移动速度（原1650，降速防过冲，保证升降到位稳定）
#define STEP_SPEED          75        // 齿轮齿条电机位置模式移动速度

// 伸缩齿轮齿条速度模式
uint8_t stretch_speed_on = 0;         // 启动速度模式
int16_t stretch_speed = 0;            // 齿轮齿条运动速度
int8_t  stretch_state = 0;            // 用于替代线程锁 防止伸缩停止功能失灵

// 升降非阻塞计时：预计完成时刻 + busy 标志
static volatile uint8_t lift_busy = 0;          // 升降装置
static volatile uint32_t lift_deadline = 0;
static volatile uint8_t lift_use_deadline = 0;  // 相对模式按时间结束，不做编码器到位判断
static volatile uint8_t stretch_busy = 0;       // 伸缩装置
static volatile uint32_t stretch_deadline = 0;
static volatile uint8_t servo_busy = 0;         // 夹取装置
static volatile uint32_t servo_deadline = 0;

// 升降装置编码器判断是否到位
static volatile int32_t  lift_target_pos  = 0;   // 本次目标（脉冲）
static volatile uint32_t lift_read_tick   = 0;   // 上次发读请求的时刻
static volatile uint32_t lift_start_tick  = 0;   // 本次动作开始时刻
static volatile uint32_t lift_retry_times  = 0;  // 当前重发次数
static int32_t  lift_zero_pos      = 0;          // 升降零点（清零后实际编码器值），动态抓取
static volatile int32_t  lift_raw_pos   = 0;     // 1ms 中断读取 Motor_5 位置时保存的原始编码器值，供初始化抓零点
static volatile uint8_t lift_raw_ready  = 0;     // 有新原始位置可用时置 1，初始化读取后清零
#define LIFT_POS_TOL          1000               // 到位容差（脉冲），1000/4000=0.25cm
#define LIFT_READ_PERIOD_MS   50                 // 读位置周期
#define LIFT_TIMEOUT_MS       3000               // 超时保护（防卡死）
#define LIFT_RETRY_MAX        3                  // 最多重发几次

// 伸缩装置判断
volatile uint8_t stretch_pos_valid = 0;          // 是否读到过有效位置
static volatile int32_t stretch_raw_pos = 0;     // 1ms 中断读取 Motor_6 位置时保存的原始编码器值，供初始化阶段取作零点
static volatile uint8_t stretch_raw_ready = 0;   // 有新原始位置可用时置 1，初始化读取后由 catch_init 清零
static int32_t stretch_zero_pos = 0;             // 完全缩回时的原始编码器值
static uint8_t stretch_home_ok = 0;              // 是否已完成零点标定
static int32_t stretch_cmd_zero_pos = 0;         // 绝对位置命令的零点
static int32_t stretch_target_pos = 0;           // 本次绝对定位目标
static uint8_t stretch_abs_mode = 0;             // 绝对定位进行中
static volatile uint32_t stretch_read_tick = 0;  // 上次发伸缩读请求时刻
static volatile uint32_t stretch_start_tick = 0; // 本次绝对定位开始时刻
static volatile uint8_t stretch_retry_times = 0; // 当前重发次数

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
#define FOLLOW_KD       0.5f
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
    lift_busy = 0;
    lift_deadline = 0;
    motor_pos_set(Motor_5, 0);
    HAL_Delay(20);
    get_lift_distance();
    HAL_Delay(20);

    // 升降动态抓零点（同伸缩），消除上电不在 15cm 或清零帧残留导致的零点偏移
    lift_raw_ready = 0;
    get_lift_distance();
    uint32_t lift_home_start = HAL_GetTick();
    while ((HAL_GetTick() - lift_home_start) < 100) {
        if (lift_raw_ready) {
            lift_zero_pos = lift_raw_pos;
            break;
        }
    }
    motor_pos_set(Motor_5, lift_zero_pos);

    // 夹爪初始高度
    task_state.cur_lift_height = LIFT_ZERO_CM;
    task_state.cur_height = LIFT_ZERO_CM;
    // 夹爪初始伸出距离0.0cm
    task_state.cur_stretch_dist = STRETCH_NORMAL;
    task_state.cur_stretch = STRETCH_NORMAL;

    stretch_pos_valid = 0;

    // 处理伸缩电机
    motor_encoder_clear(Motor_6);   // 清零6号电机编码器 齿轮齿条
    HAL_Delay(20);
    motor_pos_set(Motor_6, stretch_cmd_zero_pos);  // 账本同步归零
    HAL_Delay(20);
    get_stretch_distance();
    HAL_Delay(20);

    // 位置回复会由 1ms catch_tick 取走，这里只等待它记录到的原始值
    stretch_raw_ready = 0;
    get_stretch_distance();

    uint32_t stretch_home_start = HAL_GetTick();
    while ((HAL_GetTick() - stretch_home_start) < 100) {
        if (stretch_raw_ready) {
            stretch_zero_pos = stretch_raw_pos;
            stretch_home_ok = 1;
            break;
        }
    }
    motor_pos_set(Motor_6, stretch_zero_pos);

    // 舵机初始化
    servo_init();

    // 舵机复位
    // holder_rotate(HOLDER_CATCH);     // 云台舵机
    holder_rotate(HOLDER_SAFE);     // 云台舵机
    turntable_rotate(ONE_LOCATION); // 转盘舵机
    claw_ctrl(CLAW_LOOSE);          // 夹爪舵机
    HAL_Delay(500);

    // 伸缩到合规位置
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
    stretch_abs_mode = 0;   // 速度停止时取消绝对到位判断
    stretch_state = 3;      // 直接请求停止
    motor_set_speed(Motor_6, 0);
}
// 抓取装置夹爪伸缩状态赋值
void catch_stretch_send(int16_t speed)
{
    stretch_speed_on = 1;   // 启动伸缩装置速度模式
    stretch_speed = speed;  // 设置速度
    stretch_abs_mode = 0;   // 追踪模式不参与绝对到位判断
}
// 伸出距离(cm) → 编码器绝对位置(脉冲)
// 齿轮齿条：一圈走的距离 = π × 齿轮直径
static int32_t stretch_dist_to_pos(float dist_cm)
{
    float p = (float)stretch_cmd_zero_pos
            + dist_cm * STRETCH_CMD_PULSE_PER_CM; // 伸出为负
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
    stretch_target_pos = pos;
    stretch_abs_mode = 1;
    stretch_start_tick = HAL_GetTick();
    stretch_read_tick = stretch_start_tick;
    stretch_retry_times = 0;

    // 距离 → 绝对位置命令脉冲
    float f_pulses = distance * STRETCH_CMD_PULSE_PER_CM;
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
    *dist_cm = (float)(pos - stretch_zero_pos) / STRETCH_FEEDBACK_PER_CM;
    return 1;
}
// 获取伸缩电机编码器 发送指令
void get_stretch_distance(void)
{
    Emm_V5_Read_Sys_Params(6, S_CPOS);
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

    stretch_abs_mode = 0;   // 丝杆相对模式，不使用齿轮齿条绝对定位判断

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
    lift_use_deadline = 1;
    lift_busy = 1;
    task_state.cur_lift_height += delta_cm;
}
// 抓取装置夹爪升降至固定高度 单位:cm 绝对模式
void catch_lift_to_height(float height)
{
    // 超限和忙保护（同伸缩 catch_stretch_distance）
    if (lift_busy || height < -2.0f || height > 15.0f) {
        return;
    }

    // 绝对模式
    int32_t pos;
    pos  = lift_height_to_pos(height);

    // 绝对位置命令：覆盖重发同目标也是安全的
    motor_set_speed_Pos(Motor_5, STEP_MOVE_SPEED, pos);

    // 记录本次目标，启动"持续读位置"的到位判断
    lift_target_pos = pos;
    lift_start_tick = HAL_GetTick();
    lift_read_tick  = lift_start_tick;
    lift_retry_times = 0;          // 每次新动作重置重试次数
    lift_use_deadline = 0;
    lift_busy = 1;

    task_state.cur_lift_height = height;
    my_control.lift_state = 1;
}
// 获取升降电机编码器 发送指令
void get_lift_distance(void)
{
    Emm_V5_Read_Sys_Params(5, S_CPOS);
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
            if (send_time == 4) {
                stretch_state = 1;  // 1表示准备发送指令
                catch_stretch(stretch_speed);
                stretch_state = 2;  // 2表示发送完指令
            }
            else if (send_time == 8) {  // 隔 4ms 再发读位置请求，避免两帧粘连
                stretch_state = 1;  // 1表示准备发送指令
                if (Emm_V5_Tx_Count() < 2) {           // 队列空闲才发
                    Emm_V5_Read_Sys_Params(6, S_CPOS);
                }
                stretch_state = 2;  // 2表示发送完指令
            }
            else if (send_time >= 10) {
                send_time = 0;                        // 一轮结束
                stretch_state = 0;                    // 回到等待状态
            }
        }
    }
    else {
        send_time = 0;
        stretch_state = 0;
    }
    // 读伸缩编码器 将脉冲数转化为实际距离cm
    {
        int32_t pos;
        if (Emm_V5_Get_Pos(6, &pos)) {
            stretch_raw_pos = pos;
            stretch_raw_ready = 1;
            float dist = (float)(pos - stretch_zero_pos)
                       / STRETCH_FEEDBACK_PER_CM; // 相对零点
            task_state.cur_stretch = dist;        // 齿轮齿条实时位置
            task_state.cur_stretch_dist = dist;   // 丝杆字段也同步（绝对定位在用）
            stretch_pos_valid = stretch_home_ok;  // 零点标定完成才启用保护

            // 绝对定位时用反馈距离判断是否到位
            if (stretch_abs_mode && stretch_busy) {
                float target_cm = (float)(stretch_target_pos - stretch_cmd_zero_pos)
                                / STRETCH_CMD_PULSE_PER_CM;

                if (my_absf(task_state.cur_stretch - target_cm) <= STRETCH_TOL_CM) {
                    stretch_abs_mode = 0;
                    stretch_busy = 0;
                    stretch_retry_times = 0;
                }
            }
        }
    }
    // 读升降编码器 将脉冲数转化为实际高度cm
    {
        int32_t pos;
        if (Emm_V5_Get_Pos(5, &pos)) {
            lift_raw_pos = pos;                    // 供初始化阶段抓零点
            lift_raw_ready = 1;
            float height = LIFT_ZERO_CM - (float)(pos - lift_zero_pos) / LIFT_FEEDBACK_PER_CM;
            task_state.cur_height = height;        // 齿轮齿条实时位置
            task_state.cur_lift_height = height;   // 丝杆字段也同步（绝对定位在用）

            // 如果升降正在执行，判断是否到位
            if (lift_busy) {
                float target_height = LIFT_ZERO_CM
                                    - (float)lift_target_pos / LIFT_CMD_PULSE_PER_CM;
                float err = my_absf(height - target_height);
                if (err <= LIFT_TOL_CM) {
                    lift_retry_times = 0;
                    lift_busy = 0;      // 真到位
                }
            }
        }
    }

    // 升降判断 持续读取编码器数据判断是否到达
    if (lift_busy) {
        if (lift_use_deadline) {
            // 串口屏相对升降：到达估算时间即结束，避免误触发绝对位置判断
            if ((int32_t)(HAL_GetTick() - lift_deadline) >= 0) {
                lift_busy = 0;
            }
        }
        else {
            // 绝对模式：每 20ms 发一次读位置请求
            if ((int32_t)(HAL_GetTick() - lift_read_tick) >= LIFT_READ_PERIOD_MS) {
                lift_read_tick = HAL_GetTick();
                if (Emm_V5_Tx_Count() < 2) {
                    Emm_V5_Read_Sys_Params(5, S_CPOS);
                }
            }
            // 超时保护：规定时间没到位，说明驱动器丢失指令，再次发送
            if ((int32_t)(HAL_GetTick() - lift_start_tick) >= LIFT_TIMEOUT_MS && lift_retry_times < LIFT_RETRY_MAX) {
                motor_set_speed_Pos(Motor_5, STEP_MOVE_SPEED, lift_target_pos);
                lift_start_tick = HAL_GetTick();
                lift_read_tick  = lift_start_tick;
                lift_busy = 1;
                my_control.lift_state = 1;
                lift_retry_times += 1;
            }
            else if ((int32_t)(HAL_GetTick() - lift_start_tick) >= LIFT_TIMEOUT_MS && lift_retry_times >= LIFT_RETRY_MAX) {
                lift_retry_times = 0;
                lift_busy = 0;  // 放行
            }
        }
    }

    // 齿轮齿条绝对定位：按反馈到位，超时重发
    if (stretch_abs_mode && stretch_busy) {
        if ((int32_t)(HAL_GetTick() - stretch_read_tick) >= STRETCH_READ_PERIOD_MS) {
            stretch_read_tick = HAL_GetTick();
            if (Emm_V5_Tx_Count() < 2) {
                Emm_V5_Read_Sys_Params(6, S_CPOS);
            }
        }

        if ((int32_t)(HAL_GetTick() - stretch_start_tick) >= STRETCH_TIMEOUT_MS) {
            if (stretch_retry_times < STRETCH_RETRY_MAX) {
                motor_set_speed_Pos(Motor_6, STEP_SPEED, stretch_target_pos);
                stretch_start_tick = HAL_GetTick();
                stretch_read_tick = stretch_start_tick;
                stretch_retry_times++;
            }
            else {
                stretch_retry_times = 0;
                stretch_abs_mode = 0;
                stretch_busy = 0;
            }
        }
    }

    // 丝杆相对伸缩仍按原时间等待结束
    if (stretch_busy && !stretch_abs_mode &&
        (int32_t)(HAL_GetTick() - stretch_deadline) >= 0) {
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
