#include "headfile.h"

// 电机当前速度
int16_t motor1_speed = 0;
int16_t motor2_speed = 0;
int16_t motor3_speed = 0;
int16_t motor4_speed = 0;

// 位置记录
static int32_t step_motor_pos[STEP_MOTOR_COUNT] = {0};

// 张大头 Emm_V5 驱动器地址：Motor_1 ~ Motor_6 对应 1 ~ 6
static const uint8_t step_motor_addr[STEP_MOTOR_COUNT] = {1, 2, 3, 4, 5, 6};

// 获取电机对应驱动器地址，非法编号返回 0
static uint8_t step_motor_get_addr(Motor_Num motor_num)
{
    if ((unsigned)motor_num >= STEP_MOTOR_COUNT) {
        return 0;
    }
    return step_motor_addr[motor_num];
}

// 同步当前速度到全局变量（前四个电机有对应变量）
static void step_motor_update_speed(Motor_Num motor_num, int16_t speed)
{
    switch (motor_num)
    {
        case Motor_1: motor1_speed = speed; break;
        case Motor_2: motor2_speed = speed; break;
        case Motor_3: motor3_speed = speed; break;
        case Motor_4: motor4_speed = speed; break;
        default: break;
    }
}

// 闭环步进电机初始化
// 使能 解除堵转保护
void step_motor_init()
{
    Emm_V5_Tx_Reset();  // 清空可能的残留发送队列
    Emm_V5_Rx_Reset();  // 清空上次运行的反馈和半帧
    HAL_Delay(1);

    for (Motor_Num motor_num = Motor_1; motor_num <= Motor_6; motor_num = (Motor_Num)(motor_num + 1)) {
        uint8_t addr = step_motor_get_addr(motor_num);

        Emm_V5_Reset_Clog_Pro(addr);              // 解除堵转保护
        HAL_Delay(10);
        Emm_V5_Modify_Ctrl_Mode(addr, false, 2);  // 闭环控制模式，不保存
        HAL_Delay(10);
        Emm_V5_En_Control(addr, true, false);     // 使能电机
        HAL_Delay(10);
        motor_set_speed(motor_num,0);    // 传电机编号  地址由函数内部换算
        HAL_Delay(10);
    }
    Emm_V5_Rx_Start();  // 电机串口接收DMA开启
}

// 电机编码器清零
void motor_encoder_clear(Motor_Num motor_num)
{
    uint8_t addr = step_motor_get_addr(motor_num);
    if (addr != 0) {
        Emm_V5_Clear_Encoder(addr);
    }
}

// 电机定速控制（非阻塞，可在主循环或定时器中断中调用,正：逆时针，负：顺时针）
void motor_set_speed(Motor_Num motor_num,int16_t speed)
{
    uint8_t addr = step_motor_get_addr(motor_num);
    uint8_t dir;
    uint16_t vel;

    if (addr == 0) {
        return;
    }

    dir = (speed < 0) ? 1 : 0;           // 0 为 CW，1 为 CCW
    vel = (speed < 0) ? (uint16_t)(-(int32_t)speed) : (uint16_t)speed;
    if (vel > MAX_SPEED) {
        vel = MAX_SPEED;
    }

    // Emm_V5_Vel_Control(addr, dir, vel, STEP_MOTOR_DEFAULT_ACC, false);
    Emm_V5_Vel_Control(addr, dir, vel, 0, false);
    step_motor_update_speed(motor_num, speed);
}

// 电机定速定位置控制 相对（非阻塞，可在主循环或定时器中断中调用）
void motor_set_speedPos(Motor_Num motor_num,int16_t speed,int32_t pulse)
{
    uint8_t addr = step_motor_get_addr(motor_num);
    uint8_t dir;
    uint16_t vel;
    uint32_t clk;

    if (addr == 0) {
        return;
    }

    vel = (speed < 0) ? (uint16_t)(-(int32_t)speed) : (uint16_t)speed;
    if (vel > 5000) {
        vel = 5000;
    }

    // pulse 正负决定方向，取绝对值作为相对脉冲数
    dir = (pulse < 0) ? 1 : 0;
    clk = (pulse < 0) ? (uint32_t)(-(int32_t)pulse) : (uint32_t)pulse;

    if (motor_num != Motor_5 && motor_num != Motor_6) {
        if (Emm_V5_Pos_Control(addr, dir, vel, STEP_MOTOR_CHASSIS_ACC, clk, false, false)) {
            step_motor_pos[motor_num] += pulse;    // 只有入队成功才更新账本
        }
    }
    else {
        if (Emm_V5_Pos_Control(addr, dir, vel, STEP_MOTOR_CATCH_ACC, clk, false, false)) {
            step_motor_pos[motor_num] += pulse;    // 只有入队成功才更新账本
        }
    }
    step_motor_update_speed(motor_num, speed);
}

// 电机定速定位置控制 绝对（非阻塞，可在主循环或定时器中断中调用）
void motor_set_speed_Pos(Motor_Num motor_num,int16_t speed,int32_t pos)
{
    uint8_t addr = step_motor_get_addr(motor_num);
    uint8_t dir;
    uint16_t vel;
    uint32_t clk;

    if (addr == 0) {
        return;
    }

    vel = (speed < 0) ? (uint16_t)(-(int32_t)speed) : (uint16_t)speed;
    if (vel > 5000) {
        vel = 5000;
    }

    // dir 是方向标志，由目标位置符号决定（绝对模式下 dir 即目标符号）
    if (motor_num == Motor_6) {
        dir = (pos < 0) ? 1 : 0;   // 伸缩：目标带符号
        dir = !dir;                // 伸缩电机方向翻转（如果机械装反了）
    } else {
        dir = (pos < 0) ? 1 : 0;   // 升降/底盘：目标带符号（升降目标恒正，dir 恒 0）
    }
    clk = (pos < 0) ? (uint32_t)(-(int32_t)pos) : (uint32_t)pos;
    
    if (Emm_V5_Pos_Control(addr, dir, vel, STEP_MOTOR_CATCH_ACC, clk, true, false)) {
        step_motor_pos[motor_num] = pos;   // 只有命令成功入队才更新账本
    }

    step_motor_update_speed(motor_num, speed);
}

void motor_pos_set(Motor_Num motor_num, int32_t pos)
{
    if (step_motor_get_addr(motor_num) == 0)
        return;
    step_motor_pos[motor_num] = pos;
}

// 电机定速定位置控制 多机协同
void motor_set_speedPos_sync(Motor_Num motor_num, int16_t speed, int32_t pulse)
{
    uint8_t addr = step_motor_get_addr(motor_num);
    uint8_t dir;
    uint16_t vel;
    uint32_t clk;

    if (addr == 0) {
        return;
    }

    vel = (speed < 0) ? (uint16_t)(-(int32_t)speed) : (uint16_t)speed;
    if (vel > 5000) {
        vel = 5000;
    }

    dir = (pulse < 0) ? 1 : 0;
    clk = (pulse < 0) ? (uint32_t)(-(int32_t)pulse) : (uint32_t)pulse;

    // snF = true：装填目标，不立即启动
    if (Emm_V5_Pos_Control(addr, dir, vel, STEP_MOTOR_CHASSIS_ACC, clk, false, true)) {
        step_motor_pos[motor_num] += pulse;
    }
    step_motor_update_speed(motor_num, speed);
}
