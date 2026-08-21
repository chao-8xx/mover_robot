#include "headfile.h"

void all_init(void)
{
    timer_init();
    uart_init();
}

// 定时器初始化
void timer_init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);    // 10ms中断
    HAL_TIM_Base_Start_IT(&htim6);    // 1ms中断
    HAL_TIM_Base_Start_IT(&htim7);    // 5ms中断
}

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim ->Instance == TIM6) {    // 1ms中断
        servo_rotate_tick();    // 舵机转动计时/停止（中断里更稳，不受主循环阻塞影响，用于360°连续舵机）
        hwt101_tick();          // HWT101 非阻塞复位状态机
        car_run_tick();         // 底盘移动非阻塞计时
        catch_tick();           // 夹取装置非阻塞计时
    }
    else if (htim ->Instance == TIM7) {    // 5ms中断
        yaw_rate = hwt101_get_yaw_rate();  // 偏航角角速度
        my_yaw = hwt101_get_yaw();         // 偏航角
        remind_test();
    }
    else if (htim ->Instance == TIM2) {    // 10ms中断
        // car_rotate_ctrl();  // 用于控制底盘非阻塞转向 放进10ms中断 不影响后续控制 不可删
    }
}
