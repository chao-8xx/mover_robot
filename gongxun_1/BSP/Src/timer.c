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
        servo_rotate_tick();    // 舵机转动计时/停止（中断里更稳，不受主循环阻塞影响）
        hwt101_tick();          // HWT101 非阻塞复位状态机
    }
    else if (htim ->Instance == TIM7) {    // 5ms中断

    }
    else if (htim ->Instance == TIM2) {    // 10ms中断

    }
}
