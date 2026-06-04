//
// Created by mechreuo on 2026/6/3.
//

#include "headfile.h"

void all_init(void)
{
    led_init();
    timer_init();
    // usart_init();
    uart_init();
     // OLED_Init();
}

// 定时器初始化
void timer_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);    // 1ms中断
    HAL_TIM_Base_Start_IT(&htim7);    // 5ms中断
}

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim ->Instance == TIM6){    // 1ms中断

    }
    else if (htim ->Instance == TIM7){    // 5ms中断
        key_tick();
        // usart_handle();

    }
}
