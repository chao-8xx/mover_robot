//
// Created by mechreuo on 2026/6/3.
//

#include "headfile.h"

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim ->Instance == TIM6){    // 1ms中断

        void usart_handle(void );
        key_tick();
    }
    else if (htim ->Instance == TIM7){    // 5ms中断

        void usart_handle(void );
        key_tick();
    }
}