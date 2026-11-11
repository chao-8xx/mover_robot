#include "headfile.h"

// 全部初始化
void all_init(void)
{
    timer_init();
    uart_init();
    light_buzzer_init();

    tf_init();
}

// 定时器初始化
void timer_init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);    // 10ms中断
    HAL_TIM_Base_Start_IT(&htim6);    // 1ms中断
    HAL_TIM_Base_Start_IT(&htim7);    // 5ms中断
}

// 定时器失能
void timer_disable(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);     // 10ms中断
    HAL_TIM_Base_Stop_IT(&htim6);     // 1ms中断
    HAL_TIM_Base_Stop_IT(&htim7);     // 5ms中断
}


// 定时器中断回调函数
// 调度：TIM6=发送与动作计时，TIM7=IMU/提示，TIM2=比赛状态机。
// 这些是裸机中断上下文，不是线程；新增阻塞发送/等待会影响周期，不能只按函数名判断是否非阻塞。
// PID 的 Ki/Kd 和稳定计数依赖调用节拍；改定时周期时同步检查 control/chassis/catch 中的计数与参数。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim ->Instance == TIM6) {    // 1ms中断
        hwt101_tick();           // HWT101 非阻塞复位状态机
        car_run_tick();          // 底盘移动非阻塞计时
        catch_tick();            // 夹取装置非阻塞计时 包含串口屏
    }
    else if (htim ->Instance == TIM7) {    // 5ms中断
        imu_task();
        imu_get();
        remind_control();
    }
    else if (htim ->Instance == TIM2) {    // 10ms中断
        system_control();
        debug_printf_task();    // DMA启动失败时重试已存在的数据   缓冲区仍由 debug_printf 模块统一管理
        // hmi_rx_task();          // 串口屏常规模式DMA接收失败后重试 不等待
    }
}
