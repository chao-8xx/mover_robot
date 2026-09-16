#include "headfile.h"

// 虚拟串口 USB CDC 测试函数，验证 CDC 是否正常工作
// 现已变为无线串口 串口六输出了
void cdc_test(void) {
    // led_toggle();         // LED 闪烁表示程序在跑
    // printf("Hello from CDC!\r\n");
    printf("hello world\r\n");
    HAL_Delay(1000);
}

// 陀螺仪测试函数
void hwt_test(void)
{
    static uint32_t last_print_ms = 0;
    uint32_t now_ms = HAL_GetTick();

    // 每 50 ms（20 Hz）输出一次；不要在 USART3 的 DMA/IDLE 回调中 printf
    if (now_ms - last_print_ms < 50) {
        return;
    }

    last_print_ms = now_ms;

    printf("valid=%u yaw=%.2f deg wz=%.2f deg/s\r\n",
           (unsigned int)hwt101_data_valid(),
           hwt101_get_yaw(),
           hwt101_get_yaw_rate());
}

// 提示装置测试代码 5ms定时中断
void remind_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();
    // remind_init();

    static uint16_t work_time = 600;
    static uint8_t step = 0;

    switch (step)
    {
        case 0:
            remindState = NO_REMIND;
            step = 1;
            break;
        case 1:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = RED_REMIND;
                step = 2;
            }
            break;
        case 2:
            if (remindState == NO_REMIND) {
                step = 3;
            }
            break;
        case 3:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = GREEN_REMIND;
                step = 4;
            }
            break;
        case 4:
            if (remindState == NO_REMIND) {
                step = 5;
            }
            break;
        case 5:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = BLUE_REMIND;
                step = 6;
            }
            break;
        case 6:
            if (remindState == NO_REMIND) {
                step = 7;
            }
            break;
        case 7:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = YELLOW_REMIND;
                step = 8;
            }
            break;
        case 8:
            if (remindState == NO_REMIND) {
                step = 9;
            }
            break;
        case 9:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = PINK_REMIND;
                step = 10;
            }
            break;
        case 10:
            if (remindState == NO_REMIND) {
                step = 11;
            }
            break;
        case 11:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = WHITE_REMIND;
                step = 12;
            }
            break;
        case 12:
            step = 1;
            break;
        default:
            break;
    }

    remind_control();
}
