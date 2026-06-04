//
// Created by mechreuo on 2026/6/4.
//

#include "headfile.h"

// 虚拟串口 USB CDC 测试函数，验证 CDC 是否正常工作
void cdc_test(void) {
    led_toggle();         // LED 闪烁表示程序在跑
    printf("Hello from CDC!\r\n");
    HAL_Delay(1000);
}

// openmv扫码测试 key1 按下后开始扫码，扫码结果通过虚拟串口CDC输出
void openmv_qr_test(void) {
    // key
    // void key_handle(void)
    // {
    //   uint8_t shortpress = key_getshortpress ();
    //   uint8_t longpress = key_getlongpress ();
    //
    //   if (shortpress != 0 )
    //   {
    //     switch(shortpress )
    //     {
    //       case 1:
    //         led_off();
    //         openmv_send_start_scan();
    //         break ;
    //     }
    //   }
    //   if (longpress != 0 )
    //   {
    //     switch(longpress )
    //     {
    //       // case 1:
    //       //  led_tick ();
    //     }
    //   }
    // }

    //  主函数
    //   while (1)
    //   {
    //     key_handle();
    //     openmv_qr_task();
    //   }
}

// LiDAR 测试函数，按键1开始扫描，按键1长按停止扫描，扫描结果通过虚拟串口CDC输出
void lidar_test(void) {
    printf("================================\r\n");
    // 遍历输出 0-359 度的雷达数据
    for(int i = 0; i < 360; i++)
    {
        // 格式对齐：@后面跟3位角度，5位距离
        printf("@%3d %5d \r\n", i, lidar_data[i]);
    }
    printf("================================\r\n");
}