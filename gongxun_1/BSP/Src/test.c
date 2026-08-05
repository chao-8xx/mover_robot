//
// Created by mechreuo on 2026/6/4.
//

#include "headfile.h"

// 虚拟串口 USB CDC 测试函数，验证 CDC 是否正常工作
// 现已变为无线串口 串口六输出了
void cdc_test(void) {
    led_toggle();         // LED 闪烁表示程序在跑
    // printf("Hello from CDC!\r\n");
    printf("hello world\r\n");
    HAL_Delay(1000);
}

// openmv扫码测试 key1 按下后开始扫码，扫码结果通过无线串口输出
void openmv_qr_test(void)
{
    openmv_send_start_scan();
    printf("OpenMV QR test start\r\n");

    while (1)
    {
        openmv_qr_task();

        if (key_getlongpress() == 1)
        {
            openmv_send_stop_scan();
            led_off();
            printf("OpenMV QR test stop\r\n");
            break;
        }

        HAL_Delay(10);
    }
}

/*
// 之前的openmv测试代码
void openmv_qr_test_legacy(void) {
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
*/

// maixcam测试函数，按键1开始识别物料，按键1长按停止识别，识别结果通过虚拟串口CDC输出
void maix_blob_test(void)
{
    uint8_t started = 0;
    maix_vision_t vision;

    while (1)
    {
        if (!started)
        {
            led_on();
            maix_start_blob(MAIX_COLOR_GREEN);
            printf("Maix blob test start\r\n");
            started = 1;
        }

        if (maix_get_result(&vision))
        {
            printf("Maix result: cmd=%02X target=%02X ok=%d dx=%d dy=%d tick=%lu\r\n",
                   vision.cmd, vision.target, vision.ok,
                   vision.dx, vision.dy, vision.tick);
        }

        if (key_getlongpress() == 1)
        {
            led_off();
            maix_stop();
            printf("Maix blob test stop\r\n");
            break;
        }

        HAL_Delay(10);
    }
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
