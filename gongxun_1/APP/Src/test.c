#include "test.h"

// 虚拟串口 USB CDC 测试函数，验证 CDC 是否正常工作
// 现已变为无线串口 串口六输出了
void cdc_test(void) {
    // led_toggle();         // LED 闪烁表示程序在跑
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
            // led_off();
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

// 底盘测试代码 放入10ms中断
void chassis_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();
    // step_motor_init();
    // my_hwt101_init();
    // chassis_init();

    car_rotate_ctrl();

    static uint8_t step = 0;
    static int16_t chassis_speed = 50;

    switch (step)
    {
        case 0:
            car_run_distance(chassis_speed,40.0f,0.0f);    // 发出指令，底盘进入忙状态，向右平移40cm
            if (!car_is_busy()) {      // 空闲状态时，进入第1步
                step = 1;
            }
            break;
        case 1:
            car_run_distance(chassis_speed,-40.0f,0.0f);    // 发出指令，底盘进入忙状态，向左平移40cm
            if (!car_is_busy()) {      // 空闲状态时，进入第2步
                step = 2;
            }
            break;
        case 2:
            car_run_distance(chassis_speed,0.0f,40.0f);    // 发出指令，底盘进入忙状态，向前平移40cm
            if (!car_is_busy()) {      // 空闲状态时，进入第3步
                step = 3;
            }
            break;
        case 3:
            car_run_distance(chassis_speed,0.0f,-40.0f);    // 发出指令，底盘进入忙状态，向后平移40cm
            if (!car_is_busy()) {      // 空闲状态时，进入第4步
                step = 4;
            }
            break;
        case 4:
            car_rotate_angle(90.0f);   // 发出指令，底盘进入忙状态，向左旋转90°
            if (!car_is_busy()) {      // 空闲状态时，进入第5步
                step = 5;
            }
            break;
        case 5:
            car_rotate_angle(90.0f);   // 发出指令，底盘进入忙状态，向右旋转90°
            if (!car_is_busy()) {      // 空闲状态时，进入第6步
                step = 6;
            }
            break;
        case 6:
            car_stop();                // 发出指令，底盘进入空闲状态，停车
            break;
        default:
            car_stop();                // 发出指令，底盘进入空闲状态，停车
            step = 6;
            break;
    }
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
