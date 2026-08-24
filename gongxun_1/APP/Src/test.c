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

// 底盘绕场一周测试 10ms中断
void chassis_run_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();
    // step_motor_init();
    // my_hwt101_init();
    // chassis_init();
    // HAL_Delay(100);

    static uint16_t work_time = 250;
    static uint8_t step = 0;
    int16_t speed = 120;

    switch (step)
    {
        case 0:   // 开始前等待
            work_time--;
            if (work_time <= 0) {
                work_time = 250;
                step = 1;
            }
            break;

        case 1:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 2;
            }
            break;
        case 2:   // 旋转 -90°（必须持续调用）
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-90.0f);
                if (!car_is_busy()) {
                    step = 3;
                }
            }
            break;
        case 3:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 4;
            }
            break;
        case 4:   // 旋转 -180°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-180.0f);
                if (!car_is_busy()) {
                    step = 5;
                }
            }
            break;
        case 5:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 6;
            }
            break;
        case 6:   // 旋转 -270°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-270.0f);
                if (!car_is_busy()) {
                    step = 7;
                }
            }
            break;
        case 7:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 8;
            }
            break;
        case 8:   // 转回 0°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(0.0f);
                if (!car_is_busy()) {
                    step = 9;
                }
            }
            break;
        case 9:
            car_stop();
            break;
        default:
            car_stop();
            step = 9;
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

// bfs路径规划测试（无实物，仅串口打印路径）
// 放主函数while外
void bfs_test(void)
{
    uint8_t x, y;

    // 1. 初始化地图
    bfs_init();

    // 2. 设几个障碍（比如中间挡路）
    bfs_set_obstacle(0, 1, 1);
    bfs_set_obstacle(3, 3, 1);

    // 3. 规划路径
    printf("BFS: (0,0) -> (3,4)\r\n");
    printf("obstacle: (0,1) (3,3)\r\n");

    if (bfs_find_path(0, 0, 3, 4) == 1)
    {
        printf("found, len=%d\r\n", bfs_get_path_len());

        // 4. 把整条路径打出来
        for (uint8_t i = 0; i < bfs_get_path_len(); i++)
        {
            bfs_get_path_step(i, &x, &y);
            printf("step %d -> (%d,%d)\r\n", i, x, y);
        }
    }
    else
    {
        printf("no path\r\n");
    }
}
