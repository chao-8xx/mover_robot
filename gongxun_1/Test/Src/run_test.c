//
// Created by mechreuo on 2026/9/2.
//

#include "headfile.h"


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
