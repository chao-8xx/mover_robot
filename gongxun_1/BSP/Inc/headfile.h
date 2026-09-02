//
// Created by mechreuo on 2026/5/31.
//

#ifndef GONGXUN_1_HEADFILE_H
#define GONGXUN_1_HEADFILE_H

#include "stm32f4xx_hal.h"                  // Device header

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>     /* 为了使用可变参数 va_list */
#include <stddef.h>
#include <math.h>

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"


// 基础外设、协议
#include "fun.h"
#include "timer.h"
#include "uart_dma.h"
#include "key_led.h"
#include "led_buzzer.h" // 拓展板板载LED和蜂鸣器
#include "camera.h"     // 摄像头
#include "lidar.h"      // 激光雷达
#include "Emm_V5.h"     // Emm42电机协议
#include "step_motor.h" // 闭环步进电机控制
#include "servo.h"      // 舵机
#include "ws2812.h"     // 8位RGB灯
#include "hwt101.h"     // HWT101陀螺仪
#include "hmi.h"        // 串口屏

// 应用、算法
#include "bfs.h"        // BFS广度优先搜索算法
#include "grid.h"       // 48x48占据栅格
#include "track_map.h"  // 5x5赛道拓扑
#include "planner.h"    // BFS与动态重规划
#include "tf2d.h"       // 2D坐标变换
#include "pid.h"        // PID控制算法
#include "chassis.h"    // 底盘控制
#include "catch.h"      // 抓取装置
#include "remind.h"     // 提示装置（ws2812、蜂鸣器、LED）
#include "hmi_app.h"    // hmi 应用层函数

// 测试
#include "basic_test.h" // 基础模块测试
#include "cam_test.h"   // 摄像头测试
#include "lidar_test.h" // 激光雷达测试
#include "run_test.h"   // 底盘测试
#include "bfs_test.h"   // BFS测试
#include "map_test.h"   // 地图测试

// 任务、比赛控制
#include "system.h"     // 车辆系统
#include "task.h"       // 任务设置

#endif //GONGXUN_1_HEADFILE_H
