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
#include "timer.h"
#include "uart_dma.h"
#include "key_led.h"
#include "led_buzzer.h"  // 拓展板板载LED和蜂鸣器
#include "camera.h"      // 摄像头
#include "lidar.h"       // 激光雷达
#include "Emm_V5.h"      // Emm42电机协议
#include "step_motor.h"  // 闭环步进电机控制
#include "servo.h"       // 舵机
#include "ws2812.h"      // 8位RGB灯
#include "hwt101.h"      // HWT101陀螺仪
#include "hmi.h"         // 串口屏
#include "debug_printf.h"// 串口打印调试

// 应用、算法
#include "bfs.h"        // BFS广度优先搜索算法
// #include "grid.h"       // 旧栅格接口  本版道路地图不使用
#include "map_track.h"  // 5x5赛道静态拓扑
#include "map_data.h"   // 道路状态和已确认障碍物
#include "map_update.h" // 跨圈障碍确认与道路更新
#include "map_plan.h"   // 9节点12道路路径搜索
#include "map_lidar.h"  // 雷达聚类到地图
#include "map_pose.h"   // 定位模块发布的实际地图位姿
#include "map_nav.h"    // 道路分段导航与实际到位确认
#include "map_tf2d.h"   // 2D坐标变换
#include "map_obs.h"    // 雷达障碍物扫描与道路阻塞
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
#include "map_stage_test.h" // 新地图各阶段手动测试

// 任务、比赛控制
#include "control.h"    // 控制函数
#include "system.h"     // 车辆系统
#include "task.h"       // 任务设置

#endif //GONGXUN_1_HEADFILE_H
