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

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"



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

#include "../../APP/Inc/test.h"


#endif //GONGXUN_1_HEADFILE_H
