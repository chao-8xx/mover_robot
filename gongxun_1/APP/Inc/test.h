//
// Created by mechreuo on 2026/6/4.
//

#ifndef GONGXUN_1_TEST_H
#define GONGXUN_1_TEST_H

#include "headfile.h"

void cdc_test(void);
void openmv_qr_test(void);
void maix_blob_test(void);
void lidar_test(void);
void hwt_test(void);

// 底盘测试代码 放入10ms中断
void chassis_test(void);
// 提示装置测试代码 5ms定时中断
void remind_test(void);

#endif //GONGXUN_1_TEST_H
