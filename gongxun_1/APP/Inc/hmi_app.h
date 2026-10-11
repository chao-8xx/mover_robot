#ifndef GONGXUN_1_HMI_APP_H
#define GONGXUN_1_HMI_APP_H


// 串口屏初始化
void my_hmi_init(void);

// 触摸串口屏事件
void hmi_touch(void);

// 触摸串口屏发车事件
void hmi_start(void);

// 触摸串口屏停车事件
void hmi_stop(void);

// 触摸串口屏事件总处理
// 在hmi.c的hmi_rx_callback回调函数内调用
void hmi_solve(UART_HandleTypeDef *huart, uint16_t Size);
void hmi_rx_reset(void); // 串口错误后丢弃尚未拼完的屏幕指令

// 串口屏控制发送指令 放入1ms中断
void hmi_control_send_tick(void);

#endif //GONGXUN_1_HMI_APP_H

// 发车：0x65 0x01 0x04 0x00 0xFF 0xFF 0xFF
/* 急停：0x65 0x01 0x06 0x00 0xFF 0xFF 0xFF
 *      0x65 0x03 0x02 0x00 0xFF 0xFF 0xFF
 *      0x65 0x04 0x02 0x00 0xFF 0xFF 0xFF
*/
