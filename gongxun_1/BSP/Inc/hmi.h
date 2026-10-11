#ifndef GONGXUN_1_HMI_H
#define GONGXUN_1_HMI_H

#define HMI_TX_SIZE  256
#define HMI_RX_SIZE  256

#define HMI_END_CMD_LEN     3       // 串口屏指令结束符 连续三个0xFF

extern uint8_t hmi_rxbuffer[HMI_RX_SIZE];   // hmi 接收缓冲区
extern char hmi_txbuffer[HMI_TX_SIZE];   // hmi 发送缓冲区

void hmi_init(UART_HandleTypeDef *huart);
void hmi_rx_callback(UART_HandleTypeDef *huart, uint16_t Size);
void hmi_error_callback(UART_HandleTypeDef *huart);
void hmi_rx_task(void); // 非阻塞重试正常模式DMA接收

void hmi_SendCmd(const char *format, ...); // 显示发送只在主循环调用
void hmi_SetText(const char *objName, const char *text);
void hmi_SetValue(const char *objName, int val);
void hmi_ChangePage(const char *pageName);

#endif //GONGXUN_1_HMI_H
