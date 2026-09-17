#ifndef GONGXUN_1_HMI_H
#define GONGXUN_1_HMI_H

#define HMI_TX_SIZE  256
#define HMI_RX_SIZE  256

#define HMI_END_CMD_LEN     3       // 串口屏指令结束符 连续三个0xFF

extern uint8_t hmi_rxbuffer[HMI_RX_SIZE];   // hmi 接收缓冲区
extern char hmi_txbuffer[HMI_TX_SIZE];   // hmi 发送缓冲区

// 屏幕触摸事件结构体
typedef struct {
    uint8_t page_id;        // 触摸发生在哪一页
    uint8_t obj_id;         // 被点击的控件编号
    uint8_t event_type;     // 事件类型: 0x01=按下, 0x00=弹起
    bool    is_valid;       // 是否成功解析出一个有效事件
} hmi_touch_t;


void hmi_init(UART_HandleTypeDef *huart);
void hmi_rx_callback(UART_HandleTypeDef *huart, uint16_t Size);

void hmi_SendCmd(const char *format, ...);
void hmi_SetText(const char *objName, const char *text);
void hmi_SetValue(const char *objName, int val);
void hmi_ChangePage(const char *pageName);

hmi_touch_t hmi_GetTouch(uint8_t byte);

#endif //GONGXUN_1_HMI_H