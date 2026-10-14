#include "headfile.h"

uint8_t hmi_rxbuffer[HMI_RX_SIZE];   // hmi 接收缓冲区
char hmi_txbuffer[HMI_TX_SIZE];   // hmi 发送缓冲区

uint8_t tx_len;     // 发送字节数

static uint8_t hmi_end_cmd[HMI_TX_SIZE] = {0xFF,0xFF,0xFF};  // 串口屏结束符序列

static UART_HandleTypeDef *hmi_uart = NULL;


// hmi 串口初始化
void hmi_init(UART_HandleTypeDef *huart) {
    hmi_uart = huart;

    if (HAL_UARTEx_ReceiveToIdle_DMA(hmi_uart, hmi_rxbuffer, sizeof(hmi_rxbuffer)) != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);   // 常亮 = 启动失败
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);     // 灭 = 启动成功
    }
    __HAL_DMA_DISABLE_IT(hmi_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
}

// hmi 串口接收
void hmi_rx_callback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart != hmi_uart) {
        return;
    }

    // 在hmi_app内处理
    hmi_solve(huart, Size);

    // 重新开启 DMA 空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(hmi_uart,hmi_rxbuffer,sizeof(hmi_rxbuffer));
    __HAL_DMA_DISABLE_IT(hmi_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
}

/**
 * @brief  向串口屏发送带格式化的指令 (核心协议封装)
 * @param  format: 格式化字符串 (类似 printf)
 * @param  ...   : 可变参数
 *
 * @note   【设计思想】为什么用可变参数？
 *         屏幕指令如 "t0.txt=\"220V\""。用 sprintf 先格式化，再追加 0xFF 0xFF 0xFF
 *         是最高效、最通用的方法。这一步封装好，上层调用就极其舒服。
 */
void hmi_SendCmd(const char *format, ...) {
    va_list args;       // 可变参数列表

    // 将上层传入的格式化字符串 解析到 hmi_txbuffer 中
    va_start(args,format);  // args:参数列表    format:最后一个具名参数
    vsnprintf(hmi_txbuffer, HMI_TX_SIZE, format, args); //format:格式串   args:参数列表
    va_end(args);

    // 获取需要发送的有效字符串长度
    tx_len = strlen(hmi_txbuffer);

    // 发送数据太多 则截断
    if (tx_len > HMI_TX_SIZE - HMI_END_CMD_LEN) {
        tx_len = HMI_TX_SIZE - HMI_END_CMD_LEN;
    }

    // 在发送的字符串后面添加三个 0xFF 作为结束符
    hmi_txbuffer[tx_len]     = hmi_end_cmd[0];
    hmi_txbuffer[tx_len + 1] = hmi_end_cmd[1];
    hmi_txbuffer[tx_len + 2] = hmi_end_cmd[2];

    // 发送封装好的字符串数据   总长度 = 字符长度 + 3个结束符
    // 上一次 DMA 没发完就不发，避免 BUSY 和缓冲被覆盖
    if (hmi_uart->gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_DMA(hmi_uart,(uint8_t *)hmi_txbuffer, tx_len + 3);
    }
}

/**
 * @brief  设置文本控件的内容
 * @param  objName: 控件名称，例如 "t0" (电压显示文本框)
 * @param  text: 要显示的文本内容，例如 "220.5"
 */
void hmi_SetText(const char *objName, const char *text) {

    // 修改文本格式：控件名.txt="text内容"
    hmi_SendCmd("%s.txt=\"%s\"", objName , text);
}

/**
 * @brief  设置数值控件的内容
 * @param  objName: 控件名称，例如 "n0"
 * @param  val:  要修改为的数值大小,整数值
 */
void hmi_SetValue(const char *objName, int val) {

    // 修改数值格式：控件名.val="val数值"
    hmi_SendCmd("%s.val=%d", objName , val);
}

/**
 * @brief  切换屏幕页面
 * @param  pageName: 页面名称，例如 "page1"
 */
void hmi_ChangePage(const char *pageName) {

    // 切页格式: page 页面名
    hmi_SendCmd("page %s", pageName);
}

/**
 * @brief  非阻塞状态机解析屏幕回传的触摸事件
 * @note   状态机逐字节匹配接收   当前在本函数中没有循环逐字节调用 需要在串口接收回调中循环调用
 * @note   通信协议: 0x65 + page_id + obj_id + event_type + 0xFF + 0xFF + 0xFF
 */
hmi_touch_t hmi_GetTouch(uint8_t byte) {
    static uint8_t state = 0;       // 状态机的状态
    static uint8_t rx_cache[4];     // 缓存: 页码、控件ID、事件类型
    static uint8_t end_count = 0;   // 统计连续收到几个 0xFF 结束符

    hmi_touch_t touch = {0, 0, 0, false};

    switch (state)
    {
        case 0:     // 等待帧头 0x65
            if (byte == 0x65)
                state = 1;
            break;

        case 1:     // 接收页码 page_id
            rx_cache[0] = byte;
            state = 2;
            break;

        case 2:     // 接收控件ID obj_id
            rx_cache[1] = byte;
            state = 3;
            break;

        case 3:     // 接收事件类型 event_type
            rx_cache[2] = byte;
            end_count = 0;
            state = 4;
            break;

        case 4:     // 等待3个连续的 0xFF 作为帧尾结束符
            if (byte == 0xFF) {
                end_count++;
                if (end_count >= HMI_END_CMD_LEN) { // 完整的一帧解析完成
                    touch.page_id      = rx_cache[0];
                    touch.obj_id       = rx_cache[1];
                    touch.event_type   = rx_cache[2];
                    touch.is_valid = true;

                    state = 0;
                    end_count = 0;

                    return touch;
                }
            }
            else {      // 帧尾错误 放弃这一数据帧
                state = 0;
                end_count = 0;
            }
            break;
        default:
            break;
    }
    return touch;       // 无完整事件
}

