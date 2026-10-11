#include "headfile.h"

uint8_t hmi_rxbuffer[HMI_RX_SIZE];   // hmi 接收缓冲区
char hmi_txbuffer[HMI_TX_SIZE];      // hmi 发送缓冲区

static const uint8_t hmi_end_cmd[HMI_END_CMD_LEN] = {0xFF,0xFF,0xFF};  // 串口屏结束符序列

static UART_HandleTypeDef *hmi_uart = NULL;
static volatile uint8_t hmi_error_flag = 0;     // hmi 接收恢复标志位


// UART5 为常规模式DMA   每次收完或启动失败都要再次开启
static void hmi_receive_start(void) {
    uint32_t primask;
    if (hmi_uart == NULL || hmi_uart->hdmarx == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_UARTEx_ReceiveToIdle_DMA(hmi_uart, hmi_rxbuffer, sizeof(hmi_rxbuffer)) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(hmi_uart->hdmarx,DMA_IT_HT);  // 禁止半传输中断
        hmi_error_flag = 0;
    } else {
        hmi_rx_reset();     // 重启失败期间可能漏字节 之前的半条指令也放弃
        hmi_error_flag = 1; // 不等待  下一次 10ms 周期入口继续重试
    }
    __set_PRIMASK(primask);
}


// hmi 串口初始化
void hmi_init(UART_HandleTypeDef *huart) {
    hmi_uart = huart;
    hmi_error_flag = 0;
    hmi_rx_reset();
    if (hmi_uart == NULL || hmi_uart->hdmarx == NULL) return;

    hmi_receive_start();
    if (hmi_error_flag)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);   // 常亮 = 启动失败
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);     // 灭 = 启动成功
    }
}

// hmi 串口接收
void hmi_rx_callback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (hmi_uart == NULL || huart != hmi_uart || Size > HMI_RX_SIZE) {
        return;
    }

    // 在hmi_app内处理
    hmi_solve(huart, Size);

    // 重新开启 DMA 空闲中断接收
    hmi_receive_start();
}

// hmi 串口屏错误回调
void hmi_error_callback(UART_HandleTypeDef *huart) {
    if (hmi_uart == NULL || huart != hmi_uart) return;
    hmi_rx_reset(); // HAL接收错误中止DMA后 不能拼接错误前后的两半命令
    hmi_error_flag = 1;
}

void hmi_rx_task(void) {
    if (hmi_error_flag) hmi_receive_start();
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
    int cmd_len;

    // 显示命令只在主循环调用；先判断忙，避免覆盖上次DMA还在读取的缓冲
    if (hmi_uart == NULL || format == NULL || hmi_uart->gState != HAL_UART_STATE_READY)
        return;

    // 留出三个结束符的位置   过长指令直接放弃 不发送被截断的屏幕语法
    va_start(args,format);  // args:参数列表    format:最后一个具名参数
    cmd_len = vsnprintf(hmi_txbuffer, HMI_TX_SIZE - HMI_END_CMD_LEN + 1, format, args);
    va_end(args);
    if (cmd_len < 0 || cmd_len > HMI_TX_SIZE - HMI_END_CMD_LEN)
        return;

    memcpy(&hmi_txbuffer[cmd_len], hmi_end_cmd, HMI_END_CMD_LEN);
    HAL_UART_Transmit_DMA(hmi_uart,(uint8_t *)hmi_txbuffer, (uint16_t)cmd_len + HMI_END_CMD_LEN);
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
