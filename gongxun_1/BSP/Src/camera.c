//
// Created by mechreuo on 2026/6/3.
//

#include "headfile.h"

static UART_HandleTypeDef *cam_uart = NULL;

// 通信协议变量
static cam_state_t cam_state = cam_wait_aa; // 状态机状态
static uint8_t cam_cmd = 0;             // 当前帧的命令
static uint8_t cam_len = 0;             // 当前帧的长度
static uint8_t cam_idx = 0;             // 当前帧的数据下标
static uint8_t cam_sum = 0;             // 当前帧的校验和

static uint8_t cam_data[CAM_DATA_LEN];  // 协议层接收缓冲区


// UART+DMA 底层变量
uint8_t  cam_rx_buf[CAM_LEN];        // vision 接收缓冲区
uint16_t cam_rd = 0;                 // 读指针
uint16_t cam_wr = 0;                 // 写指针
volatile uint8_t cam_rx_flag = 0;    // 接收完成标志位
volatile uint8_t cam_error_flag = 0; // 错误回调标志位

uint8_t qr_tx_frame[6];          // OpenMV控制命令发送缓冲区
uint8_t qr_data[QR_DATA_LEN + 1];// 接收到的二维码数据
uint8_t qr_ok = 0;               // 接收到二维码数据的标志位
uint32_t qr_tick = 0;             // F407收到二维码结果的时间



// Maix命令发送和最新单帧原始结果
uint8_t maix_tx_buf[6];                      // maix 发送帧缓冲区
uint8_t maix_now_cmd = MAIX_CMD_NONE;        // maix cmd类枚举
uint8_t maix_now_target = MAIX_COLOR_NONE;   // maix target类枚举

maix_vision_t maix_vision = {0};
maix_result_t maix_result = {0};

maix_filter_t maix_filter[MAIX_WINDOW_SIZE]; // maix 滤波窗口
uint8_t maix_filter_count = 0;               // maix 滤波窗口计数
uint8_t maix_filter_write = 0;               // maix 滤波窗口写指针
uint8_t maix_last_frame_id = 0;              // maix 上一次帧ID
uint8_t maix_has_last_frame = 0;             // maix 是否有上一帧数据

///////////////////////////////////// 通信协议解析 ////////////////////////////////////////
// 读取小端16位整数
int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// 取中位数  插入排序，最多只处理5个数
int16_t get_median_i16(const int16_t *values, uint8_t count)
{
    int16_t sorted[MAIX_WINDOW_SIZE];
    uint8_t i;

    // 计算的数字个数在0与5之间  一般是3
    if (count == 0 || count > MAIX_WINDOW_SIZE)
        return 0;

    // 插入排序
    for (i = 0; i < count; i++)
    {
        uint8_t pos = i;
        sorted[i] = values[i];
        while (pos > 0 && sorted[pos - 1] > sorted[pos])
        {
            int16_t tmp = sorted[pos - 1];
            sorted[pos - 1] = sorted[pos];
            sorted[pos] = tmp;
            pos--;
        }
    }

    return sorted[count / 2];
}

// 计算校验和
uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p)
{
    uint16_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++)
        sum += p[i];
    return (uint8_t)sum;
}

// cam 解析字节  不定长(变长数据帧)
void cam_byte_handle(uint8_t byte)
{
    switch (cam_state) {
        case cam_wait_aa:       // 帧头1 0xAA
            if (byte == HEAD1) cam_state = cam_wait_55;
            break;

        case cam_wait_55:       // 帧头2 0x55
            if (byte == HEAD2) cam_state = cam_wait_cmd;
            else if (byte == HEAD1) cam_state = cam_wait_55;    // 连续两个 0xAA，说明前一个是噪声，继续等 0x55
            else cam_state = cam_wait_aa;       // 不是帧头，回到等待帧头1
            break;

        case cam_wait_cmd:      // 命令 cmd
            cam_cmd = byte;
            cam_state = cam_wait_len;
            break;

        case cam_wait_len:      // 长度 len
            cam_len = byte;
            cam_idx = 0;

            if (cam_len > CAM_DATA_LEN) {
                cam_state = cam_wait_aa;   // 长度不合法，重头开始
            } else if (cam_len == 0) {
                cam_state = cam_wait_sum;   // 没有数据，直接等校验
            } else {
                cam_state = cam_wait_data;   // 开始收数据
            }
            break;

        case cam_wait_data:     // 数据 cam_data
            cam_data[cam_idx++] = byte;

            if (cam_idx >= cam_len)
                cam_state = cam_wait_sum;   // 收完数据，等校验
            break;

        case cam_wait_sum:      // 校验和 sum
            cam_sum = byte;
            if (cam_sum == check_sum(cam_cmd, cam_len, cam_data)) {
                // IDLE只负责通知；真正的完整帧在这里按CMD分发
                cam_frame_handle(cam_cmd, cam_data, cam_len);
            }

            // 无论校验是否正确，都重新寻找下一帧头
            cam_state = cam_wait_aa;
            cam_cmd = 0;
            cam_len = 0;
            cam_idx = 0;
            cam_sum = 0;

            break;

        default:
            cam_state = cam_wait_aa;
            break;
    }
}

// 摄像头串口接收处理
void cam_rx_handle(void) {
    uint16_t wr;

    if (cam_uart == NULL || cam_uart->hdmarx == NULL)
        return;

    // 串口/DMA 错误回调函数
    if (cam_error_flag) {
        cam_error_flag = 0;
        HAL_UART_DMAStop(cam_uart); // 停止DMA

        cam_state = cam_wait_aa;    // 重置变量 
        cam_cmd = 0;
        cam_len = 0;
        cam_idx = 0;
        cam_sum = 0;

        cam_rd = 0;

        // 旧结果重置
        qr_ok = 0;
        qr_tick = 0;
        memset(&maix_vision, 0, sizeof(maix_vision));
        maix_filter_reset();
        maix_result.new_data = 1; // 通知上层这次结果已经失效
        maix_result.tick = HAL_GetTick();

        // 重新开启DMA空闲中断 禁止HC中断
        if (HAL_UARTEx_ReceiveToIdle_DMA(cam_uart, cam_rx_buf, CAM_LEN) == HAL_OK) {
            __HAL_DMA_DISABLE_IT(cam_uart->hdmarx, DMA_IT_HT);
            printf("[CAM RX] DMA错误后重启成功\r\n");
        } else {
            cam_error_flag = 1; // 保留恢复请求 下一次cam_task继续重试
        }
        return;
    }

    // 正常数据读取
    wr = CAM_LEN - __HAL_DMA_GET_COUNTER(cam_uart->hdmarx);  // DMA写指针
    if (wr >= CAM_LEN)
        wr = 0;

    while (cam_rd != wr) {
        uint8_t byte = cam_rx_buf[cam_rd];

        // 解析协议
        cam_byte_handle(byte);

        cam_rd++;
        if (cam_rd >= CAM_LEN)
            cam_rd = 0;
    }
}

// 通信层解析 cmd
void cam_frame_handle(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd) {
        case CMD_MV:

            break;

        case CMD_QR:
            openmv_frame_handle(data, len);
            break;

        case MAIX_CMD_BLOB:
        case MAIX_CMD_RING:
        case MAIX_CMD_ONUP:
        {
            maix_frame_handle(cmd, data, len);
            break;
        }

        default:
            break;
    }
}

// openmv 扫码数据帧处理
void openmv_frame_handle(const uint8_t *data, uint8_t len)
{
    // 只处理长度为 QR_DATA_LEN 的数据帧
    if (len != QR_DATA_LEN)
        return;

    memcpy(qr_data, data, QR_DATA_LEN); // 复制数据到 qr_data
    qr_data[QR_DATA_LEN] = '\0';    // 字符串结束符
    qr_tick = HAL_GetTick();
    qr_ok = 1;                      // 标记收到二维码数据
}

// maix 视觉数据帧处理
void maix_frame_handle(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    // 固定 maix 数据帧字节数为MAIX_DATA_LEN 即为6
    if (len != MAIX_DATA_LEN || cmd != maix_now_cmd ||
        maix_now_cmd == MAIX_CMD_NONE)
        return;

    // 接收解析 DATA数据 单帧数据
    maix_vision.cmd      = cmd;
    maix_vision.target   = maix_now_target;
    maix_vision.frame_id = data[0];
    maix_vision.ok       = data[1];
    maix_vision.dx       = read_i16_le(&data[2]);
    maix_vision.dy       = read_i16_le(&data[4]);

    maix_vision.tick     = HAL_GetTick();
    maix_vision.new_data = 1;

    maix_filter_handle(&maix_vision);
}

/////////////////////////////////////////// vision整体串口层 ///////////////////////////////////////////
// cam 初始化
void cam_init(UART_HandleTypeDef *huart)
{
    cam_uart = huart;
    cam_rd = 0;
    cam_rx_flag = 0;
    cam_error_flag = 0;

    cam_state = cam_wait_aa;       // 复位串口解析状态
    cam_cmd = 0;
    cam_len = 0;
    cam_idx = 0;
    cam_sum = 0;

    qr_ok = 0;
    qr_tick = 0;

    memset(&maix_vision, 0, sizeof(maix_vision));   // 内存清空
    maix_now_cmd = MAIX_CMD_NONE;
    maix_now_target = MAIX_COLOR_NONE;
    maix_filter_reset();

    // 开启DMA空闲中断接收 IDLE只通知主循环批量处理新字节
    if (cam_uart != NULL && cam_uart->hdmarx != NULL) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(cam_uart, cam_rx_buf, sizeof(cam_rx_buf)) == HAL_OK)
            __HAL_DMA_DISABLE_IT(cam_uart->hdmarx, DMA_IT_HT);
        else cam_error_flag = 1; // 初次启动失败也交给主循环恢复
    }
}

// 串口接收缓冲区清空，丢弃所有未处理的字节
void cam_empty_rx(void)
{
    uint16_t wr = 0;

    if (cam_uart != NULL && cam_uart->hdmarx != NULL)
    {
        wr = CAM_LEN - __HAL_DMA_GET_COUNTER(cam_uart->hdmarx);
        if (wr >= CAM_LEN)
            wr = 0;
    }

    cam_rd = wr;
    cam_state = cam_wait_aa;
    cam_cmd = 0;
    cam_len = 0;
    cam_idx = 0;
    cam_sum = 0;
    cam_rx_flag = 0;

    qr_ok = 0;
    qr_tick = 0;
    qr_data[0] = '\0';
}

// cam 串口接收中断回调   USART1
void cam_rx_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1)
        return;

    // 接收中断只置位 通知主循环过来处理接收到的新字节
    (void)Size;
    HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);
    if (event == HAL_UART_RXEVENT_IDLE) {
        cam_rx_flag = 1;
    }

    // cam_byte_handle(mv_rxb);                       // 把刚收到的字节丢去解析

    // // 重新开启cam的DMA空闲中断   放到主循环里处理，避免中断嵌套
    // HAL_UARTEx_ReceiveToIdle_DMA(cam_uart, cam_rx_buf, sizeof(cam_rx_buf));
    // __HAL_DMA_DISABLE_IT(cam_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
}

// cam 串口接收中断错误回调
void cam_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    // 只置位
    cam_error_flag = 1;
    cam_rx_flag = 1;
}


///////////////////////////////////// maix稳定化处理 ////////////////////////////////////////
// 清空滤波数据窗口并重置滤波状态
void maix_filter_reset(void)
{
    // 清空滤波数据窗口 重置状态
    memset(maix_filter, 0, sizeof(maix_filter));
    maix_filter_count = 0;
    maix_filter_write = 0;
    maix_last_frame_id = 0;
    maix_has_last_frame = 0;

    // 重置输出结果 保留当前的 cmd 和 target
    memset(&maix_result, 0, sizeof(maix_result));
    maix_result.cmd = maix_now_cmd;
    maix_result.target = maix_now_target;
}

// 将最新的单帧原始结果压入滤波窗口，并计算稳定化结果
void maix_filter_handle(const maix_vision_t *raw)
{
    int16_t valid_dx[MAIX_WINDOW_SIZE]; // 有效帧dx数组
    int16_t valid_dy[MAIX_WINDOW_SIZE]; // 有效帧dy数组
    uint8_t valid_count = 0;            // 有效帧计数

    uint8_t best_found = 0;             // 是否找到稳定簇
    uint16_t best_span_score = 255;     // 最小跨度评分
    int16_t best_dx = 0;                // 最佳簇的中位数dx
    int16_t best_dy = 0;                // 最佳簇的中位数dy

    // raw 需与当前的cmd 一致
    if (raw == NULL || raw->cmd != maix_now_cmd ||
        maix_now_cmd == MAIX_CMD_NONE)
        return;

    // 帧ID防重
    if (maix_has_last_frame && raw->frame_id == maix_last_frame_id)
        return;

    // 更新上一帧的帧ID  并标记存在上一帧数据
    maix_last_frame_id = raw->frame_id;
    maix_has_last_frame = 1;

    // raw 进入滤波窗口
    maix_filter[maix_filter_write].ok = raw->ok;
    maix_filter[maix_filter_write].dx = raw->dx;
    maix_filter[maix_filter_write].dy = raw->dy;

    // 更新写指针和计数
    maix_filter_write = (uint8_t)((maix_filter_write + 1) % MAIX_WINDOW_SIZE);
    if (maix_filter_count < MAIX_WINDOW_SIZE)
        maix_filter_count++;

    // 提取有效帧的偏移量 供后续计算
    for (uint8_t i = 0; i < maix_filter_count; i++)
    {
        if (maix_filter[i].ok)
        {
            valid_dx[valid_count] = maix_filter[i].dx;
            valid_dy[valid_count] = maix_filter[i].dy;
            valid_count++;
        }
    }

    // 填充结果结构体
    maix_result.cmd = raw->cmd;
    maix_result.target = raw->target;
    maix_result.frame_id = raw->frame_id;
    maix_result.sample_count = maix_filter_count;
    maix_result.valid_count = valid_count;
    maix_result.valid = (valid_count >= MAIX_VALID_MIN) ? 1 : 0;
    maix_result.stable = 0;
    maix_result.dx = 0;
    maix_result.dy = 0;
    maix_result.tick = raw->tick;

    // 稳定性判定 —— 寻找“小跨度簇”
    // 注: 算法目标：从最多5个有效点中，找出任意 3个 点，使得它们的 dx 和 dy 的跨度（最大差值）都小于预设阈值（MAIX_FILTER_STABLE_DX_SPAN 和 DY_SPAN）
    //      若找到，则认为这3点构成了稳定簇，输出其中位数作为最终稳定结果
    // 该算法允许存在跳点，只要有3个点在小跨度内即可判定稳定
    if (maix_result.valid)
    {
        uint8_t a;
        uint8_t b;
        uint8_t c;

        // 计算有效帧的全局中位数
        maix_result.dx = get_median_i16(valid_dx, valid_count);
        maix_result.dy = get_median_i16(valid_dy, valid_count);

        // 从最多5个有效点中寻找任意3点的小跨度簇；单个跳点不会破坏稳定判定
        for (a = 0; a + 2 < valid_count; a++)
        {
            for (b = (uint8_t)(a + 1); b + 1 < valid_count; b++)
            {
                for (c = (uint8_t)(b + 1); c < valid_count; c++)
                {
                    int16_t cluster_dx[3] = {valid_dx[a], valid_dx[b], valid_dx[c]};
                    int16_t cluster_dy[3] = {valid_dy[a], valid_dy[b], valid_dy[c]};
                    int16_t min_dx = cluster_dx[0];
                    int16_t max_dx = cluster_dx[0];
                    int16_t min_dy = cluster_dy[0];
                    int16_t max_dy = cluster_dy[0];
                    uint8_t k;

                    for (k = 1; k < 3; k++)
                    {
                        if (cluster_dx[k] < min_dx) min_dx = cluster_dx[k];
                        if (cluster_dx[k] > max_dx) max_dx = cluster_dx[k];
                        if (cluster_dy[k] < min_dy) min_dy = cluster_dy[k];
                        if (cluster_dy[k] > max_dy) max_dy = cluster_dy[k];
                    }

                    if ((max_dx - min_dx) <= MAIX_STABLE_DX_SPAN &&
                        (max_dy - min_dy) <= MAIX_STABLE_DY_SPAN)
                    {
                        uint16_t span_score = (uint16_t)(max_dx - min_dx) +
                                              (uint16_t)(max_dy - min_dy);
                        if (!best_found || span_score < best_span_score)
                        {
                            best_found = 1;
                            best_span_score = span_score;
                            best_dx = get_median_i16(cluster_dx, 3);
                            best_dy = get_median_i16(cluster_dy, 3);
                        }
                    }
                }
            }
        }

        // 所得出的稳定处理后的 x y 坐标
        if (best_found)
        {
            maix_result.stable = 1;
            maix_result.dx = best_dx;
            maix_result.dy = best_dy;
        }
    }

    maix_result.new_data = 1;
}


/////////////////////////////////////////// openmv扫码任务 ///////////////////////////////////////////

// 通信 openmv 开始扫码
void openmv_send_start_scan(void)
{
    if (cam_uart == NULL)
        return;

    uint8_t dat = 1;
    qr_tx_frame[0] = HEAD1;
    qr_tx_frame[1] = HEAD2;
    qr_tx_frame[2] = CMD_MV;
    qr_tx_frame[3] = 1;               // 长度 = 1
    qr_tx_frame[4] = dat;             // 数据 = 1
    qr_tx_frame[5] = check_sum(CMD_MV, 1, &dat);  // 校验

    qr_ok = 0;  // 清除旧结果
    qr_tick = 0;

    // 阻塞发送直接确认发送结果
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(cam_uart, qr_tx_frame, sizeof(qr_tx_frame), 100);
    printf("[OpenMV] 开始扫码   发送指令=%d: AA 55 B1 01 01 %02X\r\n",
           status, qr_tx_frame[5]);
}

// 通信 openmv 停止扫码
void openmv_send_stop_scan(void)
{
    if (cam_uart == NULL)
        return;

    uint8_t dat = 0;
    qr_tx_frame[0] = HEAD1;
    qr_tx_frame[1] = HEAD2;
    qr_tx_frame[2] = CMD_MV;
    qr_tx_frame[3] = 1;
    qr_tx_frame[4] = dat;
    qr_tx_frame[5] = check_sum(CMD_MV, 1, &dat);

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(cam_uart, qr_tx_frame, sizeof(qr_tx_frame), 100);
    printf("[OpenMV] 停止扫码   发送指令=%d: AA 55 B1 01 00 %02X\r\n",
        status, qr_tx_frame[5]);
}

// openmv 扫码任务
void openmv_qr_task(void)
{
    qr_result_t result;

    if (!openmv_get_qr_result(&result)) return;

    led_on();    // 亮灯提示收到码
    printf("OpenMV 扫码结果: %s\r\n", result.qr_data);
}

// 读取OpenMV任务码结果
uint8_t openmv_get_qr_result(qr_result_t *out)
{
    if (out == NULL || !qr_ok)
        return 0;

    memcpy(out->qr_data, qr_data, QR_DATA_LEN + 1);
    out->qr_tick = qr_tick;
    out->qr_ok = qr_ok;
    qr_ok = 0;  // 一帧BB只允许上层消费一次
    return 1;
}

//////////////////////////////////// maix 视觉任务 ///////////////////////////////////////////

// maix 发送帧数据
void maix_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (cam_uart == NULL) {
        printf("Maix TX failed: UART not initialized\r\n");
        return;
    }
    if (len > 1 || (len != 0 && data == NULL)) {
        printf("Maix TX failed: invalid len=%u\r\n", len);
        return;            // 此处的len只能为0/1  为0时 停止 为1时 启动maix
    }

    maix_tx_buf[0] = HEAD1;
    maix_tx_buf[1] = HEAD2;
    maix_tx_buf[2] = cmd;
    maix_tx_buf[3] = len;
    if (len == 1) {
        maix_tx_buf[4] = data[0];
    }
    maix_tx_buf[4 + len] = check_sum(cmd, len, data);

    // 测试命令只有 5~6 字节，使用阻塞发送可以直接确认发送结果，
    // 也避免 HAL_UART_Transmit_IT 因上一次发送未完成而返回 BUSY。
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(cam_uart, maix_tx_buf, 5 + len, 100);

    printf("Maix TX status=%d:", status);
    for (uint8_t i = 0; i < 5 + len; i++) {
        printf(" %02X", maix_tx_buf[i]);
    }
    printf("\r\n");
}

// maix 停止视觉任务
void maix_stop(void)
{
    maix_now_cmd = MAIX_CMD_NONE;
    maix_now_target = MAIX_COLOR_NONE;
    memset(&maix_vision, 0, sizeof(maix_vision));
    maix_filter_reset();
    maix_send_frame(MAIX_CMD_NONE, NULL, 0);
}

// maix 启动物料识别任务
void maix_start_blob(uint8_t color)
{
    maix_now_cmd = MAIX_CMD_BLOB;
    maix_now_target = color;
    memset(&maix_vision, 0, sizeof(maix_vision));
    maix_vision.target = color;
    maix_filter_reset();
    maix_send_frame(MAIX_CMD_BLOB, &color, 1);
}

// maix 启动数字识别与圆环定位任务
void maix_start_ring(uint8_t number)
{
    maix_now_cmd = MAIX_CMD_RING;
    maix_now_target = number;
    memset(&maix_vision, 0, sizeof(maix_vision));
    maix_vision.target = number;
    maix_filter_reset();
    maix_send_frame(MAIX_CMD_RING, &number, 1);
}

// maix 启动码垛定位摆放任务
void maix_start_onup(uint8_t color)
{
    maix_now_cmd = MAIX_CMD_ONUP;
    maix_now_target = color;
    memset(&maix_vision, 0, sizeof(maix_vision));
    maix_vision.target = color;
    maix_filter_reset();
    maix_send_frame(MAIX_CMD_ONUP, &color, 1);
}

// 在主循环中持续调用  若Maix超过设定时间没有提供新的唯一帧，立即撤销有效/稳定状态
void maix_result_task(void)
{
    uint32_t now;

    if (maix_now_cmd == MAIX_CMD_NONE || maix_filter_count == 0)
        return;

    now = HAL_GetTick();
    if ((uint32_t)(now - maix_result.tick) <= MAIX_TIMEOUT_MS)
        return;

    maix_filter_reset();
    maix_result.tick = now;
    maix_result.new_data = 1;
}

// maix 获取视觉结构体结果 原始单帧
uint8_t maix_get_result(maix_vision_t *out)
{
    if (out == NULL || !maix_vision.new_data) return 0;

    // 解析器运行在主循环而不是中断中，这里不需要全局关中断
    *out = maix_vision;
    maix_vision.new_data = 0;

    return 1;
}

// maix 获取滤波稳定结果  不影响原始单帧读取接口
uint8_t maix_get_filtered_result(maix_result_t *out)
{
    if (out == NULL || !maix_result.new_data) return 0;

    *out = maix_result;
    maix_result.new_data = 0;
    return 1;
}
