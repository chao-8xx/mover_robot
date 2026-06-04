#include "../Inc/headfile.h"


// 通信协议定义
#define HEAD1       0xAA
#define HEAD2       0x55
#define FRAME_LEN    10         // 帧长度
#define MAX_LEN      85         // 单帧最大长度
#define DMA_BUF_LEN  512        // DMA环形缓冲区大小

// 雷达数据定义
uint16_t lidar_data[360];    // 雷达数据数组，存储360度的距离值 单位：mm

static UART_HandleTypeDef *lidar_uart = NULL;

uint8_t rx_buf[MAX_LEN];         // 接收缓存    CPU从DMA缓冲区中读取有效字节 后组成一帧数据
uint8_t rx_idx = 0;              // 当前接收索引
uint8_t rx_target_len = 0;       // 当前帧期望长度

uint8_t dma_rx_buf[DMA_BUF_LEN]; // DMA接收环形缓冲区
uint16_t dma_rd_ptr = 0;         // DMA读指针   从DMA缓冲区取数据的位置

// 雷达初始化
void lidar_init(UART_HandleTypeDef *huart)
{
    lidar_uart = huart;

    // 初始化接收状态
    rx_idx = 0;
    rx_target_len = 0;
    memset(lidar_data, 0, sizeof(lidar_data));

    // 开启DMA循环接收
    HAL_UART_Receive_DMA(lidar_uart, dma_rx_buf, DMA_BUF_LEN);
    dma_rd_ptr = 0;
}

// 启动雷达
void lidar_start(void)
{
    // 必须使用 static 或全局变量，保证中断发送过程中内存不被释放
    static const uint8_t cmd[4] = {0xAA, 0x55, 0xF0, 0x0F};
    printf("开启雷达\r\n");

    HAL_UART_Transmit_IT(lidar_uart, (uint8_t *)cmd, 4);
}

// 停止雷达
void lidar_stop(void)
{
    static const uint8_t cmd[4] = {0xAA, 0x55, 0xF5, 0x0A};
    printf("停止雷达\r\n");

    HAL_UART_Transmit_IT(lidar_uart, (uint8_t *)cmd, 4);
}

// 用于启动/停止雷达电机
void lidar_change(void) {
    static uint8_t lidar_state = 0;
    if(lidar_state == 0)
    {
        lidar_start();
        lidar_state = 1;
    }
    else
    {
        lidar_stop();
        lidar_state = 0;
    }
}

// 雷达数据解析
void lidar_data_handle(void)
{
    uint8_t i;
    uint16_t index;
    uint16_t distance;

    float angle_start;
    float angle_end;
    float angle_area;

    // 解析起始角度和结束角度，单位：度
    angle_start = (float)((((uint16_t)rx_buf[5] << 8) + rx_buf[4]) >> 1) / 64.0f;
    angle_end   = (float)((((uint16_t)rx_buf[7] << 8) + rx_buf[6]) >> 1) / 64.0f;

    // 处理角度跨越0度的情况，确保angle_end始终大于或等于angle_start
    if (angle_end < angle_start)
    {
        angle_end += 360.0f;
    }

    if (rx_buf[3] == 0)
    {
        return;
    }

    // 处理特殊帧：如果rx_buf[3] == 0xC9，表示只有一个点的数据，直接解析这个点的距离和角度
    if (rx_buf[3] == 0xC9)
    {
        index = (uint16_t)angle_start;

        if (index >= 360)
        {
            index -= 360;
        }

        distance = ((uint16_t)rx_buf[36] << 6) + (rx_buf[33] >> 2);
        lidar_data[index] = distance;
    }
    else
    {
        // 多点数据帧，根据起始角度、结束角度和点数计算每个点的角度间隔，并解析每个点的距离和角度
        if (rx_buf[3] > 1)
        {
            angle_area = (angle_end - angle_start) / (float)(rx_buf[3] - 1);
        }
        else
        {
            angle_area = 0;
        }

        for (i = 0; i < rx_buf[3]; i++)
        {
            /* cast index to float to avoid int->float narrowing warnings */
            float angle = angle_start + angle_area * (float)i;

            // 处理角度范围，确保angle在0~360度之间
            while (angle >= 360.0f)
            {
                angle -= 360.0f;
            }

            while (angle < 0.0f)
            {
                angle += 360.0f;
            }

            // 将角度转换为整数索引，单位：度
            index = (uint16_t)angle;

            // 解析距离值，单位：mm
            distance = ((uint16_t)rx_buf[12 + i * 3] << 6)
                     + (rx_buf[11 + i * 3] >> 2);

            if (index < 360)
            {
                lidar_data[index] = distance;
            }
        }
    }
}

// DMA环形缓冲区轮询处理
void lidar_process(void)
{
    if (lidar_uart == NULL || lidar_uart->hdmarx == NULL) return;

    // 获取当前DMA写指针
    //  注: __HAL_DMA_GET_COUNTER返回DMA当前剩余未传输的数量
    uint16_t dma_wr_ptr = DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(lidar_uart->hdmarx);

    // 从DMA缓冲区循环读取数据，直到读指针追上写指针
    while (dma_rd_ptr != dma_wr_ptr)
    {
        uint8_t res = dma_rx_buf[dma_rd_ptr];       // 暂存从 DMA 环形缓冲区中读出的一个字节
        dma_rd_ptr = (dma_rd_ptr + 1) % DMA_BUF_LEN;// 更新读指针，循环回绕

        // 数据帧解析状态机
        if (rx_idx < 2)                  // 帧头解析
        {
            if (rx_idx == 0)
            {
                if (res == HEAD1)
                {
                    rx_buf[rx_idx] = res;
                    rx_idx = 1;
                }
            }
            else
            {
                if (res == HEAD2)
                {
                    rx_buf[rx_idx] = res;
                    rx_idx = 2;
                }
                else
                {
                    rx_idx = 0;
                }
            }
        }
        else
        {
            if (rx_idx < FRAME_LEN)     // 接收前导固定长度部分   FRAME_LEN 10字节，包含点数信息
            {
                rx_buf[rx_idx] = res;
                rx_idx++;

                if (rx_idx == 4)        // 接收完前4字节后，根据第4字节的点数信息计算整帧长度
                {
                    // rx_buf[3] 为数据点个数，每点占3字节    整帧长度 = 前导固定长度 + 点数 * 每点字节数
                    rx_target_len = FRAME_LEN + rx_buf[3] * 3;

                    if (rx_target_len > MAX_LEN)
                    {
                        rx_idx = 0;
                    }
                }
            }
            else if (rx_idx < rx_target_len)    // 接收剩余数据，根据之前计算的整帧长度继续接收数据，直到接收完整帧数据
            {
                rx_buf[rx_idx] = res;
                rx_idx++;

                if (rx_idx >= rx_target_len)
                {
                    lidar_data_handle();
                    rx_idx = 0;
                }
            }
        }
    }
}
