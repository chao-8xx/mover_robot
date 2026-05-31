#include "../Inc/headfile.h"

// 帧头
#define LD_HEADER1      0xAA
#define LD_HEADER2      0x55

// 帧长度和最大长度
#define LD_F_LEN        10
#define LD_MAX_LEN      85

// 雷达数据数组，存储360度的距离值
uint16_t ax_lidar_data[360];

static UART_HandleTypeDef *lidar_uart = NULL;

static uint8_t rx_byte;             // 接收字节
static uint8_t uart_rx_con = 0;     // 接收计数
static uint8_t uart_rx_max = 0;     // 接收最大长度
static uint8_t uart_rx_buf[100];    // 接收缓冲区

// 雷达初始化
void AX_LIDAR_Init(UART_HandleTypeDef *huart)
{
    lidar_uart = huart;

    uart_rx_con = 0;
    uart_rx_max = 0;
    memset(ax_lidar_data, 0, sizeof(ax_lidar_data));

    HAL_UART_Receive_IT(lidar_uart, &rx_byte, 1);
}

// 启动雷达
void AX_LIDAR_Start(void)
{
    uint8_t cmd[4] = {0xAA, 0x55, 0xF0, 0x0F};

    HAL_UART_Transmit(lidar_uart, cmd, 4, 100);
}

// 停止雷达
void AX_LIDAR_Stop(void)
{
    uint8_t cmd[4] = {0xAA, 0x55, 0xF5, 0x0A};

    HAL_UART_Transmit(lidar_uart, cmd, 4, 100);
}

// 雷达数据解析
void LD_DataHandle(void)
{
    uint8_t i;
    uint16_t index;
    uint16_t distance;

    float angle_start;
    float angle_end;
    float angle_area;

    angle_start = (float)((((uint16_t)uart_rx_buf[5] << 8) + uart_rx_buf[4]) >> 1) / 64.0f;
    angle_end   = (float)((((uint16_t)uart_rx_buf[7] << 8) + uart_rx_buf[6]) >> 1) / 64.0f;

    if (angle_end < angle_start)
    {
        angle_end += 360.0f;
    }

    if (uart_rx_buf[3] == 0)
    {
        return;
    }

    if (uart_rx_buf[3] == 0xC9)
    {
        index = (uint16_t)angle_start;

        if (index >= 360)
        {
            index -= 360;
        }

        distance = ((uint16_t)uart_rx_buf[36] << 6) + (uart_rx_buf[33] >> 2);
        ax_lidar_data[index] = distance;
    }
    else
    {
        if (uart_rx_buf[3] > 1)
        {
            /* ensure floating-point division to avoid narrowing conversion warnings */
            angle_area = (angle_end - angle_start) / (float)(uart_rx_buf[3] - 1);
        }
        else
        {
            angle_area = 0;
        }

        for (i = 0; i < uart_rx_buf[3]; i++)
        {
            /* cast index to float to avoid int->float narrowing warnings */
            float angle = angle_start + angle_area * (float)i;

            while (angle >= 360.0f)
            {
                angle -= 360.0f;
            }

            while (angle < 0.0f)
            {
                angle += 360.0f;
            }

            index = (uint16_t)angle;

            distance = ((uint16_t)uart_rx_buf[12 + i * 3] << 6)
                     + (uart_rx_buf[11 + i * 3] >> 2);

            if (index < 360)
            {
                ax_lidar_data[index] = distance;
            }
        }
    }
}

// 串口接收完成回调
void AX_LIDAR_RxCallback(UART_HandleTypeDef *huart)
{
    uint8_t Res = rx_byte;

    if (huart != lidar_uart)
    {
        return;
    }

    if (uart_rx_con < 2)
    {
        if (uart_rx_con == 0)
        {
            if (Res == LD_HEADER1)
            {
                uart_rx_buf[uart_rx_con] = Res;
                uart_rx_con = 1;
            }
        }
        else
        {
            if (Res == LD_HEADER2)
            {
                uart_rx_buf[uart_rx_con] = Res;
                uart_rx_con = 2;
            }
            else
            {
                uart_rx_con = 0;
            }
        }
    }
    else
    {
        if (uart_rx_con < LD_F_LEN)
        {
            uart_rx_buf[uart_rx_con] = Res;
            uart_rx_con++;

            if (uart_rx_con == 4)
            {
                uart_rx_max = LD_F_LEN + uart_rx_buf[3] * 3;

                if (uart_rx_max > LD_MAX_LEN)
                {
                    uart_rx_con = 0;
                }
            }
        }
        else if (uart_rx_con < uart_rx_max)
        {
            uart_rx_buf[uart_rx_con] = Res;
            uart_rx_con++;
        }
        else
        {
            LD_DataHandle();
            uart_rx_con = 0;
        }
    }

    HAL_UART_Receive_IT(lidar_uart, &rx_byte, 1);
}
