#include "headfile.h"


// 通信协议定义
#define HEAD1       0xAA
#define HEAD2       0x55
#define FRAME_LEN    10         // 帧长度  AA 55、M&T、LSN、FSA、LSA、CS
#define MAX_LEN      85         // 单帧最大长度


// 雷达数据定义
#define DMA_BUF_LEN  512     // DMA环形缓冲区大小
uint16_t lidar_data[360];    // 雷达数据数组，存储360度的距离值 只保存最近发布的完整一圈

// 串口与解析缓存变量
static UART_HandleTypeDef *lidar_uart = NULL;

static uint8_t rx_buf[MAX_LEN];         // 接收缓存    CPU从DMA缓冲区中读取有效字节 后组成一帧数据
static uint16_t rx_idx = 0;             // 当前已写入rx_buf的字节数量
static uint16_t rx_target_len = 0;      // 当前数据包应接收的总长度

static uint8_t dma_rx_buf[DMA_BUF_LEN]; // DMA接收环形缓冲区
static uint16_t dma_rd_ptr = 0;         // DMA读指针   从DMA缓冲区取数据的位置

// 协议层 双缓冲变量
static lidar_scan_t scan_buffer[2];     // 扫描双缓冲区
static lidar_scan_t *scan_work = NULL;  // 当前工作缓冲区
static lidar_scan_t *scan_ok   = NULL;  // 当前已完成扫描的缓冲区

static uint32_t lidar_xor_ok_cnt = 0;   // XOR校验成功计数
static uint32_t lidar_xor_err_cnt = 0;  // XOR校验失败计数

static uint8_t scan_ok_flag = 0;        // 扫描完成标志位
static uint8_t scan_collect_flag = 0;   // 扫描数据收集标志位
static uint32_t scan_num = 0;           // 扫描数据收集计数


/////////////////////////////////////// 内部辅助函数 ///////////////////////////////
// 读取小端16位数据
static uint16_t read_u16(const uint8_t *data) {
    return ((uint16_t)data[0]) | (uint16_t)data[1] << 8;
}

// 清空扫描区
static void lidar_scan_clear(lidar_scan_t *scan) {
    if (scan == NULL) return;

    memset(scan, 0, sizeof(*scan));
}

// 雷达开启扫描
static void lidar_scan_start(uint32_t tick) {
    lidar_scan_t *now_scan;

    if (scan_work == NULL || scan_ok == NULL) {
        return;
    }

    // 第一次检测雷达整圈数据  当前还没有完整的上一圈 只开始收集
    if (scan_collect_flag == 0) {
        lidar_scan_clear(scan_work);
        scan_work->start_time = tick;

        scan_collect_flag = 1;
        return;
    }

    // 除第一次之外的检测到起始包时: 说明 scan_work 中的上一圈已经结束
    scan_work->end_time = tick;
    scan_work->scan_num = ++scan_num;

    // 保存刚完成扫描的缓冲区地址 scan_work
    now_scan = scan_work;

    // 交换缓冲区指针
    scan_work = scan_ok;
    scan_ok = now_scan;

    memcpy(lidar_data,scan_ok->distance, sizeof(lidar_data));
    scan_ok_flag = 1;
    lidar_scan_clear(scan_work);

    // 开启下一圈扫描
    scan_work->start_time = tick;
}

// 计算D6 数据包的XOR校验值
static uint16_t lidar_xor_check(const uint8_t *data, uint16_t len) {
    uint16_t check_num;
    uint16_t offset;

    // 包头、M&T、LSN、FSA、LSA
    check_num  = read_u16(&data[0]);
    check_num ^= read_u16(&data[2]);
    check_num ^= read_u16(&data[4]);
    check_num ^= read_u16(&data[6]);

    // CS 位于data[8] data[9] 不参与计算
    for (uint8_t i = 0; i < len; i++) {
        offset = FRAME_LEN + (uint16_t)i * 3;

        // 每个采样点占3字节 前两个字节组成16位 最后一个字节单独参与XOR
        check_num ^= read_u16(&data[offset]);
        check_num ^= (uint16_t)data[offset + 2];
    }

    return check_num;
}


// 雷达数据解析
static void lidar_data_handle(void)
{
    // 手册协议字段
    uint8_t lsn;        // 代表 rx_buf[3]  采样点数
    uint16_t fsa_raw;   // 起始角度原始值
    uint16_t lsa_raw;   // 结束角度原始值

    uint16_t distance;      // 距离值
    uint8_t intensity;      // 强度值

    uint8_t is_scan_start;  // 是否为扫描起始点
    uint16_t index;         // 角度索引
    uint16_t sample_offset; // 采样点偏移量

    uint32_t now_tick;

    float angle_start;
    float angle_end;
    float angle_area;
    float angle;

    // rx_buf[3]是LSN，即当前包中的采样点数量
    lsn = rx_buf[3];

    if (lsn == 0){  // 无采样点数据包
        return;
    }

    // 读取FSA和LSA原始数据 小端模式
    fsa_raw = ((uint16_t)rx_buf[5] << 8) | (uint16_t)rx_buf[4];  // 起始角度原始值
    lsa_raw = ((uint16_t)rx_buf[7] << 8) | (uint16_t)rx_buf[6];

    // FSA和LSA最低位是角度校验位   手册规定该位固定为1
    if ((fsa_raw & 0x01) == 0 || (lsa_raw & 0x01) == 0) {  // 起始角度和结束角度的最低位必须为1，否则数据无效
        return;
    }

    // 协议的M&T 判断是否为扫描起始点
    is_scan_start = rx_buf[2] & 0x01;

    if (is_scan_start != 0) {    // 手册规定起始包中LSN必须为1
        if (lsn != 1) {
            return;
        }

        // 记录当前开始扫描的时间戳
        now_tick = HAL_GetTick();
        lidar_scan_start(now_tick);
    }

    if (scan_collect_flag == 0 || scan_work == NULL) {
        return;
    }

    // D6角度解析   最低校验位先右移去掉  然后除以64得到角度
    angle_start = (float)(fsa_raw >> 1) / 64.0f;
    angle_end   = (float)(lsa_raw >> 1) / 64.0f;

    // 处理角度跨越0度的情况，确保angle_end始终大于或等于angle_start
    if (angle_end < angle_start) {
        angle_end += 360.0f;
    }


    // // 处理特殊帧：如果rx_buf[3] == 0xC9，表示只有一个点的数据，直接解析这个点的距离和角度
    // if (rx_buf[3] == 0xC9) {
    //     index = (uint16_t)angle_start;
    //
    //     if (index >= 360) {
    //         index -= 360;
    //     }
    //
    //     distance = ((uint16_t)rx_buf[36] << 6) + (rx_buf[33] >> 2);
    //     lidar_data[index] = distance;
    // }
    // else {
    //     // 多点数据帧，根据起始角度、结束角度和点数计算每个点的角度间隔，并解析每个点的距离和角度

    // 计算当前数据包内相邻采样点的角度间隔
    if (lsn > 1) {
        angle_area = (angle_end - angle_start) / (float)(lsn - 1);
    }
    else {
        angle_area = 0.0f;
    }

    // 依次解析当前数据包中的采样点
    for (uint8_t i = 0; i < lsn; i++) {
        angle = angle_start + angle_area * (float)i;

        // 限制角度范围  确保angle在0~360度之间
        while (angle >= 360.0f) {
            angle -= 360.0f;
        }

        while (angle < 0.0f) {
            angle += 360.0f;
        }

        // 向下取整  将角度转换为整数索引
        index = (uint16_t)angle;

        // 数据区从rx_buf[10]开始 每个采样点占3字节 计算当前采样点在rx_buf中的偏移量
        sample_offset = FRAME_LEN + (uint16_t)i * 3;

        // 解析距离值 单位：mm   Si_H：Distance[13:6]  Si_2nd>>2 ：Distance[5:0]
        distance = ((uint16_t)rx_buf[sample_offset + 2] << 6)
                 | (rx_buf[1 + sample_offset] >> 2);

        // 解析强度值 单位：0~255  Si_2nd bit1:0：Intensity[7:6] Si_L   bit7:2：Intensity[5:0]
        intensity = (uint8_t)((rx_buf[sample_offset + 1] & 0x03) << 6)
                    | (rx_buf[sample_offset] >> 2);

        // 统计本次扫描原始数据点数
        scan_work->raw_count++;

        if (index >= 360) {
            continue;
        }
        if (distance == 0) {    // 无效数据点 跳过
            continue;
        }

        // 记录整数角度值在本圈中的第一次的有效距离
        if (scan_work->valid[index] == 0) {
            scan_work->valid[index] = 1;
            scan_work->distance[index] = distance;
            scan_work->intensity[index] = intensity;
            scan_work->valid_count++;
        }
        // 同一整数角度出现多个采样点时 只保留距离雷达最近的一个点
        else if (distance < scan_work->distance[index]) {
            scan_work->distance[index] = distance;
            scan_work->intensity[index] = intensity;
        }


    }
}


//////////////////////////////////////////// 雷达对外函数 //////////////////////////////////
// 雷达初始化
void lidar_init(UART_HandleTypeDef *huart) {
    lidar_uart = huart;

    // 初始化接收状态
    rx_idx = 0;
    rx_target_len = 0;
    dma_rd_ptr = 0;
    memset(lidar_data, 0, sizeof(lidar_data));

    // 初始化清空扫描缓冲区
    lidar_scan_clear(&scan_buffer[0]);
    lidar_scan_clear(&scan_buffer[1]);

    scan_work = &scan_buffer[0];
    scan_ok = &scan_buffer[1];

    lidar_xor_ok_cnt = 0;
    lidar_xor_err_cnt = 0;

    scan_ok_flag = 0;
    scan_collect_flag = 0;
    scan_num =0;

    // 开启DMA循环接收
    HAL_UART_Receive_DMA(lidar_uart, dma_rx_buf, DMA_BUF_LEN);
}

// 启动雷达
void lidar_start(void) {
    // 必须使用 static 或全局变量，保证中断发送过程中内存不被释放
    static const uint8_t cmd[4] = {0xAA, 0x55, 0xF0, 0x0F};
    printf("开启雷达\r\n");

    HAL_UART_Transmit_IT(lidar_uart, (uint8_t *)cmd, 4);
}

// 停止雷达
void lidar_stop(void) {

    static const uint8_t cmd[4] = {0xAA, 0x55, 0xF5, 0x0A};
    printf("停止雷达\r\n");

    HAL_UART_Transmit_IT(lidar_uart, (uint8_t *)cmd, 4);
}

// 用于启动/停止雷达电机
void lidar_change(void) {
    static uint8_t lidar_state = 0;

    if(lidar_state == 0) {
        lidar_start();
        lidar_state = 1;
    } else {
        lidar_stop();
        lidar_state = 0;
    }
}


// 当前是否已扫描完数据
uint8_t lidar_scan_ok(void) {
    return scan_ok_flag;
}

// 获取xor 校验成功计数
uint32_t lidar_get_xor_ok_cnt(void) {
    return lidar_xor_ok_cnt;
}

// 获取xor 校验失败计数
uint32_t lidar_get_xor_err_cnt(void) {
    return lidar_xor_err_cnt;
}

// 获取当前扫描数据  const表示只读 不可被上层修改
const lidar_scan_t *lidar_get_scan(void) {
    if (scan_ok_flag == 0) {
        return NULL;
    }

    return scan_ok;
}


// DMA环形缓冲区轮询处理
void lidar_process(void) {
    uint16_t dma_wr_ptr; // DMA写指针
    uint8_t res;         // 暂存从 DMA 环形缓冲区中读出的一个字节
    uint8_t lsn;         // 当前数据包中的采样点数

    if (lidar_uart == NULL || lidar_uart->hdmarx == NULL) return;

    // 获取当前DMA写指针   __HAL_DMA_GET_COUNTER返回DMA当前剩余未传输的数量
    dma_wr_ptr = DMA_BUF_LEN - (uint16_t)__HAL_DMA_GET_COUNTER(lidar_uart->hdmarx);

    if (dma_wr_ptr >= DMA_BUF_LEN) {
        dma_wr_ptr = 0;
    }

    // 从DMA缓冲区循环读取数据
    while (dma_rd_ptr != dma_wr_ptr) {
        res = dma_rx_buf[dma_rd_ptr];    // 暂存从 DMA 环形缓冲区中读出的一个字节
        dma_rd_ptr = (dma_rd_ptr + 1) % DMA_BUF_LEN;// 更新读指针 循环回绕

        // 数据帧解析状态机
        if (rx_idx == 0) {      // 帧头字节0xAA
            if (res == HEAD1){
                rx_buf[0] = res;
                rx_idx = 1;
            }
            continue;
        }
            if (rx_idx == 1) {  // 帧头字节0x55
                if (res == HEAD2) {
                    rx_buf[1] = res;
                    rx_idx = 2;
                }
                else if (res == HEAD1) { // 如果连续收到两个0xAA 说明可能是新的帧头，重新开始接收
                    rx_buf[0] = res;
                    rx_idx = 1;
                }
                else {
                    rx_idx = 0;
                    rx_target_len = 0;
                }
                continue;
            }

        // 最大范围限制
        if (rx_idx >= MAX_LEN) {
            rx_idx = 0;
            rx_target_len = 0;
            continue;
        }
        // 保存当前字节
        rx_buf[rx_idx] = res;
        rx_idx++;

        // 接收完前4字节后已经得到LSN
        if (rx_idx == 4) {
            lsn = rx_buf[3];

            rx_target_len = FRAME_LEN + (uint16_t)lsn * 3;

            // 当前缓冲区最多支持25个采样点
            if (rx_target_len > MAX_LEN) {
                rx_idx = 0;
                rx_target_len = 0;
                continue;
            }
        }

        // rx_target_len非0且已收到完整数据包
        if (rx_target_len != 0 && rx_idx == rx_target_len) {
            lsn = rx_buf[3];

            // 计算值必须等于帧内小端CS；校验值为0也可以合法，不能仅判断非零
            if (lidar_xor_check(rx_buf, lsn) == read_u16(&rx_buf[8])) {
                lidar_xor_ok_cnt++;
                lidar_data_handle();
            }
            else {              // xor校验失败 数据包直接丢弃
                lidar_xor_err_cnt++;
            }

            rx_idx = 0;
            rx_target_len = 0;
        }
    }
}







