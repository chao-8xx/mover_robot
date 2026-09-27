#ifndef __AX_LIDAR_H
#define __AX_LIDAR_H


// 表示这一次完整的雷达扫描的一整圈数据
typedef struct {
    uint16_t distance[360];  // 雷达距离数据数组  每个整数角度保留的最近距离
    uint8_t valid[360];      // 该整数角度在本圈是否真的收到过有效数据

    uint8_t intensity[360];  // 对应距离点的回波强度

    uint16_t raw_count;      // 本圈收到的原始采样点总数   即多个采样点落入同一整数角度也都计数  按手册算 应该是400
    uint16_t valid_count;    // 圈实际被填充的整数角度格子数量  最大360

    uint32_t scan_num;       // 完整圈数编号
    uint32_t start_time;     // 本圈扫描开始时间戳
    uint32_t end_time;       // 本圈扫描结束时间戳
} lidar_scan_t;


extern uint16_t lidar_data[360];    // 雷达数据数组，存储360度的距离值

// static void lidar_scan_clear(lidar_scan_t *scan);
// static void lidar_scan_start(uint32_t tick);
// static void lidar_data_handle(void);
// static uint16_t lidar_read_u16(const uint8_t *data);

void lidar_init(UART_HandleTypeDef *huart);
void lidar_start(void);
void lidar_stop(void);
void lidar_change(void);
const lidar_scan_t* lidar_get_scan(void);   // 获取最新完成的一整圈 没有完整圈时返回 NULL
void lidar_process(void);

uint8_t lidar_scan_ok(void);
uint32_t lidar_get_xor_ok_cnt(void);
uint32_t lidar_get_xor_err_cnt(void);


#endif
