#ifndef __AX_LIDAR_H
#define __AX_LIDAR_H


// 按相邻有效起始包分段的雷达扫描；已观测错误会丢弃工作圈。
// 整个起始包静默丢失仍无法在这里证明物理整圈完整性。
typedef struct {
    uint16_t distance[360];  // 雷达距离数据数组  每个整数角度保留的最近距离
    uint8_t valid[360];      // 该整数角度在本圈是否真的收到过有效数据

    uint8_t intensity[360];  // 对应距离点的回波强度

    uint16_t raw_count;      // 本圈收到的原始采样点总数   即多个采样点落入同一整数角度也都计数  按手册算 应该是400
    uint16_t valid_count;    // 圈实际被填充的整数角度格子数量  最大360

    uint32_t scan_num;       // 发布扫描编号，不等于严格的物理旋转圈数
    uint32_t start_time;     // 本圈扫描开始时间戳
    uint32_t end_time;       // 本圈扫描结束时间戳
} lidar_scan_t;

// 独立小柱测试旁路：按实际采样顺序保存目标角区，不改变原有360格最近距离。
// angle_q6单位为1/64度；同角度的多个真实采样仍分别保存，distance=0也保留。
#define LIDAR_RAW_SECTOR_MAX 256 // 正面90度约100个原始采样，保留余量并继续检测溢出
typedef struct {
    uint16_t angle_q6;
    uint16_t distance;
    uint8_t intensity;
} lidar_raw_sample_t;

typedef struct {
    lidar_raw_sample_t samples[LIDAR_RAW_SECTOR_MAX];
    uint16_t count;
    uint16_t overflow;       // 超出容量的采样数，非0时本圈不能用于完整测量
    uint16_t center_deg;
    uint16_t half_width_deg;
    uint32_t scan_num;       // 与同圈lidar_scan_t编号相同
    uint32_t start_time;
    uint32_t end_time;
} lidar_raw_sector_t;

// 默认关闭，仅在主循环调用；更换角区后等下一完整扫描分段，不拼接请求前的半圈。
uint8_t lidar_raw_sector_start(uint16_t center_deg, uint16_t half_width_deg);
void lidar_raw_sector_stop(void);
const lidar_raw_sector_t *lidar_raw_sector_get(void);


extern uint16_t lidar_data[360];    // 雷达数据数组，存储360度的距离值

// static void lidar_scan_clear(lidar_scan_t *scan);
// static void lidar_scan_start(uint32_t tick);
// static void lidar_data_handle(void);
// static uint16_t lidar_read_u16(const uint8_t *data);

void lidar_init(UART_HandleTypeDef *huart);
void lidar_start(void);
void lidar_stop(void);
void lidar_change(void);
const lidar_scan_t* lidar_get_scan(void);   // 获取最新发布扫描；尚未发布时返回NULL
void lidar_process(void);
void lidar_error_callback(UART_HandleTypeDef *huart); // 中断标记错误，主循环恢复

uint8_t lidar_scan_ok(void);
uint32_t lidar_get_xor_ok_cnt(void);
uint32_t lidar_get_xor_err_cnt(void);
uint32_t lidar_get_scan_drop_cnt(void);   // 地图入口判断扫描中断，只保留这一项计数


#endif
