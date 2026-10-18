#ifndef GONGXUN_1_MAP_UPDATE_H
#define GONGXUN_1_MAP_UPDATE_H

#include <stdint.h>

// 道路连续两圈受到障碍影响才封锁；同一圈有几个框都只算一次
#define MAP_CONFIRM_SCAN_NUM    2
// 两次扫描间隔超过1秒就重新累计，不能拿很久以前的一圈凑确认次数
#define MAP_SCAN_TIMEOUT_MS     1000U

// 一圈雷达聚类转换成的地图候选框     (地图坐标系下)
typedef struct {
    float center_x;
    float center_y;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
} map_scan_obs_t;

// 启动或停车重置时初始化；car_width 是含展开机构的外接正方形边长
// 按正方形对角线的一半膨胀，覆盖任意车头角度；无效参数不接受雷达更新
uint8_t map_update_init(float car_width_mm, float safety_mm);
uint8_t map_update_is_ready(void);
void map_update_clear_candidates(void); // 位姿/接收中断时清零道路确认次数，保留封锁道路

// 输入完整一圈聚类结果；没有障碍也要提交count=0，让道路确认次数清零
// 本版只更新道路状态，不再向map_obs数组保存、配对单个障碍物
// 返回1表示本圈已处理；是否封路需读取道路状态，相同扫描编号不会重复确认
uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count,
                        uint32_t scan_num, uint32_t tick);

// 本模块确认障碍并封锁道路时递增，可供控制层判断是否需要重新规划
uint32_t map_update_change_get(void);

#endif // GONGXUN_1_MAP_UPDATE_H
