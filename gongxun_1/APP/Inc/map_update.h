#ifndef GONGXUN_1_MAP_UPDATE_H
#define GONGXUN_1_MAP_UPDATE_H

// 道路连续两圈受到障碍影响才封锁；同一圈有几个框都只算一次
#define MAP_CONFIRM_SCAN_NUM    2
// 两次扫描间隔超过1秒就重新累计  不能拿很久以前的一圈凑确认次数
#define MAP_SCAN_TIMEOUT_MS     1000

// 一圈雷达聚类转换成的地图候选框     (地图坐标系下)
typedef struct {
    float center_x;
    float center_y;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
} map_scan_obs_t;



uint8_t map_update_init(float car_width_mm, float safety_mm);
uint8_t map_update_is_ready(void);
void map_update_clear_candidates(void); // 位姿/接收中断时清零道路确认次数 保留封锁道路

// 输入完整一圈聚类结果  只更新道路状态
uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count, uint32_t scan_num, uint32_t tick);

// 上层确认待进入道路的整段通行范围已经有效观察后调用，不能仅凭没有聚类就调用
// scan_num必须是最近一次接受的扫描编号；tick传当前时间HAL_GetTick()，用于检查是否过期
// 返回1表示已确认可通行；本圈有障碍、数据无效/过期或道路已封锁时返回0，不自动解封
uint8_t map_update_confirm_free(uint8_t road_id, uint32_t scan_num, uint32_t tick);

// 本模块封路或显式放行时递增 重复确认同一状态不重复计数 直接map_road_set不计入
uint32_t map_update_change_get(void);

#endif // GONGXUN_1_MAP_UPDATE_H
