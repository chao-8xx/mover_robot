#ifndef GONGXUN_1_MAP_UPDATE_H
#define GONGXUN_1_MAP_UPDATE_H

// 道路连续两圈受到障碍影响才封锁；同一圈有几个框都只算一次
#define MAP_CONFIRM_SCAN_NUM    2
// 两次扫描间隔超过1秒就重新累计  不能拿很久以前的一圈凑确认次数
#define MAP_SCAN_TIMEOUT_MS     1000
// 每圈最多接收20个候选框  与旧的单个障碍物记录容量分开
#define MAP_SCAN_OBS_MAX        20

// 一圈雷达聚类转换成的地图候选框     (地图坐标系下)
typedef struct {
    float center_x;
    float center_y;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
} map_scan_obs_t;



// 清空道路与扫描记录；本版按道路区域封锁，不再配置车体膨胀参数
void map_update_init(void);
uint8_t map_update_is_ready(void);
void map_update_clear_candidates(void); // 位姿/接收中断时清零道路确认次数 保留封锁道路

// 明确占据的矩形输入，保留给预设地图测试；真实雷达用下方实际点形成的道路命中结果
// tick传扫描采集结束时间；complete=0表示结果不完整，只能封路、不能放行
uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count, uint32_t scan_num, uint32_t tick, uint8_t complete);

// road_mask低12位对应R0~R11，本圈实际点命中哪条路就置哪一位；内部仍按连续两圈确认
uint8_t map_update_roads(uint16_t road_mask, uint32_t scan_num, uint32_t tick, uint8_t complete);

// 上层确认待进入道路的整段通行范围已经有效观察后调用，不能仅凭没有聚类就调用
// scan_num必须是最近一次接受的扫描编号；tick传当前时间HAL_GetTick()，用于检查是否过期
// 返回1表示已确认可通行；本圈有障碍、数据无效/过期或道路已封锁时返回0，不自动解封
uint8_t map_update_confirm_free(uint8_t road_id, uint32_t scan_num, uint32_t tick);

// 进入道路前检查本次许可；旧FREE不代表现在可走，新扫描需要上层重新确认观察范围
uint8_t map_update_road_ready(uint8_t road_id, uint32_t tick);

// 本模块封路或显式放行时递增 重复确认同一状态不重复计数 直接map_road_set不计入
uint32_t map_update_change_get(void);

#endif // GONGXUN_1_MAP_UPDATE_H
