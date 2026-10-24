#ifndef GONGXUN_1_MAP_NAV_H
#define GONGXUN_1_MAP_NAV_H

#include "map_plan.h"

// 导航参数  软件初值 待定位和低速实车联调后调整
#define MAP_NAV_NODE_TOL_MM        30.0f   // 节点到位范围 mm
#define MAP_NAV_PATH_TOL_MM        50.0f   // 行驶中允许偏离道路中心线的范围 mm
#define MAP_NAV_YAW_TOL_DEG         1.2f   // 车头对准道路的角度范围
#define MAP_NAV_SETTLE_MM           5.0f   // 停稳观察期间允许的位置变化 mm
#define MAP_NAV_SETTLE_YAW_DEG      1.0f   // 停稳观察期间允许的角度变化
#define MAP_NAV_SETTLE_MS           300   // 连续新位姿确认停稳的时间 ms
#define MAP_NAV_WAIT_TIMEOUT_MS  10000   // 观察、转向和到位确认的超时时间 ms
#define MAP_NAV_MOVE_TIMEOUT_MS  30000   // 单条道路移动超时时间 ms
#define MAP_NAV_MAX_SPEED          200   // 本模块允许的电机指令速度上限 RPM

// 导航运行状态
typedef enum
{
    map_nav_idle = 0,       // 空闲 尚未接收任务
    map_nav_plan,           // 从已确认节点规划路径
    map_nav_scan,           // 等待停稳和下一条道路许可
    map_nav_align,          // 车头对准下一条道路
    map_nav_move,           // 已下发本段移动 等待底盘结束
    map_nav_arrive,         // 等待实际到位和停稳
    map_nav_finish,         // 已到达目标节点
    map_nav_stop            // 取消或异常停车
} map_nav_state_t;

// 停车或启动失败的原因
typedef enum
{
    map_nav_error_none = 0, // 无错误
    map_nav_error_param,    // 节点、速度或未知路策略参数错误
    map_nav_error_pose,     // 位姿无效、过期或角度参考发生变化
    map_nav_error_start,    // 实际位置不在指定起点附近
    map_nav_error_busy,     // 底盘忙、指令未接受或控制指令冲突
    map_nav_error_no_path,  // 当前道路状态下无法规划到目标
    map_nav_error_road,     // 地图未初始化或当前道路不可继续使用
    map_nav_error_timeout,  // 当前阶段超过允许时间
    map_nav_error_deviation,// 车辆偏离当前节点或规划道路范围
    map_nav_error_cancel    // 上层主动取消任务
} map_nav_error_t;

// 导航结果  current_node只在实际到位并停稳后更新
typedef struct
{
    map_nav_state_t state;  // 当前导航运行状态
    map_nav_error_t error;  // 最近一次启动失败或停车的原因
    uint8_t current_node;   // 最后确认到达的节点  道路中间停车时不冒充已到下一节点
    uint8_t target_node;    // 本次任务最终目标节点
    uint8_t next_node;      // 当前段准备到达的下一节点
    uint8_t road_id;        // 当前段道路编号 无道路时为MAP_NO_ID
    uint8_t road_index;     // 正在处理path.roads中的哪一条道路
    map_path_t path;        // 当前规划路径 按起点到终点排列
} map_nav_t;

///////////////////////////////////// 导航接口 ////////////////////////////////////////
// 全部接口在主循环调用  本模块运行时须由上层保证独占底盘
// 初始化会取消本模块正在执行的任务  不清空道路地图或定位结果
void map_nav_init(void);

// 从指定节点前往目标节点  实际位姿必须位于start附近 且底盘空闲
// speed单位RPM  允许范围1~MAP_NAV_MAX_SPEED  返回1表示接受任务
// 拒绝替换正在执行的任务  先cancel再启动  不接管其他任务正在使用的底盘
uint8_t map_nav_start(uint8_t start, uint8_t goal, map_unknown_mode_t mode, int16_t speed);

// 周期执行  不阻塞  只在获准进入道路时下发一次移动指令
// 外部先发布新位姿、处理静止建图及有效观察放行  本模块不自行确认FREE
// 运动中近场危险检测由后续模块补充  本阶段不在正式主循环自动启用
void map_nav_task(void);
// 主动结束本模块任务 不自动恢复行驶
void map_nav_cancel(void);
// 读取当前导航结果 out非空时返回1
uint8_t map_nav_get_result(map_nav_t *out);

#endif //GONGXUN_1_MAP_NAV_H
