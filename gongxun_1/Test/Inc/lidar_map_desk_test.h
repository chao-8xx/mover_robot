#ifndef GONGXUN_1_LIDAR_MAP_DESK_TEST_H
#define GONGXUN_1_LIDAR_MAP_DESK_TEST_H

// 固定测试位姿：雷达放在虚拟N1，原始0度朝N4；不是实际全局定位。
#define LIDAR_MAP_DESK_X             1200.0f
#define LIDAR_MAP_DESK_Y             2125.0f
#define LIDAR_MAP_DESK_YAW            180.0f
#define LIDAR_MAP_DESK_CURRENT_NODE       1
#define LIDAR_MAP_DESK_NEXT_NODE          4
#define LIDAR_MAP_DESK_ROAD               8
#define LIDAR_MAP_DESK_WHEEL_MASK         0 // 当前只接开发板和裸雷达；装车后重新验证屏蔽

// 诊断使用实际点计数，包围框中心归属只供显示，不代替真实点判路。
typedef struct {
    map_area_t center_area;
    map_cell_t center_cell;
    uint16_t roads;
    uint16_t road_points, node_points, forbidden_points, outside_points;
    uint8_t in_front, in_task;
} lidar_map_desk_candidate_t;

// 主循环/主机回放共用；清空本次测试地图与位置窗口，不发送底盘指令。
uint8_t lidar_map_desk_begin(void);
uint8_t lidar_map_desk_process(const lidar_scan_t *scan, uint32_t tick);
uint8_t lidar_map_desk_candidate(uint16_t index, lidar_map_desk_candidate_t *out);
void lidar_map_desk_test_init(void);
void lidar_map_desk_test(void);

#endif
