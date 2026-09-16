#ifndef GONGXUN_1_BFS_H
#define GONGXUN_1_BFS_H


// 纯搜索路径，不做控制

// 每格间隔cm
#define BLOCK_DISTANCE  50.0f

// 地图尺寸：5x5
#define BFS_MAP_SIZE  5

// 转弯代价：每转一次弯额外增加的代价
// 0 = 和纯BFS一样只看步数；越大越优先走少转弯的路径
#define BFS_TURN_COST 2

// 地图格子状态
#define BFS_FREE      0    // 可通行
#define BFS_OBSTACLE  1    // 障碍物，不可触碰

// 坐标结构体：x=列，y=行，范围 0~4
typedef struct {
    uint8_t x;   // 列
    uint8_t y;   // 行
} BFS_Point;

// 初始化地图：清空障碍物和路径
void bfs_init(void);

// 设置/清除障碍物：obstacle=1 设障碍，0 清除
void bfs_set_obstacle(uint8_t x, uint8_t y, uint8_t obstacle);

// 查询某格是否为障碍
uint8_t bfs_is_obstacle(uint8_t x, uint8_t y);

// 清除所有障碍物（除了特定点）
void bfs_obstacle_clear(void);

// BFS 搜索：从 (sx,sy) 到 (tx,ty)
// 返回 1 = 找到路径，0 = 无路可走
uint8_t bfs_find_path(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty);

// 获取路径长度（找到路径后有效）
uint8_t bfs_get_path_len(void);

// 获取路径第 index 步的坐标
void bfs_get_path_step(uint8_t index, uint8_t *x, uint8_t *y);

// ============ 车头朝向与坐标换算 ============
// 小车局部坐标系：y 正 = 车头前方，x 正 = 车右侧
// 车头朝向向量表示车头在地图上的指向，只有上下左右四个方向
// 当前车头朝向向量，默认朝地图上方
extern int8_t bfs_heading_x;
extern int8_t bfs_heading_y;

// 设置车头朝向向量（例如朝地图x正传 (1,0)）
void bfs_set_heading_vec(int8_t hx, int8_t hy);

// 获取车头朝向向量
void bfs_get_heading_vec(int8_t *hx, int8_t *hy);

// 把车体系移动量换算成地图绝对移动量
// car_dx>0 右移，car_dx<0 左移；car_dy>0 前进，car_dy<0 后退
void bfs_car_to_map(int8_t car_dx, int8_t car_dy,
                      int8_t *map_dx, int8_t *map_dy);

#endif //GONGXUN_1_BFS_H
