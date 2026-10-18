#include "headfile.h"

// ============ 地图数据 ============
// 5x5 地图：0 = 可通行，1 = 障碍物
static uint8_t bfs_map[BFS_MAP_SIZE][BFS_MAP_SIZE];

// 父节点记录：记录每个格子是从哪个格子走过来的，用于最后回溯路径
static int8_t bfs_prev_x[BFS_MAP_SIZE][BFS_MAP_SIZE];
static int8_t bfs_prev_y[BFS_MAP_SIZE][BFS_MAP_SIZE];

// 加权搜索：到每个格子的最小代价，以及进入该格子时的方向
#define BFS_DIR_NONE 0xFF
static uint16_t bfs_cost[BFS_MAP_SIZE][BFS_MAP_SIZE];
static uint8_t  bfs_dir_in[BFS_MAP_SIZE][BFS_MAP_SIZE];

// 找到的路径（起点 -> 终点）
static BFS_Point bfs_path[BFS_MAP_SIZE * BFS_MAP_SIZE];
static uint8_t bfs_path_len = 0;

// 上下左右四个方向：右、左、上、下
static const int8_t bfs_dx[4] = { 1, -1, 0, 0 };
static const int8_t bfs_dy[4] = { 0, 0, 1, -1 };

// 判断坐标是否在地图范围内
static uint8_t bfs_in_map(int8_t x, int8_t y)
{
    return (x >= 0 && x < BFS_MAP_SIZE && y >= 0 && y < BFS_MAP_SIZE);
}

// 初始化地图：所有格子清成可通行，路径清空
void bfs_init(void)
{
    memset(bfs_map, 0, sizeof(bfs_map));
    bfs_path_len = 0;
    bfs_base_obstacle();        // 基础障碍设置
}

// 设置/清除障碍物 1 设障碍 0 清除
void bfs_set_obstacle(uint8_t x, uint8_t y, uint8_t obstacle)
{
    if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
        bfs_map[y][x] = obstacle ? BFS_OBSTACLE : BFS_FREE;   // 内部存储为了直观 y行 x列
    }
}

// 清除所有障碍物（除了特定点）
void bfs_obstacle_clear(void)
{
    for(int x = 0; x < BFS_MAP_SIZE; x++) {
        for(int y = 0; y < BFS_MAP_SIZE; y++) {
            bfs_set_obstacle(x,y,0);
        }
    }
    bfs_set_obstacle(1,1,1);
    bfs_set_obstacle(3,1,1);
    bfs_set_obstacle(1,3,1);
    bfs_set_obstacle(3,3,1);
}

// 查询某格是否为障碍，越界按障碍处理
uint8_t bfs_is_obstacle(uint8_t x, uint8_t y)
{
    if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
        return bfs_map[y][x] == BFS_OBSTACLE;
    }
    return 1;
}

// 带转弯代价的最短路径搜索
// 注意：实际是带转弯代价的加权最短路径（Dijkstra算法），保留bfs名字是为了兼容旧调用
uint8_t bfs_find_path(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty)
{
    uint8_t i;
    uint8_t path_index = 0;
    int8_t nx, ny;           // 邻居坐标（可能越界，用有符号类型）
    BFS_Point tmp;           // 反转路径时的临时交换变量

    // 起点或终点越界，或者本身是障碍物 -> 无路可走
    if (!bfs_in_map((int8_t)sx, (int8_t)sy) ||
        !bfs_in_map((int8_t)tx, (int8_t)ty)) {
        return 0;
    }
    if (bfs_is_obstacle(sx, sy) || bfs_is_obstacle(tx, ty)) {
        return 0;
    }

    // 起点就是终点：路径只有起点自己
    if (sx == tx && sy == ty) {
        bfs_path[0].x = sx;
        bfs_path[0].y = sy;
        bfs_path_len = 1;
        return 1;
    }

    // 遍历所有格子：初始化代价、进入方向、父节点
    for (uint8_t yy = 0; yy < BFS_MAP_SIZE; yy++) {
        for (uint8_t xx = 0; xx < BFS_MAP_SIZE; xx++) {
            bfs_cost[yy][xx] = 0xFFFF;          // 到达该格子的最小代价
            bfs_dir_in[yy][xx] = BFS_DIR_NONE;  // 从哪个方向进入该格子（0右/1左/2上/3下），起点无方向
            bfs_prev_x[yy][xx] = -1;            // 该格子的父节点坐标，用于最后回溯
            bfs_prev_y[yy][xx] = -1;
        }
    }
    bfs_cost[sy][sx] = 0;

    // 反复松弛：重复更新邻居代价，直到代价不再变化
    for (int iter = 0; iter < BFS_MAP_SIZE * BFS_MAP_SIZE * 4; iter++) {
        uint8_t changed = 0;

        // 遍历每一行
        for (uint8_t yy = 0; yy < BFS_MAP_SIZE; yy++) {
            // 遍历每一列
            for (uint8_t xx = 0; xx < BFS_MAP_SIZE; xx++) {
                // 这个格子还没有可达代价，跳过
                if (bfs_cost[yy][xx] == 0xFFFF) {
                    continue;
                }

                // 尝试上下左右四个方向
                for (i = 0; i < 4; i++) {
                    // 邻居坐标：右、左、上、下
                    nx = (int8_t)(xx + bfs_dx[i]);
                    ny = (int8_t)(yy + bfs_dy[i]);

                    // 越界或障碍物不可走
                    if (!bfs_in_map(nx, ny)) {
                        continue;
                    }
                    if (bfs_is_obstacle((uint8_t)nx, (uint8_t)ny)) {
                        continue;
                    }

                    // 走一步的基本代价
                    uint16_t nd = (uint16_t)(bfs_cost[yy][xx] + 1);

                    // 如果进入 (yy,xx) 的方向和这次走向 (ny,nx) 的方向不同，
                    // 说明在这里转了一次弯，额外加 BFS_TURN_COST
                    if (bfs_dir_in[yy][xx] != BFS_DIR_NONE &&
                        bfs_dir_in[yy][xx] != i) {
                        nd += BFS_TURN_COST;
                    }

                    // 只有新代价更小才更新，保证 bfs_cost 一直是最小代价
                    if (nd < bfs_cost[ny][nx]) {
                        bfs_cost[ny][nx] = nd;
                        bfs_dir_in[ny][nx] = i;
                        bfs_prev_x[ny][nx] = (int8_t)xx;
                        bfs_prev_y[ny][nx] = (int8_t)yy;
                        changed = 1;
                    }
                }
            }
        }

        if (!changed) {
            break;   // 这一轮没有任何更新，代价已收敛
        }
    }

    // 终点代价还是无穷大 -> 被障碍挡死，无路可走
    if (bfs_cost[ty][tx] == 0xFFFF) {
        bfs_path_len = 0;
        return 0;
    }

    // ============ 回溯路径 ============
    // 从终点开始，不断找“我是从哪个格子来的”，一路倒推回起点。
    // 先存成“终点 -> 起点”，最后再反转。
    path_index = 0;
    bfs_path[path_index].x = tx;
    bfs_path[path_index].y = ty;
    path_index++;

    // 先取终点的父节点
    nx = bfs_prev_x[ty][tx];
    ny = bfs_prev_y[ty][tx];

    // 一路倒推，直到回到起点
    while (!(nx == (int8_t)sx && ny == (int8_t)sy) &&
           path_index < BFS_MAP_SIZE * BFS_MAP_SIZE) {
        bfs_path[path_index].x = (uint8_t)nx;
        bfs_path[path_index].y = (uint8_t)ny;
        path_index++;

        // 保存当前坐标，再查当前坐标的父节点，继续往上倒推
        uint8_t px = (uint8_t)nx;
        uint8_t py = (uint8_t)ny;
        nx = bfs_prev_x[py][px];
        ny = bfs_prev_y[py][px];
    }

    // 把起点也加进去
    bfs_path[path_index].x = sx;
    bfs_path[path_index].y = sy;
    path_index++;

    // 反转数组，变成“起点 -> 终点”的行走顺序
    for (i = 0; i < path_index / 2; i++) {
        tmp = bfs_path[i];
        bfs_path[i] = bfs_path[path_index - 1 - i];
        bfs_path[path_index - 1 - i] = tmp;
    }

    // 记录路径长度（包含起点和终点）
    bfs_path_len = path_index;
    return 1;
}

// 获取路径长度
uint8_t bfs_get_path_len(void)
{
    return bfs_path_len;
}

// 获取路径第 index 步的坐标
void bfs_get_path_step(uint8_t index, uint8_t *x, uint8_t *y)
{
    if (index < bfs_path_len) {
        *x = bfs_path[index].x;
        *y = bfs_path[index].y;
    }
}

// ============ 车头朝向与坐标换算 ============
// 小车局部坐标系：y 正 = 车头前方，x 正 = 车右侧
// 车头朝向向量表示车头在地图上的指向，只有上下左右

// 当前车头朝向向量，默认朝地图上方
int8_t bfs_heading_x = 0;
int8_t bfs_heading_y = 1;

// 设置车头朝向向量（例如朝地图x正传 (1,0)）
void bfs_set_heading_vec(int8_t hx, int8_t hy)
{
    if ((hx == 1 || hx == -1) && hy == 0) {
        bfs_heading_x = hx;
        bfs_heading_y = 0;
    }
    else if ((hy == 1 || hy == -1) && hx == 0) {
        bfs_heading_x = 0;
        bfs_heading_y = hy;
    }
}

// 获取车头朝向向量
void bfs_get_heading_vec(int8_t *hx, int8_t *hy)
{
    *hx = bfs_heading_x;
    *hy = bfs_heading_y;
}

// 把车体系移动量换算成地图绝对移动量
// car_dx>0 右移，car_dx<0 左移；car_dy>0 前进，car_dy<0 后退
void bfs_car_to_map(int8_t car_dx, int8_t car_dy,
                      int8_t *map_dx, int8_t *map_dy)
{
    // 车右侧向量 = 车头向量逆时针旋转90°（按你定义的坐标系）
    // 例如车头(0,1)时，车右侧为(1,0)
    int8_t right_x = bfs_heading_y;
    int8_t right_y = (int8_t)(-bfs_heading_x);

    // 地图位移 = 右侧分量 + 前进分量
    *map_dx = (int8_t)(car_dx * right_x + car_dy * bfs_heading_x);
    *map_dy = (int8_t)(car_dx * right_y + car_dy * bfs_heading_y);
}

// 设置默认障碍坐标（地图要求）
void bfs_base_obstacle(void)
{
    bfs_set_obstacle(1,1,1);    // (1,1)
    bfs_set_obstacle(3,1,1);    // (3,1)
    bfs_set_obstacle(1,3,1);    // (1,3)
    bfs_set_obstacle(3,3,1);    // (3,3)
}
