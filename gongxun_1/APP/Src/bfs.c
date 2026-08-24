#include "bfs.h"

// ============ 地图数据 ============
// 5x5 地图：0 = 可通行，1 = 障碍物
static uint8_t bfs_map[BFS_MAP_SIZE][BFS_MAP_SIZE];

// BFS 访问标记：1 = 已经搜索过
static uint8_t bfs_visited[BFS_MAP_SIZE][BFS_MAP_SIZE];

// 父节点记录：记录每个格子是从哪个格子走过来的，用于最后回溯路径
static int8_t bfs_prev_x[BFS_MAP_SIZE][BFS_MAP_SIZE];
static int8_t bfs_prev_y[BFS_MAP_SIZE][BFS_MAP_SIZE];

// BFS 搜索队列：最多 25 个格子
static BFS_Point bfs_queue[BFS_MAP_SIZE * BFS_MAP_SIZE];

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
}

// 设置/清除障碍物
void bfs_set_obstacle(uint8_t x, uint8_t y, uint8_t obstacle)
{
    if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
        bfs_map[y][x] = obstacle ? BFS_OBSTACLE : BFS_FREE;   // 内部存储为了直观 y行 x列
    }
}

// 查询某格是否为障碍，越界按障碍处理
uint8_t bfs_is_obstacle(uint8_t x, uint8_t y)
{
    if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
        return bfs_map[y][x] == BFS_OBSTACLE;
    }
    return 1;
}

// BFS 搜索主函数：
// 从起点开始逐层向外扩展，第一次到达终点的那条路就是最短路径
uint8_t bfs_find_path(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty)
{
    uint8_t q_head = 0;      // 队列头：下一个要扩展的格子
    uint8_t q_tail = 0;      // 队列尾：新格子入队的位置
    uint8_t i;
    uint8_t path_index;
    int8_t nx, ny;           // 新扩展出来的邻居坐标
    BFS_Point cur;           // 当前正在扩展的格子
    BFS_Point tmp;

    // 起点或终点越界/是障碍 -> 直接失败
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

    // 清空访问标记和父节点记录
    memset(bfs_visited, 0, sizeof(bfs_visited));
    memset(bfs_prev_x, -1, sizeof(bfs_prev_x));
    memset(bfs_prev_y, -1, sizeof(bfs_prev_y));

    // 起点入队并标记访问
    bfs_queue[q_tail].x = sx;
    bfs_queue[q_tail].y = sy;
    q_tail++;
    bfs_visited[sy][sx] = 1;

    while (q_head < q_tail) {
        // 出队（读队头）：每次从 q_head 位置取最前面的格子，然后 head 后移
        cur = bfs_queue[q_head];
        q_head++;

        // 尝试上下左右四个方向
        for (i = 0; i < 4; i++) {
            // 搜索顺序：右、左、上、下
            nx = (int8_t)(cur.x + bfs_dx[i]);
            ny = (int8_t)(cur.y + bfs_dy[i]);

            // 越界、访问过、是障碍 -> 跳过
            if (!bfs_in_map(nx, ny)) {
                continue;
            }
            if (bfs_visited[ny][nx]) {
                continue;
            }
            if (bfs_is_obstacle((uint8_t)nx, (uint8_t)ny)) {
                continue;
            }

            // 标记访问并记录“我是从哪个格子来的”
            bfs_visited[ny][nx] = 1;
            bfs_prev_x[ny][nx] = (int8_t)cur.x;
            bfs_prev_y[ny][nx] = (int8_t)cur.y;

            // 到达终点：回溯父节点得到完整路径
            if ((uint8_t)nx == tx && (uint8_t)ny == ty) {
                // 从终点倒推回起点，先存在 bfs_path 里（终点 -> 起点）
                path_index = 0;
                bfs_path[path_index].x = (uint8_t)nx;
                bfs_path[path_index].y = (uint8_t)ny;
                path_index++;

                nx = bfs_prev_x[ty][tx];
                ny = bfs_prev_y[ty][tx];

                while (!(nx == (int8_t)sx && ny == (int8_t)sy) &&
                       path_index < BFS_MAP_SIZE * BFS_MAP_SIZE) {
                    bfs_path[path_index].x = (uint8_t)nx;
                    bfs_path[path_index].y = (uint8_t)ny;
                    path_index++;

                    // 保存当前坐标再往前倒推一层
                    uint8_t px = (uint8_t)nx;
                    uint8_t py = (uint8_t)ny;
                    nx = bfs_prev_x[py][px];
                    ny = bfs_prev_y[py][px];
                }

                // 把起点也加进去
                bfs_path[path_index].x = sx;
                bfs_path[path_index].y = sy;
                path_index++;

                // 现在数组里是“终点 -> 起点”，反转成“起点 -> 终点”
                for (i = 0; i < path_index / 2; i++) {
                    tmp = bfs_path[i];
                    bfs_path[i] = bfs_path[path_index - 1 - i];
                    bfs_path[path_index - 1 - i] = tmp;
                }

                bfs_path_len = path_index;
                return 1;
            }

            // 还没到终点：邻居入队，等下一层扩展  先入队的点先搜
            bfs_queue[q_tail].x = (uint8_t)nx;
            bfs_queue[q_tail].y = (uint8_t)ny;
            q_tail++;   // 入队永远发生在队尾（q_tail）
        }
    }

    // 队列搜完还没到终点，说明被障碍挡住了
    bfs_path_len = 0;
    return 0;
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
