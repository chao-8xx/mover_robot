#include "headfile.h"

/**
 * 9节点12道路的 Dijkstra 路径搜索
 * 读取已有道路状态  已知路代价1  未知道路代价2  封锁道路不参与搜索
 * 只输出路径  不更新地图、不控制车辆  候选路径进入前仍需检查通行许可
 */


///////////////////////////////////// 内部调用函数 ////////////////////////////////////////
// 判断道路是否允许搜索  并给出道路代价
static uint8_t map_road_cost(uint8_t road_id, map_unknown_mode_t mode, uint16_t *cost) {
    road_state_t state;

    // 道路不存在
    if (!map_road_get(road_id, &state))
        return 0;

    // 道路被阻塞
    if (state == road_blocked)
        return 0;

    // 道路未知状态
    if (state == road_unknown) {
        if (mode != map_unknown_allow)
            return 0;
        *cost = MAP_COST_UNKNOWN;
    }

    // 道路可通行
    else if (state == road_free) {
        *cost = MAP_COST_FREE;
    }
    else
        return 0;

    return 1;
}


///////////////////////////////////// Dijkstra路径搜索 ////////////////////////////////////////
// 根据起点、终点和未知路策略搜索最小代价路径
uint8_t map_plan_find(uint8_t start, uint8_t goal, map_unknown_mode_t mode, map_path_t *path) {
    uint16_t cost[MAP_NODE_COUNT];              // 到每个节点的当前最短代价
    uint8_t  done[MAP_NODE_COUNT] = {0};        // 是否已确定最短
    uint8_t  prev_node[MAP_NODE_COUNT];         // 最短路径的前驱节点
    uint8_t  prev_road[MAP_NODE_COUNT];         // 到达该节点所走的道路 ID
    uint8_t reverse_node[MAP_NODE_COUNT];       // 反向存储路径节点
    uint8_t reverse_road[MAP_NODE_COUNT - 1];   // 反向存储路径道路
    uint8_t count = 0;

    // 参数检查  先清空输出 防止本次失败后继续使用旧路径
    if (path == 0)
        return 0;

    // 初始化 清空路径
    memset(path, 0, sizeof(*path));

    if (start >= MAP_NODE_COUNT || goal >= MAP_NODE_COUNT || (mode != map_unknown_allow && mode != map_unknown_stop))
        return 0;

    // 初始化代价和前驱  INF表示暂时不可达  不代表道路被封锁
    for (uint8_t i = 0; i < MAP_NODE_COUNT; i++) {
        cost[i] = MAP_COST_INF;
        prev_node[i] = MAP_NO_ID;
        prev_road[i] = MAP_NO_ID;
    }
    cost[start] = 0;

    // 逐轮确定一个节点的最小代价  只有9个节点  直接遍历选点
    for (uint8_t step = 0; step < MAP_NODE_COUNT; step++) {
        uint8_t node = MAP_NO_ID;

        // 选择尚未确定且代价最小的节点  代价相同时保留先找到的小编号节点
        for (uint8_t i = 0; i < MAP_NODE_COUNT; i++) {
            if (!done[i] && cost[i] != MAP_COST_INF &&
                (node == MAP_NO_ID || cost[i] < cost[node]))
                node = i;
        }
        // 没有剩余可达节点 或终点已是当前最小代价时 结束搜索
        if (node == MAP_NO_ID || node == goal)
            break;
        done[node] = 1;

        // 遍历所有道路  找到与当前节点相连的道路
        for (uint8_t road_id = 0; road_id < MAP_ROAD_COUNT; road_id++) {
            map_road_t road;
            uint8_t next;
            uint16_t road_cost;

            // 道路不存在 或 道路不允许通行  跳过
            if (!map_road_info_get(road_id, &road) ||
                !map_road_cost(road_id, mode, &road_cost))
                continue;

            // 找到与当前节点相连的道路  获取下一个节点
            if (road.node_a == node)
                next = road.node_b;
            else if (road.node_b == node)
                next = road.node_a;
            else
                continue;
            if (next >= MAP_NODE_COUNT || done[next])
                continue;

            // 松弛相邻节点  经当前节点绕过去更便宜时 更新代价和前驱
            if (cost[node] + road_cost < cost[next]) {
                cost[next] = cost[node] + road_cost;
                prev_node[next] = node;
                prev_road[next] = road_id;
            }
        }
    }

    if (cost[goal] == MAP_COST_INF)
        return 0;

    // 从终点按前驱倒推  起点没有前驱 所以先检查是否已回到起点
    for (uint8_t node = goal; count < MAP_NODE_COUNT; node = prev_node[node]) {
        reverse_node[count] = node;

        if (node == start)
            break;

        // 最多保存9个节点和8条道路  前驱异常或容量耗尽时不能继续写入
        if (count >= MAP_NODE_COUNT - 1 ||
            prev_node[node] >= MAP_NODE_COUNT || prev_road[node] >= MAP_ROAD_COUNT)
            return 0;

        reverse_road[count] = prev_road[node];
        count++;
    }
    path->node_count = count + 1;
    path->road_count = count;

    // 反向结果写入输出  变成起点到终点的节点和道路顺序
    for (uint8_t i = 0; i < path->node_count; i++)
        path->nodes[i] = reverse_node[count - i];

    for (uint8_t i = 0; i < path->road_count; i++)
        path->roads[i] = reverse_road[count - 1 - i];

    return 1;
}

///////////////////////////////////// 路径检查与进路决策 ////////////////////////////////////////
// 检查路径连接和道路状态  不重新搜索最优路径
uint8_t map_plan_check(const map_path_t *path, map_unknown_mode_t mode) {
    if (path == 0 || path->node_count == 0 ||
        path->node_count > MAP_NODE_COUNT ||
        path->road_count + 1 != path->node_count ||
        (mode != map_unknown_allow && mode != map_unknown_stop))
        return 0;

    // 单节点路径没有道路可供下面的循环检查  也必须保证节点编号有效
    for (uint8_t i = 0; i < path->node_count; i++) {
        if (path->nodes[i] >= MAP_NODE_COUNT)
            return 0;
    }

    // 检查每条道路是否存在  是否允许通行  道路编号是否与路径中的道路编号一致
    for (uint8_t i = 0; i < path->road_count; i++) {
        uint8_t road_id;
        uint16_t cost;
        if (!map_road_find(path->nodes[i], path->nodes[i + 1], &road_id) ||
            road_id != path->roads[i] || !map_road_cost(road_id, mode, &cost))
            return 0;
    }

    return 1;
}

// 进入某条道路前的决策   根据路径和道路索引 返回该道路的状态
map_step_t map_plan_road_state(const map_path_t *path, uint8_t road_index, uint32_t tick) {
    uint8_t road_id;
    road_state_t state;

    // 检查路径和道路索引的合法性  道路是否存在  道路状态是否可用
    if (path == 0 || road_index >= path->road_count ||
        path->node_count > MAP_NODE_COUNT ||
        path->road_count + 1 != path->node_count ||
        !map_road_find(path->nodes[road_index], path->nodes[road_index + 1], &road_id) ||
        road_id != path->roads[road_index] ||
        !map_road_get(road_id, &state))
        return map_step_stop;

    if (state == road_unknown)
        return map_step_scan;

    if (state == road_free)
        return map_update_road_ready(road_id, tick) ? map_step_go : map_step_scan;

    return map_step_stop;
}
