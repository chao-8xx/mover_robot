#include "headfile.h"

/**
 * 根据新观测更新障碍物和道路状态
 */


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


// 根据起点和终点搜索最短路径  允许或不允许未知道路   Dijkstra 最短路径
uint8_t map_plan_find(uint8_t start, uint8_t goal, map_unknown_mode_t mode, map_path_t *path) {
    uint16_t cost[MAP_NODE_COUNT];              // 到每个节点的当前最短代价
    uint8_t  done[MAP_NODE_COUNT] = {0};        // 是否已确定最短
    uint8_t  prev_node[MAP_NODE_COUNT];         // 最短路径的前驱节点
    uint8_t  prev_road[MAP_NODE_COUNT];         // 到达该节点所走的道路 ID
    uint8_t reverse_node[MAP_NODE_COUNT];       // 反向存储路径节点
    uint8_t reverse_road[MAP_NODE_COUNT - 1];   // 反向存储路径道路
    uint8_t count = 0;

    // 参数检查
    if (path == 0)
        return 0;

    // 初始化 清空路径
    memset(path, 0, sizeof(*path));

    if (start >= MAP_NODE_COUNT || goal >= MAP_NODE_COUNT || (mode != map_unknown_allow && mode != map_unknown_stop))
        return 0;

    for (uint8_t i = 0; i < MAP_NODE_COUNT; i++) {
        cost[i] = MAP_COST_INF;
        prev_node[i] = MAP_NO_ID;
        prev_road[i] = MAP_NO_ID;
    }
    cost[start] = 0;

    // 只有九个节点，直接逐轮选择代价最小的节点
    for (uint8_t step = 0; step < MAP_NODE_COUNT; step++) {
        uint8_t node = MAP_NO_ID;
        for (uint8_t i = 0; i < MAP_NODE_COUNT; i++) {
            if (!done[i] && cost[i] != MAP_COST_INF &&
                (node == MAP_NO_ID || cost[i] < cost[node])) node = i;
        }
        if (node == MAP_NO_ID || node == goal) break;
        done[node] = 1;

        for (uint8_t road_id = 0; road_id < MAP_ROAD_COUNT; road_id++) {
            map_road_t road;
            uint8_t next;
            uint16_t road_cost;
            if (!map_road_info_get(road_id, &road) ||
                !map_road_cost(road_id, mode, &road_cost)) continue;
            if (road.node_a == node) next = road.node_b;
            else if (road.node_b == node) next = road.node_a;
            else continue;
            if (done[next]) continue;
            if (cost[node] + road_cost < cost[next]) {
                cost[next] = cost[node] + road_cost;
                prev_node[next] = node;
                prev_road[next] = road_id;
            }
        }
    }

    if (cost[goal] == MAP_COST_INF) return 0;

    // 从终点倒推，再反转成起点到终点
    for (uint8_t node = goal; count < MAP_NODE_COUNT; node = prev_node[node]) {
        reverse_node[count] = node;
        if (node == start) break;
        if (prev_node[node] == MAP_NO_ID) return 0;
        reverse_road[count] = prev_road[node];
        count++;
    }
    path->node_count = count + 1;
    path->road_count = count;
    for (uint8_t i = 0; i < path->node_count; i++)
        path->nodes[i] = reverse_node[count - i];
    for (uint8_t i = 0; i < path->road_count; i++)
        path->roads[i] = reverse_road[count - 1 - i];
    return 1;
}

uint8_t map_plan_check(const map_path_t *path, map_unknown_mode_t mode) {
    if (path == 0 || path->node_count == 0 ||
        path->node_count > MAP_NODE_COUNT ||
        path->road_count + 1 != path->node_count ||
        (mode != map_unknown_allow && mode != map_unknown_stop)) return 0;

    // 单节点路径没有道路可供下面的循环检查，也必须保证节点编号有效。
    for (uint8_t i = 0; i < path->node_count; i++) {
        if (path->nodes[i] >= MAP_NODE_COUNT) return 0;
    }

    for (uint8_t i = 0; i < path->road_count; i++) {
        uint8_t road_id;
        uint16_t cost;
        if (!map_road_find(path->nodes[i], path->nodes[i + 1], &road_id) ||
            road_id != path->roads[i] || !map_road_cost(road_id, mode, &cost)) return 0;
    }
    return 1;
}

map_step_t map_plan_road_state(const map_path_t *path, uint8_t road_index) {
    uint8_t road_id;
    road_state_t state;
    if (path == 0 || road_index >= path->road_count ||
        path->node_count > MAP_NODE_COUNT ||
        path->road_count + 1 != path->node_count ||
        !map_road_find(path->nodes[road_index], path->nodes[road_index + 1], &road_id) ||
        road_id != path->roads[road_index] ||
        !map_road_get(road_id, &state)) return map_step_stop;

    if (state == road_unknown) return map_step_scan;
    if (state == road_free) return map_step_go;
    return map_step_stop;
}
