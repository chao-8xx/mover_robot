// 首先包含规划头文件  检查它能否独立使用
#include "map_plan.h"
#include "map_data.h"
#include "map_update.h"
#include <stdio.h>
#include <string.h>

static unsigned failed;
static unsigned route_cases;

// 固定赛道的道路端点  独立于规划实现用于核对结果
static const uint8_t test_edges[12][2] = {
    {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
    {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}
};

static void test_check(uint8_t ok, const char *name)
{
    printf("[PLAN TEST] %s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) failed++;
}

static void test_roads_set(road_state_t state)
{
    map_data_init();
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++)
        map_road_set(i, state);
}

static uint16_t test_path_cost(const map_path_t *path)
{
    uint16_t cost = 0;
    road_state_t state;
    for (uint8_t i = 0; i < path->road_count; i++)
    {
        if (!map_road_get(path->roads[i], &state) || state == road_blocked)
            return MAP_COST_INF;
        cost += state == road_free ? 1 : 2;
    }
    return cost;
}

// Floyd全点对算法作为独立参考  不使用Dijkstra的选点或前驱逻辑
static uint8_t test_all_pairs(const road_state_t *states, map_unknown_mode_t mode)
{
    uint16_t distance[MAP_NODE_COUNT][MAP_NODE_COUNT];
    map_path_t empty = {0};

    for (uint8_t a = 0; a < MAP_NODE_COUNT; a++)
        for (uint8_t b = 0; b < MAP_NODE_COUNT; b++)
            distance[a][b] = a == b ? 0 : MAP_COST_INF;

    for (uint8_t r = 0; r < MAP_ROAD_COUNT; r++)
    {
        if (states[r] == road_blocked || (states[r] == road_unknown && mode == map_unknown_stop))
            continue;
        uint8_t a = test_edges[r][0];
        uint8_t b = test_edges[r][1];
        distance[a][b] = distance[b][a] = states[r] == road_free ? 1 : 2;
    }

    for (uint8_t k = 0; k < MAP_NODE_COUNT; k++)
        for (uint8_t a = 0; a < MAP_NODE_COUNT; a++)
            for (uint8_t b = 0; b < MAP_NODE_COUNT; b++)
                if (distance[a][k] != MAP_COST_INF && distance[k][b] != MAP_COST_INF &&
                    distance[a][k] + distance[k][b] < distance[a][b])
                    distance[a][b] = distance[a][k] + distance[k][b];

    for (uint8_t start = 0; start < MAP_NODE_COUNT; start++)
    {
        for (uint8_t goal = 0; goal < MAP_NODE_COUNT; goal++)
        {
            map_path_t path;
            uint16_t visited = 0;
            uint16_t cost = 0;
            uint8_t found;

            route_cases++;
            memset(&path, 0xAA, sizeof(path));
            found = map_plan_find(start, goal, mode, &path);
            if (found != (distance[start][goal] != MAP_COST_INF)) return 0;
            if (!found)
            {
                if (memcmp(&path, &empty, sizeof(path)) != 0) return 0;
                continue;
            }
            if (path.node_count == 0 || path.node_count > MAP_NODE_COUNT ||
                path.road_count + 1 != path.node_count ||
                path.nodes[0] != start || path.nodes[path.node_count - 1] != goal)
                return 0;

            for (uint8_t i = 0; i < path.node_count; i++)
            {
                uint8_t node = path.nodes[i];
                if (node >= MAP_NODE_COUNT || (visited & (1U << node))) return 0;
                visited |= 1U << node;
            }
            for (uint8_t i = 0; i < path.road_count; i++)
            {
                uint8_t road = path.roads[i];
                uint8_t a = path.nodes[i];
                uint8_t b = path.nodes[i + 1];
                if (road >= MAP_ROAD_COUNT || states[road] == road_blocked ||
                    (states[road] == road_unknown && mode == map_unknown_stop))
                    return 0;
                if (!((test_edges[road][0] == a && test_edges[road][1] == b) ||
                      (test_edges[road][0] == b && test_edges[road][1] == a)))
                    return 0;
                cost += states[road] == road_free ? 1 : 2;
            }
            if (cost != distance[start][goal] || !map_plan_check(&path, mode))
            {
                printf("start=N%u goal=N%u mode=%u expected=%u actual=%u\n",
                       start, goal, (unsigned)mode, distance[start][goal], cost);
                return 0;
            }
        }
    }
    return 1;
}

static void test_examples(void)
{
    map_path_t path, old_path;
    map_path_t empty = {0};
    const uint8_t snake_roads[] = {0, 1, 10, 3, 2, 7, 4, 5};
    const uint8_t free_detour[] = {6, 7, 4, 5, 11};

    test_roads_set(road_free);
    test_check(map_plan_find(1, 4, map_unknown_stop, &path) &&
               path.node_count == 2 && path.roads[0] == 8 && test_path_cost(&path) == 1,
               "N1 -> N4 direct R8");
    old_path = path;
    map_road_set(8, road_blocked);
    test_check(!map_plan_check(&old_path, map_unknown_stop) &&
               map_plan_find(1, 4, map_unknown_stop, &path) &&
               path.node_count == 4 && path.nodes[1] == 0 && path.nodes[2] == 3 &&
               test_path_cost(&path) == 3,
               "block R8 -> N1 N0 N3 N4");
    map_road_set(0, road_blocked);
    map_road_set(1, road_blocked);
    test_check(!map_plan_find(1, 4, map_unknown_allow, &path) &&
               memcmp(&path, &empty, sizeof(path)) == 0,
               "all exits blocked clears old path");
    test_check(map_plan_find(1, 1, map_unknown_stop, &path) &&
               path.node_count == 1 && path.road_count == 0 && path.nodes[0] == 1,
               "start equals goal even without exits");

    test_roads_set(road_unknown);
    test_check(!map_plan_find(1, 4, map_unknown_stop, &path) &&
               map_plan_find(1, 4, map_unknown_allow, &path) && test_path_cost(&path) == 2,
               "unknown policy and cost 2");

    // 只开放外围两条路线  3条未知路代价6  5条已知路代价5
    test_roads_set(road_blocked);
    for (uint8_t i = 0; i < sizeof(free_detour); i++)
        map_road_set(free_detour[i], road_free);
    map_road_set(0, road_unknown);
    map_road_set(1, road_unknown);
    map_road_set(10, road_unknown);
    test_check(map_plan_find(0, 5, map_unknown_allow, &path) &&
               path.road_count == 5 && test_path_cost(&path) == 5,
               "weighted route prefers 5 free roads over 3 unknown roads");

    test_roads_set(road_blocked);
    for (uint8_t i = 0; i < sizeof(snake_roads); i++)
        map_road_set(snake_roads[i], road_free);
    test_check(map_plan_find(0, 8, map_unknown_stop, &path) &&
               path.node_count == MAP_NODE_COUNT && path.road_count == MAP_NODE_COUNT - 1 &&
               test_path_cost(&path) == 8 && map_plan_check(&path, map_unknown_stop),
               "maximum path uses all 9 nodes");

    test_check(!map_plan_find(0, 8, map_unknown_stop, NULL), "null output rejected");
    test_check(!map_plan_find(MAP_NODE_COUNT, 8, map_unknown_stop, &path) &&
               memcmp(&path, &empty, sizeof(path)) == 0,
               "invalid start clears path");
    memset(&path, 0xAA, sizeof(path));
    test_check(!map_plan_find(0, 255, map_unknown_stop, &path) &&
               memcmp(&path, &empty, sizeof(path)) == 0,
               "invalid goal clears path");
    memset(&path, 0xAA, sizeof(path));
    test_check(!map_plan_find(0, 8, (map_unknown_mode_t)2, &path) &&
               memcmp(&path, &empty, sizeof(path)) == 0,
               "invalid policy clears path");

    test_roads_set(road_free);
    map_plan_find(1, 4, map_unknown_stop, &path);
    path.roads[0] = 0;
    test_check(!map_plan_check(&path, map_unknown_stop) &&
               map_plan_road_state(&path, 0, 100) == map_step_stop,
               "road and node mismatch rejected");
    path.node_count = MAP_NODE_COUNT + 1;
    test_check(!map_plan_check(&path, map_unknown_stop) &&
               map_plan_road_state(&path, 0, 100) == map_step_stop,
               "oversized path rejected");
    memset(&path, 0, sizeof(path));
    path.node_count = 1;
    path.nodes[0] = 255;
    test_check(!map_plan_check(&path, map_unknown_stop) &&
               !map_plan_check(NULL, map_unknown_stop), "invalid single node and null path rejected");
}

static void test_entry_permission(void)
{
    map_path_t path;
    map_scan_obs_t box = {1200.0f, 1625.0f, 1190.0f, 1615.0f, 1210.0f, 1635.0f};

    map_update_init();
    map_plan_find(1, 4, map_unknown_allow, &path);
    test_check(map_plan_road_state(&path, 0, 100) == map_step_scan,
               "unknown candidate still requires SCAN");
    map_road_set(8, road_free);
    test_check(map_plan_road_state(&path, 0, 100) == map_step_scan,
               "stored FREE alone cannot GO");
    map_update_scan(NULL, 0, 1, 100, 1);
    test_check(map_update_confirm_free(8, 1, 101) &&
               map_plan_road_state(&path, 0, 102) == map_step_go &&
               map_plan_road_state(&path, 0, 1101) == map_step_scan,
               "fresh permission GO and expired permission SCAN");
    map_update_scan(NULL, 0, 2, 200, 1);
    test_check(map_plan_road_state(&path, 0, 201) == map_step_scan &&
               map_update_confirm_free(8, 2, 201) &&
               map_plan_road_state(&path, 0, 202) == map_step_go,
               "new scan requires renewed permission");
    map_update_scan(&box, 1, 3, 300, 1);
    test_check(map_plan_road_state(&path, 0, 301) == map_step_scan,
               "first obstacle observation suspends GO");
    map_update_scan(&box, 1, 4, 400, 1);
    test_check(map_plan_road_state(&path, 0, 401) == map_step_stop &&
               !map_plan_check(&path, map_unknown_allow),
               "confirmed obstacle stops old path");
    test_check(map_plan_road_state(NULL, 0, 401) == map_step_stop &&
               map_plan_road_state(&path, 255, 401) == map_step_stop,
               "invalid entry request stops");
}

int main(void)
{
    road_state_t states[MAP_ROAD_COUNT];
    uint32_t seed = 20261021U;
    uint8_t all_ok = 1;

    test_check(MAP_NODE_COUNT == 9 && MAP_ROAD_COUNT == 12 &&
               MAP_COST_FREE == 1 && MAP_COST_UNKNOWN == 2, "fixed topology and weight contract");
    test_examples();
    test_entry_permission();

    // 遍历12条道路全部4096种封锁组合  每种比较81对起终点
    for (unsigned mask = 0; mask < (1U << MAP_ROAD_COUNT); mask++)
    {
        for (uint8_t r = 0; r < MAP_ROAD_COUNT; r++)
        {
            states[r] = mask & (1U << r) ? road_blocked : road_free;
            map_road_set(r, states[r]);
        }
        if (!test_all_pairs(states, map_unknown_stop))
        {
            printf("blocked mask=%u\n", mask);
            all_ok = 0;
            break;
        }
    }
    test_check(all_ok, "all 4096 blocked-road combinations match Floyd");

    // 固定种子生成128组未知/已知/封锁混合地图  两种未知策略分别核对
    all_ok = 1;
    for (unsigned sample = 0; sample < 128; sample++)
    {
        for (uint8_t r = 0; r < MAP_ROAD_COUNT; r++)
        {
            seed = seed * 1664525U + 1013904223U;
            states[r] = (road_state_t)((seed >> 16) % 3);
            map_road_set(r, states[r]);
        }
        if (!test_all_pairs(states, map_unknown_stop) || !test_all_pairs(states, map_unknown_allow))
        {
            printf("mixed sample=%u\n", sample);
            all_ok = 0;
            break;
        }
    }
    test_check(all_ok, "128 mixed maps and both unknown policies match Floyd");
    printf("map plan: %u route comparisons, %u failures\n", route_cases, failed);
    return failed ? 1 : 0;
}
