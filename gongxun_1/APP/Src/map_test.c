#include "headfile.h"


static void map_planner_test_reset(void)
{
    occupancy_grid_init();
    track_map_init();
    (void)track_map_set_all_road_states(ROAD_FREE);
}

static void map_planner_test_print_path(const uint8_t *path, uint8_t path_len)
{
    uint8_t path_index;

    for (path_index = 0U; path_index < path_len; path_index++)
    {
        printf("N%u", (unsigned int)path[path_index]);
        if (path_index + 1U < path_len)
        {
            printf(" -> ");
        }
    }
    printf("\r\n");
}

static bool map_planner_test_path_is_valid(const uint8_t *path,
                                           uint8_t path_len,
                                           uint8_t start_node,
                                           uint8_t goal_node)
{
    uint8_t path_index;

    if (path == NULL || path_len == 0U ||
        path[0] != start_node || path[path_len - 1U] != goal_node)
    {
        return false;
    }

    for (path_index = 0U; path_index + 1U < path_len; path_index++)
    {
        int8_t road_id = track_map_find_road(path[path_index], path[path_index + 1U]);

        if (road_id < 0 || track_map_get_road_state((uint8_t)road_id) != ROAD_FREE)
        {
            return false;
        }
    }

    return true;
}

static bool map_planner_test_inject_obstacle(uint8_t grid_x,
                                             uint8_t grid_y,
                                             uint16_t *blocked_mask)
{
    uint16_t road_mask;

    if (!occupancy_grid_set_state(grid_x, grid_y, GRID_CELL_OCCUPIED))
    {
        return false;
    }

    road_mask = track_map_grid_to_road_mask(grid_x, grid_y);
    if (road_mask == 0U ||
        !track_map_set_road_mask_state(road_mask, ROAD_BLOCKED))
    {
        return false;
    }

    if (blocked_mask != NULL)
    {
        *blocked_mask = road_mask;
    }
    return true;
}

static bool map_planner_test_macro_mapping(void)
{
    static const uint8_t macro_grid_samples[TRACK_MACRO_WIDTH] =
    {
        5U, 15U, 24U, 32U, 42U
    };
    uint8_t macro_x;
    uint8_t macro_y;
    uint8_t grid_x;
    uint8_t grid_y;
    uint8_t node_count = 0U;
    uint8_t road_count = 0U;
    uint8_t forbidden_count = 0U;
    uint16_t seen_node_mask = 0U;
    uint16_t seen_road_mask = 0U;
    uint16_t n4_mask = TRACK_ROAD_MASK(TRACK_ROAD_R2) |
                       TRACK_ROAD_MASK(TRACK_ROAD_R3) |
                       TRACK_ROAD_MASK(TRACK_ROAD_R8) |
                       TRACK_ROAD_MASK(TRACK_ROAD_R9);

    map_planner_test_reset();

    if (!occupancy_grid_world_to_cell(0, 0, &grid_x, &grid_y) ||
        grid_x != 0U || grid_y != 0U ||
        !occupancy_grid_world_to_cell(2399, 2399, &grid_x, &grid_y) ||
        grid_x != 47U || grid_y != 47U ||
        occupancy_grid_world_to_cell(2400, 1200, &grid_x, &grid_y))
    {
        return false;
    }

    for (macro_y = 0U; macro_y < TRACK_MACRO_HEIGHT; macro_y++)
    {
        for (macro_x = 0U; macro_x < TRACK_MACRO_WIDTH; macro_x++)
        {
            const TrackMacroCell *macro_cell = track_map_get_macro_cell(macro_x, macro_y);
            uint16_t mapped_road_mask = track_map_grid_to_road_mask(
                macro_grid_samples[macro_x],
                macro_grid_samples[macro_y]);

            if (macro_cell == NULL)
            {
                return false;
            }

            if (macro_cell->type == TRACK_MACRO_NODE)
            {
                const TrackNode *node = track_map_get_node(macro_cell->id);
                uint16_t node_bit;

                if (node == NULL)
                {
                    return false;
                }

                node_bit = (uint16_t)(1U << macro_cell->id);
                if ((seen_node_mask & node_bit) != 0U ||
                    mapped_road_mask != node->observation_mask)
                {
                    return false;
                }

                seen_node_mask |= node_bit;
                node_count++;
            }
            else if (macro_cell->type == TRACK_MACRO_ROAD)
            {
                const TrackRoad *road = track_map_get_road(macro_cell->id);
                uint16_t road_bit;

                if (road == NULL || road->length_mm != 925U)
                {
                    return false;
                }

                road_bit = TRACK_ROAD_MASK(macro_cell->id);
                if ((seen_road_mask & road_bit) != 0U ||
                    mapped_road_mask != road_bit)
                {
                    return false;
                }

                seen_road_mask |= road_bit;
                road_count++;
            }
            else if (macro_cell->type == TRACK_MACRO_FORBIDDEN)
            {
                if (mapped_road_mask != 0U)
                {
                    return false;
                }
                forbidden_count++;
            }
            else
            {
                return false;
            }
        }
    }

    if (node_count != TRACK_NODE_COUNT ||
        road_count != TRACK_ROAD_COUNT ||
        forbidden_count != 4U ||
        seen_node_mask != (uint16_t)((1U << TRACK_NODE_COUNT) - 1U) ||
        seen_road_mask != TRACK_ALL_ROADS_MASK)
    {
        return false;
    }

    if (!track_map_grid_to_macro(24U, 32U, &macro_x, &macro_y) ||
        macro_x != 2U || macro_y != 3U ||
        track_map_grid_to_road(24U, 32U) != TRACK_ROAD_R8 ||
        track_map_grid_to_road_mask(24U, 32U) != TRACK_ROAD_MASK(TRACK_ROAD_R8))
    {
        return false;
    }

    if (track_map_grid_to_road_mask(24U, 24U) != n4_mask)
    {
        return false;
    }

    if (track_map_grid_to_road_mask(15U, 15U) != 0U)
    {
        return false;
    }

    return true;
}

static bool map_planner_test_all_free(void)
{
    uint8_t path[PLANNER_MAX_PATH_NODES];
    uint8_t path_len;

    map_planner_test_reset();
    if (!planner_find_path(TRACK_NODE_N0,
                           TRACK_NODE_N8,
                           path,
                           sizeof(path),
                           &path_len) ||
        !map_planner_test_path_is_valid(path,
                                        path_len,
                                        TRACK_NODE_N0,
                                        TRACK_NODE_N8))
    {
        return false;
    }

    printf("  all free: ");
    map_planner_test_print_path(path, path_len);
    return true;
}

static bool map_planner_test_block_r8(void)
{
    uint8_t path[PLANNER_MAX_PATH_NODES];
    uint8_t path_len;
    uint16_t blocked_mask;

    map_planner_test_reset();
    if (!map_planner_test_inject_obstacle(24U, 32U, &blocked_mask) ||
        blocked_mask != TRACK_ROAD_MASK(TRACK_ROAD_R8) ||
        !planner_find_path(TRACK_NODE_N1,
                           TRACK_NODE_N7,
                           path,
                           sizeof(path),
                           &path_len) ||
        !map_planner_test_path_is_valid(path,
                                        path_len,
                                        TRACK_NODE_N1,
                                        TRACK_NODE_N7) ||
        planner_path_uses_road(path, path_len, TRACK_ROAD_R8))
    {
        return false;
    }

    printf("  R8 blocked: ");
    map_planner_test_print_path(path, path_len);
    return true;
}

static bool map_planner_test_multiple_blocks(void)
{
    uint8_t path[PLANNER_MAX_PATH_NODES];
    uint8_t path_len;

    map_planner_test_reset();
    if (!map_planner_test_inject_obstacle(24U, 32U, NULL) ||
        !map_planner_test_inject_obstacle(32U, 42U, NULL) ||
        !map_planner_test_inject_obstacle(42U, 32U, NULL) ||
        !planner_find_path(TRACK_NODE_N0,
                           TRACK_NODE_N8,
                           path,
                           sizeof(path),
                           &path_len) ||
        !map_planner_test_path_is_valid(path,
                                        path_len,
                                        TRACK_NODE_N0,
                                        TRACK_NODE_N8))
    {
        return false;
    }

    printf("  R1/R8/R10 blocked: ");
    map_planner_test_print_path(path, path_len);
    return true;
}

static bool map_planner_test_no_path(void)
{
    uint8_t path[PLANNER_MAX_PATH_NODES];
    uint8_t path_len = 0xFFU;

    map_planner_test_reset();
    if (!map_planner_test_inject_obstacle(15U, 42U, NULL) ||
        !map_planner_test_inject_obstacle(5U, 32U, NULL))
    {
        return false;
    }

    return !planner_find_path(TRACK_NODE_N0,
                              TRACK_NODE_N8,
                              path,
                              sizeof(path),
                              &path_len) &&
           path_len == 0U;
}

static bool map_planner_test_start_equals_goal(void)
{
    uint8_t path[PLANNER_MAX_PATH_NODES];
    uint8_t path_len;

    map_planner_test_reset();
    return planner_find_path(TRACK_NODE_N4,
                             TRACK_NODE_N4,
                             path,
                             sizeof(path),
                             &path_len) &&
           path_len == 1U &&
           path[0] == TRACK_NODE_N4;
}

static bool map_planner_test_dynamic_replan(void)
{
    PlannerContext context;
    uint8_t old_path[PLANNER_MAX_PATH_NODES];
    uint8_t new_path[PLANNER_MAX_PATH_NODES];
    uint8_t old_path_len;
    uint8_t new_path_len;

    map_planner_test_reset();
    if (!planner_context_init(&context, TRACK_NODE_N1, TRACK_NODE_N7) ||
        !planner_find_path(context.current_node,
                           context.target_node,
                           old_path,
                           sizeof(old_path),
                           &old_path_len) ||
        !planner_path_uses_road(old_path, old_path_len, TRACK_ROAD_R8) ||
        !map_planner_test_inject_obstacle(24U, 32U, NULL))
    {
        return false;
    }

    planner_context_notify_road_state(&context,
                                      TRACK_ROAD_R8,
                                      ROAD_BLOCKED,
                                      old_path,
                                      old_path_len);
    if (!context.need_replan ||
        !planner_find_path(context.current_node,
                           context.target_node,
                           new_path,
                           sizeof(new_path),
                           &new_path_len) ||
        !map_planner_test_path_is_valid(new_path,
                                        new_path_len,
                                        context.current_node,
                                        context.target_node) ||
        planner_path_uses_road(new_path, new_path_len, TRACK_ROAD_R8))
    {
        return false;
    }

    printf("  before block: ");
    map_planner_test_print_path(old_path, old_path_len);
    printf("  after block:  ");
    map_planner_test_print_path(new_path, new_path_len);
    planner_context_clear_replan(&context);
    return !context.need_replan;
}

bool map_planner_test_run(void)
{
    static const struct
    {
        const char *name;
        bool (*run)(void);
    } tests[] =
    {
        {"grid to macro/road mapping", map_planner_test_macro_mapping},
        {"all roads free", map_planner_test_all_free},
        {"block R8", map_planner_test_block_r8},
        {"block multiple roads", map_planner_test_multiple_blocks},
        {"no path", map_planner_test_no_path},
        {"start equals goal", map_planner_test_start_equals_goal},
        {"dynamic replan", map_planner_test_dynamic_replan}
    };
    uint8_t test_index;
    uint8_t failures = 0U;

    printf("=== Map/Planner tests ===\r\n");
    for (test_index = 0U; test_index < (uint8_t)(sizeof(tests) / sizeof(tests[0])); test_index++)
    {
        bool passed = tests[test_index].run();

        printf("[%s] %s\r\n", passed ? "PASS" : "FAIL", tests[test_index].name);
        if (!passed)
        {
            failures++;
        }
    }

    printf("Result: %u passed, %u failed\r\n",
           (unsigned int)((sizeof(tests) / sizeof(tests[0])) - failures),
           failures);
    return failures == 0U;
}

#ifdef MAP_PLANNER_TEST_MAIN
int main(void)
{
    return map_planner_test_run() ? 0 : 1;
}
#endif
