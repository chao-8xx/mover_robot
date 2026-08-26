#include "planner.h"

#include <stddef.h>

static bool planner_policy_is_valid(PlannerRoadPolicy policy)
{
    return policy == PLANNER_POLICY_FREE_ONLY ||
           policy == PLANNER_POLICY_ALLOW_UNKNOWN;
}

static bool planner_road_is_traversable(RoadState state,
                                        PlannerRoadPolicy policy)
{
    if (state == ROAD_FREE)
    {
        return true;
    }

    return state == ROAD_UNKNOWN && policy == PLANNER_POLICY_ALLOW_UNKNOWN;
}

bool planner_find_path(uint8_t start_node,
                       uint8_t goal_node,
                       uint8_t *path,
                       uint8_t path_capacity,
                       uint8_t *path_len)
{
    return planner_find_path_with_policy(start_node,
                                         goal_node,
                                         PLANNER_POLICY_FREE_ONLY,
                                         path,
                                         path_capacity,
                                         path_len);
}

bool planner_find_path_with_policy(uint8_t start_node,
                                   uint8_t goal_node,
                                   PlannerRoadPolicy policy,
                                   uint8_t *path,
                                   uint8_t path_capacity,
                                   uint8_t *path_len)
{
    uint8_t queue[TRACK_NODE_COUNT];
    bool visited[TRACK_NODE_COUNT] = {false};
    int8_t parent[TRACK_NODE_COUNT];
    uint8_t reverse_path[TRACK_NODE_COUNT];
    uint8_t queue_head = 0U;
    uint8_t queue_tail = 0U;
    uint8_t reverse_len = 0U;
    uint8_t node_id;
    uint8_t road_id;

    if (path_len == NULL)
    {
        return false;
    }
    *path_len = 0U;

    if (path == NULL || path_capacity == 0U ||
        start_node >= TRACK_NODE_COUNT || goal_node >= TRACK_NODE_COUNT ||
        !planner_policy_is_valid(policy))
    {
        return false;
    }

    if (start_node == goal_node)
    {
        path[0] = start_node;
        *path_len = 1U;
        return true;
    }

    for (node_id = 0U; node_id < TRACK_NODE_COUNT; node_id++)
    {
        parent[node_id] = -1;
    }

    visited[start_node] = true;
    queue[queue_tail++] = start_node;

    while (queue_head < queue_tail && !visited[goal_node])
    {
        uint8_t current_node = queue[queue_head++];

        for (road_id = 0U; road_id < TRACK_ROAD_COUNT; road_id++)
        {
            const TrackRoad *road = track_map_get_road(road_id);
            uint8_t neighbor;

            if (road == NULL ||
                !planner_road_is_traversable(road->state, policy))
            {
                continue;
            }

            if (road->node_a == current_node)
            {
                neighbor = road->node_b;
            }
            else if (road->node_b == current_node)
            {
                neighbor = road->node_a;
            }
            else
            {
                continue;
            }

            if (!visited[neighbor])
            {
                visited[neighbor] = true;
                parent[neighbor] = (int8_t)current_node;

                if (queue_tail >= TRACK_NODE_COUNT)
                {
                    return false;
                }
                queue[queue_tail++] = neighbor;
            }
        }
    }

    if (!visited[goal_node])
    {
        return false;
    }

    node_id = goal_node;
    while (true)
    {
        if (reverse_len >= TRACK_NODE_COUNT)
        {
            return false;
        }

        reverse_path[reverse_len++] = node_id;
        if (node_id == start_node)
        {
            break;
        }

        if (parent[node_id] < 0)
        {
            return false;
        }
        node_id = (uint8_t)parent[node_id];
    }

    if (reverse_len > path_capacity)
    {
        return false;
    }

    for (node_id = 0U; node_id < reverse_len; node_id++)
    {
        path[node_id] = reverse_path[reverse_len - 1U - node_id];
    }

    *path_len = reverse_len;
    return true;
}

bool planner_path_uses_road(const uint8_t *path,
                            uint8_t path_len,
                            uint8_t road_id)
{
    uint8_t path_index;

    if (path == NULL || path_len < 2U || road_id >= TRACK_ROAD_COUNT)
    {
        return false;
    }

    for (path_index = 0U; path_index + 1U < path_len; path_index++)
    {
        if (track_map_find_road(path[path_index], path[path_index + 1U]) == (int8_t)road_id)
        {
            return true;
        }
    }

    return false;
}

bool planner_context_init(PlannerContext *context,
                          uint8_t current_node,
                          uint8_t target_node)
{
    if (context == NULL ||
        current_node >= TRACK_NODE_COUNT ||
        target_node >= TRACK_NODE_COUNT)
    {
        return false;
    }

    context->current_node = current_node;
    context->target_node = target_node;
    context->need_replan = false;
    return true;
}

void planner_context_notify_road_state(PlannerContext *context,
                                       uint8_t road_id,
                                       RoadState state,
                                       const uint8_t *path,
                                       uint8_t path_len)
{
    if (context == NULL || state != ROAD_BLOCKED)
    {
        return;
    }

    if (planner_path_uses_road(path, path_len, road_id))
    {
        context->need_replan = true;
    }
}

void planner_context_clear_replan(PlannerContext *context)
{
    if (context != NULL)
    {
        context->need_replan = false;
    }
}
