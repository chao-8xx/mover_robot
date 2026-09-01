#ifndef GONGXUN_1_PLANNER_H
#define GONGXUN_1_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "track_map.h"

#define PLANNER_MAX_PATH_NODES TRACK_NODE_COUNT

typedef enum
{
    PLANNER_POLICY_FREE_ONLY = 0,
    PLANNER_POLICY_ALLOW_UNKNOWN
} PlannerRoadPolicy;

typedef struct
{
    uint8_t current_node;
    uint8_t target_node;
    bool need_replan;
} PlannerContext;

bool planner_find_path(uint8_t start_node,
                       uint8_t goal_node,
                       uint8_t *path,
                       uint8_t path_capacity,
                       uint8_t *path_len);

bool planner_find_path_with_policy(uint8_t start_node,
                                   uint8_t goal_node,
                                   PlannerRoadPolicy policy,
                                   uint8_t *path,
                                   uint8_t path_capacity,
                                   uint8_t *path_len);

bool planner_path_uses_road(const uint8_t *path,
                            uint8_t path_len,
                            uint8_t road_id);

bool planner_context_init(PlannerContext *context,
                          uint8_t current_node,
                          uint8_t target_node);

void planner_context_notify_road_state(PlannerContext *context,
                                       uint8_t road_id,
                                       RoadState state,
                                       const uint8_t *path,
                                       uint8_t path_len);

void planner_context_clear_replan(PlannerContext *context);

#endif // GONGXUN_1_PLANNER_H
