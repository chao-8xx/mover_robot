#include "headfile.h"

#define MACRO_NODE(node_id)  {TRACK_MACRO_NODE, (uint8_t)(node_id)}
#define MACRO_ROAD(road_id)  {TRACK_MACRO_ROAD, (uint8_t)(road_id)}
#define MACRO_FORBIDDEN      {TRACK_MACRO_FORBIDDEN, TRACK_INVALID_ID}

typedef struct
{
    uint8_t node_a;
    uint8_t node_b;
} TrackRoadDefinition;

static const TrackNode track_nodes[TRACK_NODE_COUNT] =
{
    { 275, 2125, TRACK_ROAD_MASK(TRACK_ROAD_R0) | TRACK_ROAD_MASK(TRACK_ROAD_R6) },
    {1200, 2125, TRACK_ROAD_MASK(TRACK_ROAD_R0) | TRACK_ROAD_MASK(TRACK_ROAD_R1) | TRACK_ROAD_MASK(TRACK_ROAD_R8) },
    {2125, 2125, TRACK_ROAD_MASK(TRACK_ROAD_R1) | TRACK_ROAD_MASK(TRACK_ROAD_R10) },
    { 275, 1200, TRACK_ROAD_MASK(TRACK_ROAD_R2) | TRACK_ROAD_MASK(TRACK_ROAD_R6) | TRACK_ROAD_MASK(TRACK_ROAD_R7) },
    {1200, 1200, TRACK_ROAD_MASK(TRACK_ROAD_R2) | TRACK_ROAD_MASK(TRACK_ROAD_R3) | TRACK_ROAD_MASK(TRACK_ROAD_R8) | TRACK_ROAD_MASK(TRACK_ROAD_R9) },
    {2125, 1200, TRACK_ROAD_MASK(TRACK_ROAD_R3) | TRACK_ROAD_MASK(TRACK_ROAD_R10) | TRACK_ROAD_MASK(TRACK_ROAD_R11) },
    { 275,  275, TRACK_ROAD_MASK(TRACK_ROAD_R4) | TRACK_ROAD_MASK(TRACK_ROAD_R7) },
    {1200,  275, TRACK_ROAD_MASK(TRACK_ROAD_R4) | TRACK_ROAD_MASK(TRACK_ROAD_R5) | TRACK_ROAD_MASK(TRACK_ROAD_R9) },
    {2125,  275, TRACK_ROAD_MASK(TRACK_ROAD_R5) | TRACK_ROAD_MASK(TRACK_ROAD_R11) }
};

static const TrackRoadDefinition track_road_definitions[TRACK_ROAD_COUNT] =
{
    {TRACK_NODE_N0, TRACK_NODE_N1},
    {TRACK_NODE_N1, TRACK_NODE_N2},
    {TRACK_NODE_N3, TRACK_NODE_N4},
    {TRACK_NODE_N4, TRACK_NODE_N5},
    {TRACK_NODE_N6, TRACK_NODE_N7},
    {TRACK_NODE_N7, TRACK_NODE_N8},
    {TRACK_NODE_N0, TRACK_NODE_N3},
    {TRACK_NODE_N3, TRACK_NODE_N6},
    {TRACK_NODE_N1, TRACK_NODE_N4},
    {TRACK_NODE_N4, TRACK_NODE_N7},
    {TRACK_NODE_N2, TRACK_NODE_N5},
    {TRACK_NODE_N5, TRACK_NODE_N8}
};

/* macro_y=0 is the bottom row of the competition field. */
static const TrackMacroCell track_macro_map[TRACK_MACRO_HEIGHT][TRACK_MACRO_WIDTH] =
{
    {
        MACRO_NODE(TRACK_NODE_N6),
        MACRO_ROAD(TRACK_ROAD_R4),
        MACRO_NODE(TRACK_NODE_N7),
        MACRO_ROAD(TRACK_ROAD_R5),
        MACRO_NODE(TRACK_NODE_N8)
    },
    {
        MACRO_ROAD(TRACK_ROAD_R7),
        MACRO_FORBIDDEN,
        MACRO_ROAD(TRACK_ROAD_R9),
        MACRO_FORBIDDEN,
        MACRO_ROAD(TRACK_ROAD_R11)
    },
    {
        MACRO_NODE(TRACK_NODE_N3),
        MACRO_ROAD(TRACK_ROAD_R2),
        MACRO_NODE(TRACK_NODE_N4),
        MACRO_ROAD(TRACK_ROAD_R3),
        MACRO_NODE(TRACK_NODE_N5)
    },
    {
        MACRO_ROAD(TRACK_ROAD_R6),
        MACRO_FORBIDDEN,
        MACRO_ROAD(TRACK_ROAD_R8),
        MACRO_FORBIDDEN,
        MACRO_ROAD(TRACK_ROAD_R10)
    },
    {
        MACRO_NODE(TRACK_NODE_N0),
        MACRO_ROAD(TRACK_ROAD_R0),
        MACRO_NODE(TRACK_NODE_N1),
        MACRO_ROAD(TRACK_ROAD_R1),
        MACRO_NODE(TRACK_NODE_N2)
    }
};

static TrackRoad track_roads[TRACK_ROAD_COUNT];

static bool track_map_road_state_is_valid(RoadState state)
{
    return state == ROAD_UNKNOWN ||
           state == ROAD_FREE ||
           state == ROAD_BLOCKED;
}

static uint16_t track_map_calculate_length(uint8_t node_a, uint8_t node_b)
{
    int32_t dx = (int32_t)track_nodes[node_a].x_mm - track_nodes[node_b].x_mm;
    int32_t dy = (int32_t)track_nodes[node_a].y_mm - track_nodes[node_b].y_mm;

    if (dx < 0)
    {
        dx = -dx;
    }
    if (dy < 0)
    {
        dy = -dy;
    }

    return (uint16_t)(dx + dy);
}

static uint8_t track_map_grid_to_macro_axis(uint8_t grid_index)
{
    if (grid_index < 11U)
    {
        return 0U;
    }
    if (grid_index < 20U)
    {
        return 1U;
    }
    if (grid_index < 28U)
    {
        return 2U;
    }
    if (grid_index < 37U)
    {
        return 3U;
    }
    return 4U;
}

void track_map_init(void)
{
    uint8_t road_id;

    for (road_id = 0; road_id < TRACK_ROAD_COUNT; road_id++)
    {
        track_roads[road_id].node_a = track_road_definitions[road_id].node_a;
        track_roads[road_id].node_b = track_road_definitions[road_id].node_b;
        track_roads[road_id].length_mm = track_map_calculate_length(
            track_roads[road_id].node_a,
            track_roads[road_id].node_b);
        track_roads[road_id].state = ROAD_UNKNOWN;
    }
}

const TrackNode *track_map_get_node(uint8_t node_id)
{
    if (node_id >= TRACK_NODE_COUNT)
    {
        return NULL;
    }

    return &track_nodes[node_id];
}

const TrackRoad *track_map_get_road(uint8_t road_id)
{
    if (road_id >= TRACK_ROAD_COUNT)
    {
        return NULL;
    }

    return &track_roads[road_id];
}

RoadState track_map_get_road_state(uint8_t road_id)
{
    if (road_id >= TRACK_ROAD_COUNT)
    {
        return ROAD_UNKNOWN;
    }

    return track_roads[road_id].state;
}

bool track_map_set_road_state(uint8_t road_id, RoadState state)
{
    if (road_id >= TRACK_ROAD_COUNT || !track_map_road_state_is_valid(state))
    {
        return false;
    }

    track_roads[road_id].state = state;
    return true;
}

bool track_map_set_all_road_states(RoadState state)
{
    uint8_t road_id;

    if (!track_map_road_state_is_valid(state))
    {
        return false;
    }

    for (road_id = 0; road_id < TRACK_ROAD_COUNT; road_id++)
    {
        track_roads[road_id].state = state;
    }

    return true;
}

bool track_map_set_road_mask_state(uint16_t road_mask, RoadState state)
{
    uint8_t road_id;

    if (road_mask == 0U ||
        (road_mask & (uint16_t)(~TRACK_ALL_ROADS_MASK)) != 0U ||
        !track_map_road_state_is_valid(state))
    {
        return false;
    }

    for (road_id = 0; road_id < TRACK_ROAD_COUNT; road_id++)
    {
        if ((road_mask & TRACK_ROAD_MASK(road_id)) != 0U)
        {
            track_roads[road_id].state = state;
        }
    }

    return true;
}

int8_t track_map_find_road(uint8_t node_a, uint8_t node_b)
{
    uint8_t road_id;

    if (node_a >= TRACK_NODE_COUNT || node_b >= TRACK_NODE_COUNT)
    {
        return -1;
    }

    for (road_id = 0; road_id < TRACK_ROAD_COUNT; road_id++)
    {
        if ((track_roads[road_id].node_a == node_a && track_roads[road_id].node_b == node_b) ||
            (track_roads[road_id].node_a == node_b && track_roads[road_id].node_b == node_a))
        {
            return (int8_t)road_id;
        }
    }

    return -1;
}

const TrackMacroCell *track_map_get_macro_cell(uint8_t macro_x,
                                               uint8_t macro_y)
{
    if (macro_x >= TRACK_MACRO_WIDTH || macro_y >= TRACK_MACRO_HEIGHT)
    {
        return NULL;
    }

    return &track_macro_map[macro_y][macro_x];
}

bool track_map_grid_to_macro(uint8_t grid_x,
                             uint8_t grid_y,
                             uint8_t *macro_x,
                             uint8_t *macro_y)
{
    if (grid_x >= OCCUPANCY_GRID_WIDTH ||
        grid_y >= OCCUPANCY_GRID_HEIGHT ||
        macro_x == NULL || macro_y == NULL)
    {
        return false;
    }

    *macro_x = track_map_grid_to_macro_axis(grid_x);
    *macro_y = track_map_grid_to_macro_axis(grid_y);
    return true;
}

uint16_t track_map_grid_to_road_mask(uint8_t grid_x,
                                     uint8_t grid_y)
{
    uint8_t macro_x;
    uint8_t macro_y;
    const TrackMacroCell *macro_cell;

    if (!track_map_grid_to_macro(grid_x, grid_y, &macro_x, &macro_y))
    {
        return 0U;
    }

    macro_cell = track_map_get_macro_cell(macro_x, macro_y);
    if (macro_cell == NULL)
    {
        return 0U;
    }

    if (macro_cell->type == TRACK_MACRO_ROAD)
    {
        return TRACK_ROAD_MASK(macro_cell->id);
    }

    if (macro_cell->type == TRACK_MACRO_NODE)
    {
        return track_nodes[macro_cell->id].observation_mask;
    }

    return 0U;
}

int8_t track_map_grid_to_road(uint16_t grid_x,
                              uint16_t grid_y)
{
    uint8_t macro_x;
    uint8_t macro_y;
    const TrackMacroCell *macro_cell;

    if (grid_x >= OCCUPANCY_GRID_WIDTH || grid_y >= OCCUPANCY_GRID_HEIGHT ||
        !track_map_grid_to_macro((uint8_t)grid_x,
                                 (uint8_t)grid_y,
                                 &macro_x,
                                 &macro_y))
    {
        return -1;
    }

    macro_cell = track_map_get_macro_cell(macro_x, macro_y);
    if (macro_cell == NULL || macro_cell->type != TRACK_MACRO_ROAD)
    {
        return -1;
    }

    return (int8_t)macro_cell->id;
}
