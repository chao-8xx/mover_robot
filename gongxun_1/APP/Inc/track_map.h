#ifndef GONGXUN_1_TRACK_MAP_H
#define GONGXUN_1_TRACK_MAP_H

#define TRACK_NODE_COUNT          9U
#define TRACK_ROAD_COUNT         12U
#define TRACK_MACRO_WIDTH         5U
#define TRACK_MACRO_HEIGHT        5U
#define TRACK_INVALID_ID        0xFFU

#define TRACK_ROAD_MASK(road_id) ((uint16_t)(1U << (uint8_t)(road_id)))
#define TRACK_ALL_ROADS_MASK     ((uint16_t)((1U << TRACK_ROAD_COUNT) - 1U))

typedef enum
{
    TRACK_NODE_N0 = 0,
    TRACK_NODE_N1,
    TRACK_NODE_N2,
    TRACK_NODE_N3,
    TRACK_NODE_N4,
    TRACK_NODE_N5,
    TRACK_NODE_N6,
    TRACK_NODE_N7,
    TRACK_NODE_N8
} TrackNodeId;

typedef enum
{
    TRACK_ROAD_R0 = 0,
    TRACK_ROAD_R1,
    TRACK_ROAD_R2,
    TRACK_ROAD_R3,
    TRACK_ROAD_R4,
    TRACK_ROAD_R5,
    TRACK_ROAD_R6,
    TRACK_ROAD_R7,
    TRACK_ROAD_R8,
    TRACK_ROAD_R9,
    TRACK_ROAD_R10,
    TRACK_ROAD_R11
} TrackRoadId;

typedef enum
{
    ROAD_UNKNOWN = 0,
    ROAD_FREE,
    ROAD_BLOCKED
} RoadState;

typedef struct
{
    int16_t x_mm;
    int16_t y_mm;
    uint16_t observation_mask;
} TrackNode;

typedef struct
{
    uint8_t node_a;
    uint8_t node_b;
    uint16_t length_mm;
    RoadState state;
} TrackRoad;

typedef enum
{
    TRACK_MACRO_NODE = 0,
    TRACK_MACRO_ROAD,
    TRACK_MACRO_FORBIDDEN
} TrackMacroType;

typedef struct
{
    TrackMacroType type;
    uint8_t id;
} TrackMacroCell;

void track_map_init(void);

const TrackNode *track_map_get_node(uint8_t node_id);
const TrackRoad *track_map_get_road(uint8_t road_id);

RoadState track_map_get_road_state(uint8_t road_id);
bool track_map_set_road_state(uint8_t road_id, RoadState state);
bool track_map_set_all_road_states(RoadState state);
bool track_map_set_road_mask_state(uint16_t road_mask, RoadState state);

int8_t track_map_find_road(uint8_t node_a, uint8_t node_b);

const TrackMacroCell *track_map_get_macro_cell(uint8_t macro_x,
                                               uint8_t macro_y);

bool track_map_grid_to_macro(uint8_t grid_x,
                             uint8_t grid_y,
                             uint8_t *macro_x,
                             uint8_t *macro_y);

uint16_t track_map_grid_to_road_mask(uint8_t grid_x,
                                    uint8_t grid_y);

int8_t track_map_grid_to_road(uint16_t grid_x,
                              uint16_t grid_y);

#endif // GONGXUN_1_TRACK_MAP_H
