#include "obstacle_map.h"

#include "grid.h"
#include "track_map.h"

#include <math.h>
#include <stddef.h>

static void obstacle_map_projection_clear(ObstacleMapProjection *projection)
{
    if (projection != NULL)
    {
        projection->grid_x = OBSTACLE_MAP_INVALID_CELL;
        projection->grid_y = OBSTACLE_MAP_INVALID_CELL;
        projection->road_mask = 0U;
    }
}

void obstacle_map_init(void)
{
    obstacle_map_reset();
}

void obstacle_map_reset(void)
{
    occupancy_grid_init();
    track_map_init();

    /* 赛道拓扑已知，只有随机障碍物未知，因此默认道路可通行。 */
    (void)track_map_set_all_road_states(ROAD_FREE);
}

bool obstacle_map_project_world_point(float x_mm,
                                      float y_mm,
                                      ObstacleMapProjection *projection)
{
    uint8_t grid_x;
    uint8_t grid_y;

    obstacle_map_projection_clear(projection);

    if (projection == NULL ||
        !isfinite(x_mm) || !isfinite(y_mm) ||
        x_mm < 0.0f || y_mm < 0.0f ||
        x_mm >= (float)OCCUPANCY_GRID_FIELD_SIZE_MM ||
        y_mm >= (float)OCCUPANCY_GRID_FIELD_SIZE_MM)
    {
        return false;
    }

    if (!occupancy_grid_world_to_cell((int16_t)x_mm,
                                      (int16_t)y_mm,
                                      &grid_x,
                                      &grid_y))
    {
        return false;
    }

    projection->grid_x = grid_x;
    projection->grid_y = grid_y;
    projection->road_mask = track_map_grid_to_road_mask(grid_x, grid_y);
    return true;
}

bool obstacle_map_mark_world_point(float x_mm,
                                   float y_mm,
                                   ObstacleMapProjection *projection)
{
    ObstacleMapProjection local_projection;

    if (!obstacle_map_project_world_point(x_mm,
                                          y_mm,
                                          &local_projection) ||
        !occupancy_grid_set_state(local_projection.grid_x,
                                  local_projection.grid_y,
                                  GRID_CELL_OCCUPIED))
    {
        obstacle_map_projection_clear(projection);
        return false;
    }

    if (local_projection.road_mask != 0U &&
        !track_map_set_road_mask_state(local_projection.road_mask,
                                       ROAD_BLOCKED))
    {
        obstacle_map_projection_clear(projection);
        return false;
    }

    if (projection != NULL)
    {
        *projection = local_projection;
    }

    return true;
}
