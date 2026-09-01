#include "headfile.h"


static OccupancyCell occupancy_cells[OCCUPANCY_GRID_HEIGHT][OCCUPANCY_GRID_WIDTH];

static bool occupancy_grid_state_is_valid(GridCellState state)
{
    return state == GRID_CELL_UNKNOWN ||
           state == GRID_CELL_FREE ||
           state == GRID_CELL_OCCUPIED;
}

void occupancy_grid_init(void)
{
    occupancy_grid_reset(GRID_CELL_UNKNOWN);
}

void occupancy_grid_reset(GridCellState state)
{
    uint8_t grid_x;
    uint8_t grid_y;

    if (!occupancy_grid_state_is_valid(state))
    {
        return;
    }

    for (grid_y = 0; grid_y < OCCUPANCY_GRID_HEIGHT; grid_y++)
    {
        for (grid_x = 0; grid_x < OCCUPANCY_GRID_WIDTH; grid_x++)
        {
            occupancy_cells[grid_y][grid_x].state = (uint8_t)state;
        }
    }
}

bool occupancy_grid_set_state(uint8_t grid_x,
                              uint8_t grid_y,
                              GridCellState state)
{
    if (grid_x >= OCCUPANCY_GRID_WIDTH ||
        grid_y >= OCCUPANCY_GRID_HEIGHT ||
        !occupancy_grid_state_is_valid(state))
    {
        return false;
    }

    occupancy_cells[grid_y][grid_x].state = (uint8_t)state;
    return true;
}

GridCellState occupancy_grid_get_state(uint8_t grid_x,
                                       uint8_t grid_y)
{
    if (grid_x >= OCCUPANCY_GRID_WIDTH ||
        grid_y >= OCCUPANCY_GRID_HEIGHT)
    {
        return GRID_CELL_UNKNOWN;
    }

    return (GridCellState)occupancy_cells[grid_y][grid_x].state;
}

bool occupancy_grid_world_to_cell(int16_t x_mm,
                                  int16_t y_mm,
                                  uint8_t *grid_x,
                                  uint8_t *grid_y)
{
    if (grid_x == NULL || grid_y == NULL ||
        x_mm < 0 || y_mm < 0 ||
        x_mm >= (int16_t)OCCUPANCY_GRID_FIELD_SIZE_MM ||
        y_mm >= (int16_t)OCCUPANCY_GRID_FIELD_SIZE_MM)
    {
        return false;
    }

    *grid_x = (uint8_t)((uint16_t)x_mm / OCCUPANCY_GRID_CELL_SIZE_MM);
    *grid_y = (uint8_t)((uint16_t)y_mm / OCCUPANCY_GRID_CELL_SIZE_MM);
    return true;
}
