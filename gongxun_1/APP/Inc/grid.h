#ifndef GONGXUN_1_OCCUPANCY_GRID_H
#define GONGXUN_1_OCCUPANCY_GRID_H

#define OCCUPANCY_GRID_FIELD_SIZE_MM  2400U
#define OCCUPANCY_GRID_CELL_SIZE_MM     50U
#define OCCUPANCY_GRID_WIDTH            48U
#define OCCUPANCY_GRID_HEIGHT           48U

typedef enum
{
    GRID_CELL_UNKNOWN = 0,
    GRID_CELL_FREE,
    GRID_CELL_OCCUPIED
} GridCellState;

typedef struct
{
    uint8_t state;
} OccupancyCell;

void occupancy_grid_init(void);
void occupancy_grid_reset(GridCellState state);

bool occupancy_grid_set_state(uint8_t grid_x,
                              uint8_t grid_y,
                              GridCellState state);

GridCellState occupancy_grid_get_state(uint8_t grid_x,
                                       uint8_t grid_y);

bool occupancy_grid_world_to_cell(int16_t x_mm,
                                  int16_t y_mm,
                                  uint8_t *grid_x,
                                  uint8_t *grid_y);

#endif // GONGXUN_1_OCCUPANCY_GRID_H
