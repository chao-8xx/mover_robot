#ifndef GONGXUN_1_TRACK_MAP_H
#define GONGXUN_1_TRACK_MAP_H


#define MAP_WIDTH   2400.0f
#define MAP_HEIGHT  2400.0f

#define MAP_FORBIDDEN_COUNT     4



// 地图区域类型
typedef enum {
    map_outside = 0,
    map_normal,
    map_forbidden,
} map_area_t;


// 地图边界矩形结构体
typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
} map_rect_t;


map_area_t map_check_area(float x, float y);




#endif // GONGXUN_1_TRACK_MAP_H
