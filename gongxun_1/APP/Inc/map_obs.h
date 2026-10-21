#ifndef GONGXUN_1_LIDAR_OBSTACLE_H
#define GONGXUN_1_LIDAR_OBSTACLE_H


// 雷达点云参数 待调
#define LIDAR_ANGLE_COUNT          360        // 雷达整数角度数量 0~359度
#define LIDAR_MAX_DISTANCE         2200.0f    // 雷达最大有效测距距离 mm
#define LIDAR_MIN_DISTANCE         50.0f      // 雷达最小有效测距距离 mm


// 相邻点聚类参数
#define CLUSTER_MAX_MISS_ANGLE        2       // 聚类点间允许连续缺失的角度数 超过该值则认为聚类结束
#define CLUSTER_MAX_COUNT            20       // 单帧最多保存的候选聚类数量
#define CLUSTER_MIN_POINT_COUNT       3       // 少于3个点的聚类作为噪声丢弃
#define CLUSTER_MAX_POINT_GAP       100.0f    // 相邻两点最大连接距离 mm

// 障碍物小圆柱候选参数: 赛题圆柱直径50mm  先用 100mm 作为测距误差、缺点情况下的宽松试验值
// 只检查平面跨度  距离先沿用点云范围  可靠距离等实测后收紧
#define LIDAR_PILLAR_MAX_SIZE       100.0f
#define LIDAR_PILLAR_MAX_DISTANCE   LIDAR_MAX_DISTANCE


// 2026-10-19 整车垫高、轮子相对雷达不变时测得的车体/车轮遮挡范围
// 全部使用雷达原始角度、包含首尾  更换碳板或改变安装位置后重新测量
#ifndef LIDAR_WHEEL_MASK_ENABLE
#define LIDAR_WHEEL_MASK_ENABLE    1    // 1 启用当前实测角度屏蔽
#endif

#define LEFT_FORNT_WHEEL_START     36   // 左前方向遮挡起始角度
#define LEFT_FORNT_WHEEL_END       62   // 左前方向遮挡结束角度

#define RIGHT_FORNT_WHEEL_START    302  // 右前方向遮挡起始角度
#define RIGHT_FORNT_WHEEL_END      314  // 右前方向遮挡结束角度

#define LEFT_REAR_WHEEL_START      125  // 左后方向遮挡起始角度
#define LEFT_REAR_WHEEL_END        140  // 左后方向遮挡结束角度

#define RIGHT_REAR_WHEEL_START     218  // 右后方向遮挡起始角度
#define RIGHT_REAR_WHEEL_END       236  // 右后方向遮挡结束角度

// 单独保留实测的短段回波
#define LIDAR_BODY_MASK_START      315
#define LIDAR_BODY_MASK_END        325


// 雷达点结构体
// 表示某一个整数角度的测距点及其经过TF后的地图坐标
typedef struct
{
    point2d_t map_point; // 雷达点在map坐标系中的位置
    float distance;      // 雷达原始测距距离 mm
    uint8_t valid;       // 1表示当前角度的数据有效，0表示无效
} lidar_point_t;


// 点云结构体
// 保存一次360度点云更新的完整结果以及生成点云时的小车位姿
typedef struct
{
    lidar_point_t points[LIDAR_ANGLE_COUNT]; // 0~359度雷达点
    uint16_t valid_count;                    // 当前有效雷达点数量

    float car_x;      // 生成点云时的小车map坐标X mm
    float car_y;      // 生成点云时的小车map坐标Y mm
    float car_angle;  // 生成点云时的小车偏航角 degree

    uint32_t scan_num;    // 点云对应的雷达扫描编号
    uint32_t scan_tick;   // 雷达采集结束时间  不能用处理时间冒充新数据
    uint32_t update_flag; // 每完成一次点云更新加1
    uint32_t update_tick; // 点云更新时刻 ms
} point_cloud_t;


// 雷达聚类结构体   (点和包围框使用map坐标系)
// 一组相互连接的雷达点形成一个候选物体
typedef struct
{
    uint16_t start_angle;   // 聚类起始角度
    uint16_t end_angle;     // 聚类结束角度，跨0度时可能小于start_angle
    uint16_t point_count;   // 聚类包含的有效点数量

    point2d_t center_point; // 聚类中心的map坐标

    float min_x;            // 聚类包围框最小X坐标
    float max_x;            // 聚类包围框最大X坐标
    float min_y;            // 聚类包围框最小Y坐标
    float max_y;            // 聚类包围框最大Y坐标

    float width;            // 聚类包围框宽度 mm
    float height;           // 聚类包围框高度 mm
    float nearest_distance; // 聚类中距离雷达最近的点 mm

    uint8_t valid; // 1表示满足最小点数要求，0表示无效
} lidar_cluster_t;


// 障碍物候选结果结构体
// 保存由当前点云分割得到的全部有效聚类
typedef struct
{
    lidar_cluster_t clusters[CLUSTER_MAX_COUNT];
    uint16_t cluster_count;     // 当前有效聚类数量
    uint8_t truncated;          // 1表示合格聚类超过容量被截断  本圈不能用于道路放行
    uint32_t cloud_flag;        // 本结果使用的点云更新序号
    uint32_t update_flag;       // 每完成一次聚类更新加1
    uint32_t update_tick;       // 聚类结果更新时间 ms
} obstacle_result_t;


extern point_cloud_t cloud;
extern obstacle_result_t obstacle_result;

// 点云生成
uint8_t point_cloud_update(const lidar_scan_t *scan, float car_x, float car_y, float car_yaw, uint32_t tick);
// 相邻点聚类
void obstacle_cluster_update(const point_cloud_t *cloud, uint32_t tick);
// 简单小柱候选筛选：保留原有最少3点要求，不按最大点数过滤，也不代表已识别颜色或圆柱形状
uint8_t obstacle_cluster_is_pillar(const lidar_cluster_t *cluster);

#endif // GONGXUN_1_LIDAR_OBSTACLE_H
