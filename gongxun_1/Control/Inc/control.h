#ifndef GONGXUN_1_CONTROL_H
#define GONGXUN_1_CONTROL_H

// ======== 步骤定义 ======== //
// 离开、回到启停区所在步骤
#define LEAVE_PARKING_STEP   2      // 离开启停区
#define BACK_PARKING_STEP    3      // 回到启停区
// 规划去往各点的所在步骤
#define TARGET_STEP_LEN      8      // 目标点顺序个数
#define TARGET_PARK_STEP     4      // 启停区
#define TARGET_SCAN_STEP     5      // 扫码区
#define TARGET_META_STEP     6      // 原料区
#define TARGET_TEMP_STEP     7      // 暂存区
#define TARGET_PROC_STEP     8      // 粗加工区
// 不同道路运行的所在步骤
#define CALIBRATE_YAW_STEP      9    // 校准角度
#define NORMAL_ROAD_MOVE_STEP   10   // 常规道路
#define FINAL_ROAD_MOVE_STEP    11   // 最后一段路
// 执行对物料各项任务的所在步骤 含跟踪物料
#define SCAN_GET_TASK_STEP      12   // 扫码获取任务
#define CATCH_MATERIAL_STEP     13   // 抓取物料
#define PUT_MATERIAL_STEP       14   // 放置物料 从车上放到暂存区、粗加工区
#define PICK_MATERIAL_STEP      15   // 拿起物料 从粗加工区拿到车上

// 控制变量结构体
typedef struct {
    uint8_t step;             // 控制步骤
    uint8_t path_index;       // 路径索引
    uint8_t path_len;         // 路径长度
    uint8_t run_start;        // 到达目标点最后一段路的起始（用于记录最后一段路的长度）
    uint8_t final_run_len;    // 最后一段路的长度
    uint8_t ax,ay,bx,by;      // 两个点坐标（用于求向量）
    int8_t  dx,dy;            // 当前目标向量
    int8_t  head_x,head_y;    // 当前车头朝向
    int8_t  last_dx, last_dy; // 到达目标点最后一段路的目标向量（与目标点任务向量一致）

    uint8_t move_state;       // 移动状态 1：等待位移完成（用于在非任务区完成每段的移动）
    uint8_t rotate_state;     // 转向状态 1：旋转闭环进行中（用于在非任务区转向和调整姿态）
    uint8_t lift_state;       // 升降状态 1：升降进行中
    uint8_t stretch_state;    // 转向状态 1：伸缩进行中

    uint8_t is_send_color;
    uint8_t is_send_circle;
    uint8_t is_send_onup;

    int16_t speed;            // 底盘运行速度（非任务区）
    float   target_yaw;       // 目标偏航角
    float   target_lift;      // 升降装置目标高度
    float   target_stretch;   // 目标伸缩距离

    uint8_t target_pos[TARGET_STEP_LEN];     // 目标点顺序 按顺序前往指定位置
    uint8_t target_index;     // 目标点顺序索引
    int8_t  tx,ty;            // 目标点目标向量
} Control_Variate;

// 结构体声明
extern Control_Variate my_control;

// 控制变量初始化
void control_variate_init(void);

// 离开启停区 步骤2
void leave_parking(void);
// 回到启停区 步骤3
void back_parking(void);

// 规划去往启停区 步骤4
void track_to_parking(void);
// 规划去往扫码区 步骤5
void track_to_scan(void);
// 规划去往原料区 步骤6
void track_to_material(void);
// 规划去往暂存区 步骤7
void track_to_temporary(void);
// 规划去往粗加工区 步骤8
void track_to_process(void);

// 校准该点角度 到点才校准
void calibrate_yaw(void);
// 常规道路运行 步骤10
void normal_road_move(void);
// 到目标点最后一段道路运行 步骤11
void final_road_move(void);


// 目标状态获取 将摄像头所捕获的状态转化成车辆状态机
void target_state_get(void);
// 确认是否稳定可抓取 1:可抓取
uint8_t catch_is_ok(void);
// 确认是否稳定可放置 1:可放置
uint8_t put_is_ok(void);
// 将当前需要抓取的颜色发给maix
void send_color_conduct(void);
// 将当前锁定的圆环编号发给maix
void send_circle_conduct(void);
// 将当前需要抓取的码垛发给maix
void send_onup_conduct(void);
// 追踪目标 物料 圆环
// 校准车身姿态 追踪x和y坐标
void follow_target(float target_yaw, int16_t target_x, int16_t target_y);

// 扫码获取任务
void scan_get_task(void);
// 发送任务码置串口屏上
void scan_send_task_hmi(void);

// 抓取物料 原料区->转盘
void catch_material(void);
// 放置物料 转盘->暂存区、粗加工区
void put_material(void);
// 拿起物料 暂存区、粗加工区->转盘
void pick_material(void);

// 整套控制流程
void control_process(void);


#endif //GONGXUN_1_CONTROL_H

/* 指示灯颜色：
 * 红色：扫码失败
 * 绿色：扫码成功、目标识别成功
 * 粉色：到达启停区
 * 蓝色：到达原料区
 * 黄色：到达粗加工区、暂存区
 * 白色：第一批物料完成、两批物料完成
 */
