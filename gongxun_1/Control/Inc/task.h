#ifndef GONGXUN_1_TASK_H
#define GONGXUN_1_TASK_H

// ======== 任务状态机、结构体定义 ======== //
// 颜色任务编码
typedef enum {
    TASK_RED = 1,         // 红色 -> 1号
    TASK_YELLOW = 2,      // 黄色 -> 2号
    TASK_BLUE = 3,        // 蓝色 -> 3号
    TASK_GREEN = 4,       // 绿色 -> 4号
    TASK_BLACK = 5,       // 黑色 -> 5号
    TASK_LOW_BLUE = 6,    // 浅蓝 -> 6号
} Color_Code;

// 粗加工区、暂存区位置编码
typedef enum {
    AREA_1 = 1, // 一号位置 -> 1号
    AREA_2 = 2, // 二号位置 -> 2号
    AREA_3 = 3, // 三号位置 -> 3号
} Area_Code;

// 车辆工作状态
typedef enum {
    STOP,       // 车辆停止
    RUNNING,    // 底盘运动中 在去目标点位的路上
    CATCHING,   // 正在抓取 把物料从赛道上抓至车上
    PUTTING,    // 正在放置 把物料从车上放至赛道上
    PICKING,    // 正在拿起 把物料从赛道上放至车上
    WAITING     // 正在等待 等待物料到来
} Car_Work;

// 目标点任务状态
// 正在执行任务的位置
typedef enum {
    PARKING_1,  // 启停区1
    PARKING_2,  // 启停区2
    SCAN,       // 扫码区
    MATERIAL,   // 原料区
    TEMPORARY,  // 暂存区
    PROCESS,    // 粗加工区
    ROAD        // 路上
} Task_Location;

// 总任务状态
typedef struct {

    // ==== 车辆当前运行状态 ==== //
    Claw_State claw_state;          // 夹爪状态
    Turntable_Pos turntable_pos;    // 转盘当前位置
    Car_Work car_work;              // 车辆工作状态
    Task_Location task_location;    // 正在执行任务的位置
    float cur_stretch_dist;         // 伸缩装置当前伸缩的距离
    float cur_lift_height;          // 升降装置当前的高度
    uint8_t cur_x, cur_y;           // 车辆当前坐标
    float catch_cur_angle;          // 齿轮齿条电机当前旋转角度
    int16_t follow_x, follow_y;     // 当前所追踪的目标的坐标
    uint8_t is_find_target;         // 车辆是否发现目标 0:未发现 1:已发现
    uint8_t is_stable_target;       // 目标是否稳定 0:不稳定 1:稳定
    uint8_t is_catch_target;        // 是否可以抓取目标 0:不可抓取 1:可抓取

    // ==== 任务编码状态 ==== //
    // 物料颜色、位置对应索引
    uint8_t index_1;    // 第一组
    uint8_t index_2;    // 第二组
    // 存放任务物料颜色和位置顺序数组(一一对应)
    Color_Code color_buf_one[3];    // 第一组
    Area_Code  area_buf_one[3];
    Color_Code color_buf_two[3];    // 第二组
    Area_Code  area_buf_two[3];
    // 第一二组完成情况
    uint8_t one_finish;     // 第一组物料搬运是否完成(到达暂存区) 0:未完成 1:完成
    uint8_t two_finish;     // 第二组物料搬运是否完成(放过粗加工区) 0:未完成 1:完成

} Task_State;

// ======== 宏定义 ======== //
#define MAX_STRETCH_DIST    6.0f      // 最远伸出距离，防止超限导致电机堵转
// ======== 结构体、枚举、结构体声明 ======== //
extern Task_State task_state;

// ======== 函数声明 ======== //
// 任务初始化
void task_init();
// 任务接收
void task_receive(void);
// 任务执行总函数
void task_process(void);

#endif //GONGXUN_1_TASK_H

/*
    156+123+516+231，表示
    第一批搬运物料的顺序是红色、黑色、浅蓝，分别放置在粗加工区和暂存区的 1、2、3 号
    圆环上，第二批搬运物料的顺序是黑色、红色、浅蓝，放置在粗加工区的 2、3、1号圆环上。
*/
