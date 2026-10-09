#include "headfile.h"

#define TARGET_DX   (0)          // 目标dx
#define TARGET_DY   (10)         // 目标dy

#define TARGET_CIRCLE_DX (-3)     // 圆环目标dx
#define TARGET_CIRCLE_DY (4)    // 圆环目标dy

#define LEAVE_PARK_DX     15.0f       // 离开启停区的横向距离
#define LEAVE_PARK_DY     17.0f       // 离开启停区的纵向距离

#define CHASSIS_X_CALI    (-7.0f)        // 底盘横移校准距离 每两格

#define MOVE_SPEED        450         // 底盘非任务区运行速度

// 根据格子数量对于行走距离 cm
#define NUM_DELTA_DIST    14.0f       // 暂存区、粗加工区圆环间距
#define ONE_BLOCK_DIST    43.5f       // 一格
#define TWO_BLOCK_DIST    87.0f       // 两格
#define THREE_BLOCK_DIST  127.5f      // 三格
#define FOUR_BLOCK_DIST   172.5f      // 四格

// 舵机等待时间 ms
#define CLAW_WAIT         600         // 等待夹爪
#define HOLDER_WAIT       1200        // 等待云台

// 任务完成标志
uint8_t scan_finish = 0;    // 扫码任务完成标志
uint8_t catch_finish = 0;   // 抓取任务完成标志
uint8_t put_finish = 0;     // 放置任务完成标志
uint8_t pick_finish = 0;    // 拿起任务完成标志

// 结构体定义
Control_Variate my_control;

// float 绝对值
static float my_absf(float data)
{
    return data < 0.0f ? -data : data;
}

// 目标点顺序初始化
static void target_pos_init(void)
{
    my_control.target_pos[0] = TARGET_SCAN_STEP;
    my_control.target_pos[1] = TARGET_META_STEP;
    my_control.target_pos[2] = TARGET_PROC_STEP;
    my_control.target_pos[3] = TARGET_TEMP_STEP;
    my_control.target_pos[4] = TARGET_META_STEP;
    my_control.target_pos[5] = TARGET_PROC_STEP;
    my_control.target_pos[6] = TARGET_TEMP_STEP;
    my_control.target_pos[7] = TARGET_PARK_STEP;

    my_control.target_index = 0;
    my_control.tx = 0;
    my_control.ty = 0;
}

// 控制变量初始化
void control_variate_init(void)
{
    target_pos_init();

    my_control.speed = MOVE_SPEED;
    my_control.target_yaw = 0.0f;

    my_control.head_x = 1;
    my_control.head_y = 0;

    my_control.move_state = 0;
    my_control.rotate_state = 0;

    my_control.step = 0;
    my_control.path_index = 0;
    my_control.path_len = 0;
}


// 离开启停区 步骤2
void leave_parking(void)
{
    static uint8_t leave_step = 0;
    if (start_location == 0) {    // 启停区2 (0,0)
        task_state.cur_x = 0;
        task_state.cur_y = 0;
        switch (leave_step)
        {
            case 0:
                car_run_distance(my_control.speed, -LEAVE_PARK_DX, 0.0f);
                my_control.move_state = 1;
                leave_step = 1;
                break;
            case 1:
                car_run_distance(my_control.speed, 0.0f, LEAVE_PARK_DY);
                my_control.move_state = 1;
                leave_step = 2;
                break;
            case 2:
                leave_step = 0;
                my_control.step = my_control.target_pos[my_control.target_index];
                break;
            default:
                break;
        }
    }
    else if (start_location == 1) {   // 启停区1 (4,0)
        task_state.cur_x = 4;
        task_state.cur_y = 0;
        switch (leave_step)
        {
            case 0:
                car_run_distance(my_control.speed, -LEAVE_PARK_DX, 0.0f);
                my_control.move_state = 1;
                leave_step = 1;
                break;
            case 1:
                car_run_distance(my_control.speed, 0.0f, -LEAVE_PARK_DY);
                my_control.move_state = 1;
                leave_step = 2;
                break;
            case 2:
                leave_step = 0;
                my_control.step = my_control.target_pos[my_control.target_index];
                break;
            default:
                break;
        }
    }
}

// 回到启停区 步骤3
void back_parking(void)
{
    static uint8_t back_step = 0;
    if (start_location == 0) {   // 启停区2 (0,0)
        task_state.cur_x = 0;
        task_state.cur_y = 0;
        switch (back_step)
        {
            case 0:
                car_run_distance(my_control.speed, LEAVE_PARK_DX, 0.0f);
                my_control.move_state = 1;
                back_step = 1;
                break;
            case 1:
                car_run_distance(my_control.speed, 0.0f, -LEAVE_PARK_DY);
                my_control.move_state = 1;
                back_step = 2;
                break;
            case 2:
                back_step = 0;
                my_control.step = 99;
                break;
            default:
                break;
        }
    }
    else if (start_location == 1) {  // 启停区1 (4,0)
        task_state.cur_x = 4;
        task_state.cur_y = 0;
        switch (back_step)
        {
            case 0:
                car_run_distance(my_control.speed, LEAVE_PARK_DX, 0.0f);
                my_control.move_state = 1;
                back_step = 1;
                break;
            case 1:
                car_run_distance(my_control.speed, 0.0f, LEAVE_PARK_DY);
                my_control.move_state = 1;
                back_step = 2;
                break;
            case 2:
                back_step = 0;
                my_control.step = 99;
                break;
            default:
                break;
        }
    }
    maix_stop();    // maix停止视觉任务
    my_control.step = 99;
    car_run = false;
}


// 规划去往某个目的地
static void track_to_pos(uint8_t pos_x, uint8_t pos_y)
{
    // 规划路径
    if (bfs_find_path(task_state.cur_x, task_state.cur_y,
            pos_x, pos_y) == 0) {
        my_control.step = 99;
        return;
    }

    // 计算路径长度和初始化路径索引
    my_control.path_len = bfs_get_path_len();
    my_control.path_index = 1;
    if (my_control.path_len < 2) {  // 若该点就是所属点（已到达）
        my_control.step = CALIBRATE_YAW_STEP;   // 校准该点位置
        return;
    }

    // 找最后一段同方向直线
    bfs_get_path_step(my_control.path_len - 2, &my_control.ax, &my_control.ay);  // 获取目标点到上一点的坐标，并求出向量
    bfs_get_path_step(my_control.path_len - 1, &my_control.bx, &my_control.by);
    my_control.last_dx = (int8_t)((int16_t)my_control.bx - (int16_t)my_control.ax);
    my_control.last_dy = (int8_t)((int16_t)my_control.by - (int16_t)my_control.ay);

    // 从目标点路径往同向量方向回推，直到向量不相等
    my_control.run_start = my_control.path_len - 1;
    while (my_control.run_start > 1) {
        bfs_get_path_step(my_control.run_start - 2, &my_control.ax, &my_control.ay);
        bfs_get_path_step(my_control.run_start - 1, &my_control.bx, &my_control.by);
        if ((int8_t)((int16_t)my_control.bx - (int16_t)my_control.ax) == my_control.last_dx &&
            (int8_t)((int16_t)my_control.by - (int16_t)my_control.ay) == my_control.last_dy) {
            my_control.run_start--;
        }
        else {break;}
    }
    my_control.final_run_len = my_control.path_len - my_control.run_start;  // 求出最后一段路的长度
    my_control.step = NORMAL_ROAD_MOVE_STEP;
}

// 规划去往启停区 步骤4
void track_to_parking(void)
{
    if (start_location == 0) {          // 启停区2 (0,0)
        track_to_pos(0,0);
    }
    else if (start_location == 1) {     // 启停区1 (4,0)
        track_to_pos(4,0);
    }
    // 往东
    my_control.tx = 1;
    my_control.ty = 0;
}

// 规划去往扫码区 步骤5
void track_to_scan(void)
{
    track_to_pos(2,0);  // (2,0)
    // 往东
    my_control.tx = 1;
    my_control.ty = 0;
}

// 规划去往原料区 步骤6
void track_to_material(void)
{
    track_to_pos(4,2);  // (4,2)
    // 往南
    my_control.tx = 0;
    my_control.ty = -1;
}

// 规划去往暂存区 步骤7
void track_to_temporary(void)
{
    track_to_pos(2,4);  // (2,4)
    // 往东
    my_control.tx = 1;
    my_control.ty = 0;
}

// 规划去往粗加工区 步骤8
void track_to_process(void)
{
    track_to_pos(0,2);  // (0,2)
    // 往北
    my_control.tx = 0;
    my_control.ty = 1;
}


// 约定：车头朝 (1,0)=0°，顺时针 my_yaw 减小
static float track_dir_to_yaw(int8_t dx, int8_t dy)
{
    if (dx == 1 && dy == 0) return 0.0f;       // 东
    if (dx == 0 && dy == 1) return 90.0f;      // 北
    if (dx == -1 && dy == 0) return -180.0f;   // 西
    if (dx == 0 && dy == -1) return -90.0f;    // 南
    return 0.0f;
}
// 判断当前 my_yaw 是否已对准 target（±1°）
static uint8_t track_yaw_ok(float target)
{
    float d = target - my_yaw;
    while (d > 180.0f)  d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return (d > -YAW_THRESH && d < YAW_THRESH);
}
// 折算目标角
static float track_rotate_target(float target_yaw)
{
    float delta = target_yaw - my_yaw;
    while (delta > 180.0f)  delta -= 360.0f;
    while (delta <= -180.0f) delta += 360.0f;
    return my_yaw + delta;
}

// 校准该点角度 到点才校准
void calibrate_yaw(void)
{
    // 判断当前位置
    switch (my_control.target_pos[my_control.target_index])
    {
        case TARGET_PARK_STEP:      // 启停区
            if (start_location == 0) {
                task_state.cur_x = 0;
                task_state.cur_y = 0;
                task_state.task_location = PARKING_2;
                task_state.remindState = PINK_REMIND;   // 闪灯提示
            }
            else if (start_location == 1) {
                task_state.cur_x = 4;
                task_state.cur_y = 0;
                task_state.task_location = PARKING_1;
                task_state.remindState = PINK_REMIND;   // 闪灯提示
            }
            break;
        case TARGET_SCAN_STEP:      // 扫码区
            task_state.cur_x = 2;
            task_state.cur_y = 0;
            task_state.task_location = SCAN;
            task_state.remindState = PINK_REMIND;   // 闪灯提示
            break;
        case TARGET_META_STEP:      // 原料区
            task_state.cur_x = 4;
            task_state.cur_y = 2;
            task_state.task_location = MATERIAL;
            task_state.remindState = BLUE_REMIND;   // 闪灯提示
            break;
        case TARGET_TEMP_STEP:      // 暂存区
            task_state.cur_x = 2;
            task_state.cur_y = 4;
            task_state.task_location = TEMPORARY;
            task_state.remindState = YELLOW_REMIND;   // 闪灯提示
            break;
        case TARGET_PROC_STEP:      // 粗加工区
            task_state.cur_x = 0;
            task_state.cur_y = 2;
            task_state.task_location = PROCESS;
            task_state.remindState = YELLOW_REMIND;   // 闪灯提示
            break;
        default:
            task_state.task_location = ROAD;
            break;
    }

    // 校准角度
    if (my_control.tx != 0 || my_control.ty != 0) {
        my_control.target_yaw = track_dir_to_yaw(my_control.tx, my_control.ty);
        if (!track_yaw_ok(my_control.target_yaw)) {
            my_control.target_yaw = track_rotate_target(my_control.target_yaw);
            my_control.rotate_state = 1;
            // 转到目标点所要求的朝向后 刷新车头朝向
            my_control.head_x = my_control.tx;
            my_control.head_y = my_control.ty;
            return;
        }
    }

    // 根据对物料的完成情况何当前坐标 决定是规划离开还是开始工作
    if (task_state.task_location == SCAN && scan_finish == 0) { // 到达扫码区且扫码任务未完成
        my_control.step = SCAN_GET_TASK_STEP;   // 切换到扫码步骤
        return;
    }
    else if (task_state.task_location == MATERIAL && catch_finish == 0) {   // 到达原料区且扫码任务未完成
        my_control.step = CATCH_MATERIAL_STEP;  // 切换到抓取物料步骤
        return;
    }
    else if ((task_state.task_location == PROCESS && put_finish == 0) ||
        (task_state.task_location == TEMPORARY && put_finish == 0)) {      // 到达粗加工区且扫码任务未完成
        my_control.step = PUT_MATERIAL_STEP;    // 切换到放置物料步骤
        return;
    }

    // 复位完成状态
    scan_finish = 0;
    catch_finish = 0;
    put_finish = 0;
    pick_finish = 0;

    // 根据目标数组 开始规划下一目标点
    my_control.target_index += 1;
    if (my_control.target_index < TARGET_STEP_LEN) {
        my_control.step = my_control.target_pos[my_control.target_index];   // 根据数组 决定下一目标坐标
        task_state.task_location = ROAD;    // 离开工作点 状态切换为在路上
    }
    else {
        my_control.step = BACK_PARKING_STEP;
        // 根据发车点 判断下一个状态是启停区一还是二
        if (start_location == 0) {
            task_state.task_location = PARKING_2;
        }
        else if (start_location == 1) {
            task_state.task_location = PARKING_1;
        }
    }
}

// 走的格子数转化为实际距离
float block_to_distance(uint8_t count)
{
    switch (count)
    {
        case 0:
            return 0.0f;
        case 1:
            return ONE_BLOCK_DIST;
        case 2:
            return TWO_BLOCK_DIST;
        case 3:
            return THREE_BLOCK_DIST;
        case 4:
            return FOUR_BLOCK_DIST;
        default:
            return 0.0f;
    }
}

// 常规道路运行
void normal_road_move(void)
{
    // 如果路径索引进入到最后一段路内，进入最后一段路状态
    if (my_control.path_index >= my_control.run_start) {
        my_control.step = FINAL_ROAD_MOVE_STEP;
        return;
    }

    // 将目标向量转化为目标角度
    bfs_get_path_step(my_control.path_index - 1, &my_control.ax, &my_control.ay);
    bfs_get_path_step(my_control.path_index, &my_control.bx, &my_control.by);
    my_control.dx = (int8_t)((int16_t)my_control.bx - (int16_t)my_control.ax);
    my_control.dy = (int8_t)((int16_t)my_control.by - (int16_t)my_control.ay);
    // my_control.target_yaw = track_dir_to_yaw(my_control.dx, my_control.dy);

    // 与当前车头朝向点积：+1 同向 / -1 反向 / 0 垂直
    int8_t dot = (int8_t)(my_control.dx * my_control.head_x + my_control.dy * my_control.head_y);

    if (dot == 0) {     // 只有目标姿态与当前朝向垂直才转
        my_control.target_yaw = track_dir_to_yaw(my_control.dx, my_control.dy);
        if (!track_yaw_ok(my_control.target_yaw)) {
            my_control.target_yaw = track_rotate_target(my_control.target_yaw);
            my_control.rotate_state = 1;
            my_control.head_x = my_control.dx;   // 转弯后车头朝该段方向
            my_control.head_y = my_control.dy;
            return;
        }
    }
    if (car_is_busy()) return;

    {
        // 同向量路段，连续直行
        uint8_t run_len = 1;
        while (my_control.path_index + run_len < my_control.run_start) {
            bfs_get_path_step(my_control.path_index + run_len - 1, &my_control.ax, &my_control.ay);
            bfs_get_path_step(my_control.path_index + run_len, &my_control.bx, &my_control.by);
            if ((int8_t)((int16_t)my_control.bx - (int16_t)my_control.ax) == my_control.dx &&
                (int8_t)((int16_t)my_control.by - (int16_t)my_control.ay) == my_control.dy) {
                run_len++;
            }
            else {break;}
        }

        // 反向时距离取负 = 倒车
        // float dist = (float)run_len * BLOCK_DISTANCE;
        float dist = block_to_distance(run_len);
        if (task_state.cur_x == 2 && task_state.cur_y == 4 &&   // 距离校准，临时存在
            run_len == 4) {
            dist = dist + 4.0f;
        }
        if (dot == -1) {
            dist = -dist;
        }

        car_run_distance(MOVE_SPEED,0.0f,
                                 (float)dist);
        my_control.path_index += run_len;
        my_control.move_state = 1;

        // 刷新坐标
        task_state.cur_x += my_control.dx * run_len;
        task_state.cur_y += my_control.dy * run_len;
    }
}

// 到目标点最后一段道路运行
void final_road_move(void)
{
    // x轴和y轴的实际距离
    float x_dist, y_dist;

    // 最后一段：先转到要求车头 h
    int8_t hx = my_control.tx;
    int8_t hy = my_control.ty;

    // 无朝向要求的段，就当“正向走完”，h 取路径最后方向
    if (hx == 0 && hy == 0) {
        hx = my_control.last_dx;
        hy = my_control.last_dy;
    }

    // 设置目标角度
    my_control.target_yaw = track_dir_to_yaw(hx, hy);
    if (!track_yaw_ok(my_control.target_yaw)) {
        my_control.target_yaw = track_rotate_target(my_control.target_yaw);
        my_control.rotate_state = 1;
        // 转到目标姿态后 刷新车头朝向
        my_control.head_x = hx;
        my_control.head_y = hy;
        return;
    }
    if (car_is_busy()) return;

    // 地图位移（格）转车系位移
    // 车头 h=(hx,hy)，右侧=(hy,-hx)
    float map_dx = (float)(my_control.last_dx * my_control.final_run_len);
    float map_dy = (float)(my_control.last_dy * my_control.final_run_len);

    float car_x = map_dx * (float)hy - map_dy * (float)hx;   // 右向分量（sin）
    float car_y = map_dx * (float)hx + map_dy * (float)hy;   // 前向分量（cos）

    if (car_x >= 0) {
        x_dist = block_to_distance((uint8_t)car_x);
    }
    else {
        x_dist = -block_to_distance((uint8_t)-car_x);
    }
    if (car_y >= 0) {
        y_dist = block_to_distance((uint8_t)car_y);
    }
    else {
        y_dist = -block_to_distance((uint8_t)-car_y);
    }

    if ((x_dist > -0.5f && x_dist < 0.5f) && my_absf(y_dist) > 80.0f &&  my_absf(y_dist) < 160.0f) {    // 横移两格
        car_run_distance(MOVE_SPEED,CHASSIS_X_CALI,y_dist);
    }
    else if ((x_dist > -0.5f && x_dist < 0.5f) && my_absf(y_dist) > 160.0f) {   // 横移四格
        car_run_distance(MOVE_SPEED,CHASSIS_X_CALI * 2.0f,y_dist);
    }
    else {  // 默认
        car_run_distance(MOVE_SPEED,x_dist,y_dist);
    }

    my_control.move_state = 1;
    my_control.step = CALIBRATE_YAW_STEP;
}


// 目标状态获取 将摄像头所捕获的状态转化成车辆状态机
void target_state_get(void)
{
    if (maix_vision.ok == 1) {
        task_state.is_find_target = 1;
        task_state.follow_x = maix_vision.dx;
        task_state.follow_y = maix_vision.dy;
    }
    else {
        task_state.is_find_target = 0;
        task_state.follow_x = TARGET_DX;
        task_state.follow_y = TARGET_DY;
    }

    if (maix_result.stable == 1) {
        task_state.is_stable_target = 1;
    }
    else {
        task_state.is_stable_target = 0;
    }
}

// 确认是否稳定可抓取 1:可抓取
uint8_t catch_is_ok(void)
{
    static uint8_t stable_frame = 0;

    // 确认目标是否跟踪到可抓取位置附近 发现且稳定时
    if ((task_state.follow_x < TARGET_DX + 2 && task_state.follow_x > TARGET_DX - 2) &&
        (task_state.follow_y < TARGET_DY + 2 && task_state.follow_y > TARGET_DY - 2) &&
        task_state.is_find_target) {
        stable_frame += 1;
    }
    else {
        stable_frame = 0;
    }

    // 稳定20帧，即达到抓取条件
    if (stable_frame >= 20) {
        stable_frame = 0;
        task_state.is_catch_target = 1;
    }
    else {
        task_state.is_catch_target = 0;
    }

    return task_state.is_catch_target;
}

// 确认是否稳定可放置 1:可放置
uint8_t put_is_ok(void)
{
    static uint8_t stable_frame = 0;

    // 确认目标是否跟踪到可抓取位置附近 发现且稳定时
    if ((task_state.follow_x < TARGET_CIRCLE_DX + 2 && task_state.follow_x > TARGET_CIRCLE_DX - 2) &&
        (task_state.follow_y < TARGET_CIRCLE_DY + 2 && task_state.follow_y > TARGET_CIRCLE_DY - 2) &&
        task_state.is_find_target) {
        stable_frame += 1;
        }
    else {
        stable_frame = 0;
    }

    // 稳定20帧，即达到抓取条件
    if (stable_frame >= 20) {
        stable_frame = 0;
        task_state.is_catch_target = 1;
    }
    else {
        task_state.is_catch_target = 0;
    }

    return task_state.is_catch_target;
}

// 将当前需要抓取的颜色发给maix
void send_color_conduct(void)
{
    switch (task_state.color_buf[task_state.index])
    {
        case TASK_RED:
            maix_start_blob(MAIX_COLOR_RED);
            task_state.index += 1;
            break;
        case TASK_YELLOW:
            maix_start_blob(MAIX_COLOR_YELLOW);
            task_state.index += 1;
            break;
        case TASK_BLUE:
            maix_start_blob(MAIX_COLOR_BLUE);
            task_state.index += 1;
            break;
        case TASK_GREEN:
            maix_start_blob(MAIX_COLOR_GREEN);
            task_state.index += 1;
            break;
        case TASK_BLACK:
            maix_start_blob(MAIX_COLOR_BLACK);
            task_state.index += 1;
            break;
        case TASK_LOW_BLUE:
            maix_start_blob(MAIX_COLOR_LIGHTBLUE);
            task_state.index += 1;
            break;
        default:
            maix_start_blob(MAIX_COLOR_NONE);
            task_state.index += 1;
            break;
    }
    if (task_state.index > 5) {
        task_state.index = 0;
    }
}

// 将当前锁定的圆环编号发给maix
uint8_t put_pos_num = 0;
void send_circle_conduct(void)
{
    switch (put_pos_num)
    {
        case 0:
            maix_start_ring(MAIX_RING_1);
            break;
        case 1:
            maix_start_ring(MAIX_RING_2);
            break;
        case 2:
            maix_start_ring(MAIX_RING_3);
            break;
        default:
            break;
    }
}

// 将当前需要抓取的码垛发给maix
void send_onup_conduct(void)
{
    switch (task_state.color_buf[task_state.index])
    {
        case TASK_RED:
            maix_start_onup(MAIX_COLOR_RED);
            task_state.index += 1;
            break;
        case TASK_YELLOW:
            maix_start_onup(MAIX_COLOR_YELLOW);
            task_state.index += 1;
            break;
        case TASK_BLUE:
            maix_start_onup(MAIX_COLOR_BLUE);
            task_state.index += 1;
            break;
        case TASK_GREEN:
            maix_start_onup(MAIX_COLOR_GREEN);
            task_state.index += 1;
            break;
        case TASK_BLACK:
            maix_start_onup(MAIX_COLOR_BLACK);
            task_state.index += 1;
            break;
        case TASK_LOW_BLUE:
            maix_start_onup(MAIX_COLOR_LIGHTBLUE);
            task_state.index += 1;
            break;
        default:
            maix_start_onup(MAIX_COLOR_NONE);
            task_state.index += 1;
            break;
    }
    if (task_state.index > 5) {
        task_state.index = 0;
    }
}

// 追踪目标 物料 圆环
// 校准车身姿态 追踪x和y坐标
void follow_target(float target_yaw, int16_t target_x, int16_t target_y)
{
    Positional_PID_Cal(&yaw_pid, track_rotate_target(target_yaw), my_yaw);
    Positional_PID_Cal(&x_pid, target_x, task_state.follow_x);
    Positional_PID_Cal(&y_pid, target_y, task_state.follow_y);

    float omega_out = yaw_pid.output;
    float x_out = x_pid.output;
    float y_out = y_pid.output;

    car_speed_move((int16_t)(-y_out), (int16_t)(-x_out), (int16_t)(omega_out));
}


// 扫码获取任务
void scan_get_task(void)
{
    static uint8_t scan_step = 0;   // 0=发起 1=等待 2=重试 3=失败
    static uint8_t retry = 0;
    switch (scan_step)
    {
        case 0:                      // 到达扫码点，发起扫码
            qr_ready = 0;
            openmv_start = 1;        // 启动openmv 由openmv_work_task在主循环发出
            retry = 0;
            scan_step = 1;
            break;
        case 1:                      // 等待解析结果
            if (qr_ready == 1) {                 // 成功
                qr_ready  = 0;
                openmv_end = 1;                  // 停止扫码
                task_state.remindState = GREEN_REMIND;  // 扫码成功提示
                scan_finish = 1;                        // 扫码完成
                task_state.send_task_hmi_ok = 1;        // 将任务发送给串口屏
                my_control.step = CALIBRATE_YAW_STEP;   // 校准角度
                scan_step = 0;                          // 复位，供下一轮使用
            }
            else if (qr_ready == 2) {   // 超时/解析失败
                qr_ready = 0;
                if (++retry < 3) {
                    openmv_start = 1;   // 再来一次
                } else {
                    openmv_end = 1;
                    task_state.remindState = RED_REMIND;    // 报警
                    scan_finish = 1;                        // 扫码完成 直接默认了
                    task_state.send_task_hmi_ok = 1;        // 将任务发送给串口屏
                    my_control.step = CALIBRATE_YAW_STEP;
                    scan_step = 0;
                }
            }
            break;
        default:
            break;
    }

}

// 发送任务码置串口屏上 阻塞式 反正只发一遍
void scan_send_task_hmi(void)
{
    if (task_state.send_task_hmi_ok == 0) return;

    uint16_t color_1 = 0,color_2 = 0;
    uint16_t area_1 = 0,area_2 = 0;

    color_1 = task_state.color_buf_one[0] * 100 + task_state.color_buf_one[1] * 10 +
        task_state.color_buf_one[2] * 1;
    area_1 = task_state.area_buf_one[0] * 100 + task_state.area_buf_one[1] * 10 +
        task_state.area_buf_one[2] * 1;
    color_2 = task_state.color_buf_two[0] * 100 + task_state.color_buf_two[1] * 10 +
        task_state.color_buf_two[2] * 1;
    area_2 = task_state.area_buf_two[0] * 100 + task_state.area_buf_two[1] * 10 +
        task_state.area_buf_two[2] * 1;

    hmi_SetValue("color1",(int)(color_1));
    HAL_Delay(10);
    hmi_SetValue("area1",(int)(area_1));
    HAL_Delay(10);
    hmi_SetValue("color2",(int)(color_2));
    HAL_Delay(10);
    hmi_SetValue("area2",(int)(area_2));
    HAL_Delay(10);
    task_state.send_task_hmi_ok = 0;
}


// 抓取物料 原料区->转盘
uint8_t catch_step = 0;
void catch_material(void)
{
    static uint8_t serve_wait_time = 0;     // 舵机复位等待时间
    static uint8_t servo_reset_step = 0;    // 舵机复位步骤
    static uint8_t catch_num = 0;           // 当前抓取个数
    static float temp_yaw = 0.0f;

    // 不在原料区 退出
    if (task_state.task_location != MATERIAL) {
        return;
    }

    switch (catch_step)
    {
        case 0:     // 复位升降高度
            catch_finish = 0;
            catch_lift_to_height(LIFT_NORMAL);
            task_state.car_work = CATCHING; // 小车当前处于夹取物料状态
            catch_step = 1;
            break;
        case 1:     // 伸缩到抓取位置
            catch_stretch_to_pos(STRETCH_CATCH);
            catch_step = 2;
            break;
        case 2:     // 复位转盘、云台、夹爪三个舵机
            serve_wait_time++;
            if (serve_wait_time >= 10 && servo_reset_step == 0) {
                holder_rotate(HOLDER_CATCH);
                servo_reset_step = 1;
            }
            else if (serve_wait_time >= 20 && servo_reset_step == 1) {
                turntable_rotate(ONE_LOCATION);
                servo_reset_step = 2;
            }
            else if (serve_wait_time >= 30 && servo_reset_step == 2) {
                claw_ctrl(CLAW_LOOSE);
                servo_reset_step = 0;
                serve_wait_time = 0;
                catch_step = 3;
            }
            break;
        case 3:     // 给maix发送识别指令
            my_control.is_send_color = 1;
            temp_yaw = my_yaw;
            catch_stretch_to_pos(STRETCH_CATCH);
            catch_step = 4;
            break;
        case 4:     // 发现目标且目标稳定时 进入目标校准(第一次到达原料区)
            target_state_get(); // 获取状态
            if (task_state.is_find_target && task_state.is_stable_target == 1) {
                catch_step = 5;
            }
            break;
        case 5:     // 目标校准
            target_state_get(); // 获取状态
            if (task_state.is_find_target == 1) {   // 目标丢失 进入保护 回退步骤继续等待发现目标
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                follow_target(temp_yaw, TARGET_DX, TARGET_DY);
            }
            else {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                car_stop();
                catch_step = 4;
            }
            if (catch_is_ok() == 1) {  // 达到抓取条件
                car_stop();
                PID_Reset(&yaw_pid);    // 清除误差和积分
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                task_state.remindState = GREEN_REMIND;   // 闪灯提示
                catch_step = 6;
            }
            break;
        case 6:     // 下降高度准备抓取
            if (!lift_is_busy()) {
                catch_lift_to_height(LIFT_CATCH);
                catch_step = 7;
            }
            break;
        case 7:     // 夹爪夹住
            claw_ctrl(CLAW_CATCH);
            servo_wait_ms(CLAW_WAIT);  // 等待夹爪闭合完全
            catch_num += 1;
            catch_step = 8;
            break;
        case 8:     // 待夹爪夹取完全时 上升
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                switch (catch_num) {    // 根据当前已经抓取的个数，决定转盘位置
                    case 1:
                        turntable_rotate(ONE_LOCATION);
                        break;
                    case 2:
                        turntable_rotate(TWO_LOCATION);
                        break;
                    case 3:
                        turntable_rotate(THREE_LOCATION);
                        break;
                    default:
                        turntable_rotate(ONE_LOCATION);
                        break;
                }
                catch_step = 9;
            }
            break;
        case 9:     // 伸缩到合适放置位置
            catch_stretch_to_pos(STRETCH_PUT);
            catch_step = 10;
            break;
        case 10:    // 让云台转到合适放置位置
            holder_rotate(HOLDER_PUT);
            servo_wait_ms(HOLDER_WAIT);  // 等待云台转到合适放置位置
            catch_step = 11;
            break;
        case 11:    // 等待云台转到合适放置位置 下降高度放置
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_PUT);
                catch_step = 12;
            }
            break;
        case 12:    // 夹爪松开
            claw_ctrl(CLAW_LOOSE);
            servo_wait_ms(CLAW_WAIT);  // 等待夹爪开张完全
            catch_step = 13;
            break;
        case 13:    // 夹爪松开后 上升高度
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);   // 上升指令
                catch_step = 14;
            }
            break;
        case 14:
            holder_rotate(HOLDER_CATCH);    // 云台转回抓取位置
            servo_wait_ms(HOLDER_WAIT);  // 等待云台转到合适放置位置
            catch_step = 15;
            break;
        case 15:    // 等待云台归位
            if (!servo_is_busy()) {
                if (catch_num < 3) {    // 未抓完
                    catch_step = 3; // 回到给maix发指令，开始抓取下一个
                }
                else if (catch_num >= 3) {  // 已抓完
                    catch_step = 16;
                }
            }
            break;
        case 16:
            maix_stop();    // maix停止视觉任务
            catch_num = 0;
            serve_wait_time = 0;
            servo_reset_step = 0;
            task_state.car_work = RUNNING; // 小车结束夹取状态 去往下一个点位
            catch_finish = 1;
            catch_step = 0;
            my_control.step = CALIBRATE_YAW_STEP;   // 跳到校准角度步骤 校准完再离开
            holder_rotate(HOLDER_PUT);    // 云台转回放置位置
            break;
        default:
            break;
    }
}

// 放置物料 转盘->暂存区、粗加工区
uint8_t put_step = 0;
void put_material(void)
{
    static uint8_t serve_wait_time = 0;     // 舵机复位等待时间
    static uint8_t servo_reset_step = 0;    // 舵机复位步骤
    static uint8_t put_num = 0;             // 当前已放置个数
    static float temp_yaw = 0.0f;
    static uint16_t lose_time = 0;          // 等待超时时间

    // 不在粗加工区或暂存区 退出
    if (task_state.task_location != PROCESS && task_state.task_location != TEMPORARY) {
        return;
    }

    switch (put_step)
    {
        case 0:     // 复位升降高度
            put_finish = 0;
            catch_lift_to_height(LIFT_NORMAL);
            task_state.car_work = PUTTING;  // 小车当前处于夹取物料状态
            put_step = 1;
            break;
        case 1:     // 伸缩到放置位置
            catch_stretch_to_pos(STRETCH_PUT_MID);
            put_step = 2;
            break;
        case 2:     // 复位转盘、云台、夹爪三个舵机
            serve_wait_time++;
            if (serve_wait_time >= 10 && servo_reset_step == 0) {
                holder_rotate(HOLDER_CATCH);
                servo_reset_step = 1;
            }
            else if (serve_wait_time >= 20 && servo_reset_step == 1) {
                turntable_rotate(ONE_LOCATION);
                servo_reset_step = 2;
            }
            else if (serve_wait_time >= 30 && servo_reset_step == 2) {
                claw_ctrl(CLAW_LOOSE);
                servo_reset_step = 0;
                serve_wait_time = 0;
                put_step = 3;
            }
            temp_yaw = my_yaw;
            break;
        case 3:     // 先校准二号圆环位置
            maix_start_ring(MAIX_RING_2);
            put_step = 4;
            break;
        case 4:     // 跟踪2号圆环
            target_state_get(); // 获取状态
            if (task_state.is_find_target) {   // 发现目标 开始跟踪
                follow_target(temp_yaw, TARGET_CIRCLE_DX, TARGET_CIRCLE_DY);
                if (put_is_ok() == 1) {  // 达到放置条件
                    car_stop();
                    PID_Reset(&yaw_pid);    // 清除误差和积分
                    PID_Reset(&x_pid);
                    PID_Reset(&y_pid);
                    task_state.remindState = GREEN_REMIND;   // 闪灯提示
                    put_step = 5;
                }
            }
            else {  // 目标丢失 进入保护 回退步骤继续等待发现目标
                car_stop();
                PID_Reset(&yaw_pid);    // 清除误差和积分
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                if (++lose_time >= 800) { // 超时
                    lose_time = 0;
                    put_step = 3;   // 再发一遍
                }
            }
            break;
        case 5:
            car_run_distance(200, 0.0f, -NUM_DELTA_DIST);   // 先挪到一号圆环位置
            my_control.move_state = 1;
            put_step = 6;
            break;
        case 6:     // 给maix发送识别指令
            if (!task_state.one_finish && !task_state.two_finish) {
                my_control.is_send_circle = 1;
            }
            else if (task_state.one_finish && !task_state.two_finish) {
                my_control.is_send_circle = 1;
            }
            else if (task_state.one_finish && task_state.two_finish) {
                my_control.is_send_onup = 1;
            }
            temp_yaw = my_yaw;
            put_step = 7;
            break;
        case 7:     // 追踪目标
            target_state_get(); // 获取状态
            if (task_state.is_find_target) {   // 发现目标 开始跟踪
                follow_target(temp_yaw, TARGET_CIRCLE_DX, TARGET_CIRCLE_DY);
                if (put_is_ok() == 1) {  // 达到放置条件
                    switch (put_num)
                    {
                        case 0:
                            turntable_rotate(ONE_LOCATION);
                            break;
                        case 1:
                            turntable_rotate(TWO_LOCATION);
                            break;
                        case 2:
                            turntable_rotate(THREE_LOCATION);
                            break;
                        case 3:
                            turntable_rotate(SAFE_LOCATION);
                            break;
                        default:
                            break;
                    }
                    car_stop();
                    PID_Reset(&yaw_pid);    // 清除误差和积分
                    PID_Reset(&x_pid);
                    PID_Reset(&y_pid);
                    task_state.remindState = GREEN_REMIND;   // 闪灯提示
                    put_step = 8;
                }
            }
            else {  // 目标丢失 进入保护 回退步骤继续等待发现目标
                car_stop();
                PID_Reset(&yaw_pid);    // 清除误差和积分
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                if (++lose_time >= 800) { // 超时
                    lose_time = 0;
                    put_step = 6;   // 再发一遍
                }
            }
            break;
        case 8:     // 云台转回转盘处 开始放置物料
            holder_rotate(HOLDER_PUT);
            servo_wait_ms(HOLDER_WAIT);  // 等待云台完成转向
            put_step = 9;
            break;
        case 9:     // 等云台转完后
            if  (!servo_is_busy()) {
                // claw_ctrl(CLAW_LOOSE);  // 夹爪松开
                catch_stretch_to_pos(STRETCH_PUT);  // 伸缩到放置位置
                put_step = 10;
            }
            break;
        case 10:     // 下降高度
            catch_lift_to_height(LIFT_PUT);
            put_step = 11;
            break;
        case 11:     // 夹爪夹紧 抓物料
            claw_ctrl(CLAW_CATCH);  // 夹爪夹紧
            servo_wait_ms(CLAW_WAIT); // 等待夹爪完全夹住
            put_step = 12;
            break;
        case 12:     // 等待夹爪夹紧 升起
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                put_step = 13;
            }
            break;
        case 13:    // 云台转到放置处
            holder_rotate(HOLDER_CATCH);
            servo_wait_ms(HOLDER_WAIT); // 等云台转到目标位置
            catch_stretch_to_pos(STRETCH_PUT_MID);  // 伸缩到置放置中心圆环位置
            put_step = 14;
            break;
        case 14:    // 等云台到达目标位置 开始放置
            if (!servo_is_busy()) {
                if (task_state.one_finish && task_state.two_finish) {
                    catch_lift_to_height(LIFT_PUT_TWO); // 放置第二组
                }
                else {
                    catch_lift_to_height(LIFT_PUT_ONE); // 放置第一组
                }
                put_step = 15;
            }
            break;
        case 15:    // 夹爪松开
            claw_ctrl(CLAW_LOOSE);  // 夹爪松开
            servo_wait_ms(CLAW_WAIT); // 等待夹爪完全松开
            put_step = 16;
            break;
        case 16:    // 夹爪松开完全后
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                put_num += 1;
                put_pos_num = put_num;
                put_step = 17;
            }
            break;
        case 17:
            if (put_num < 3) {
                if (!car_is_busy()) {                       // 确认上一动作结束
                    car_run_distance(MOVE_SPEED, 0.0f, NUM_DELTA_DIST);
                    my_control.move_state = 1;              // 让顶部守卫等它走完
                    put_step = 6;
                }
            }
            else if (put_num >= 3) {
                if (!car_is_busy()) {
                    car_run_distance(MOVE_SPEED, 0.0f, -NUM_DELTA_DIST);
                    my_control.move_state = 1;
                    put_step = 18;
                }
            }
            break;
        case 18:
            maix_stop();    // maix停止视觉任务
            put_num = 0;
            put_pos_num = put_num;
            serve_wait_time = 0;
            servo_reset_step = 0;
            if (task_state.task_location == PROCESS) {    // 如果放在粗加工区
                task_state.car_work = PICKING; // 小车结束放置状态 把物料从粗加工区拿起
                my_control.step = PICK_MATERIAL_STEP;   // 跳到拿起物料步骤
                if ((!task_state.one_finish && !task_state.two_finish) ||
                    (task_state.one_finish && task_state.two_finish)) {
                    task_state.index = 0;
                }
                else if (task_state.one_finish && !task_state.two_finish) {
                    task_state.index = 3;
                }
            }
            else if (task_state.task_location == TEMPORARY) {   // 如果放在暂存区
                task_state.car_work = RUNNING; // 小车结束放置状态 去往下一个点位
                holder_rotate(HOLDER_PUT);    // 云台转回放置位置
                my_control.step = CALIBRATE_YAW_STEP;   // 跳到校准角度步骤 校准完再离开
                if (!task_state.one_finish) {
                    task_state.one_finish = 1;
                }
            }
            put_finish = 1;
            put_step = 0;
            break;
        default:
            break;
    }
}

// 拿起物料 暂存区、粗加工区->转盘
uint8_t pick_step = 0;
void pick_material(void)
{
    static uint8_t serve_wait_time = 0;     // 舵机复位等待时间
    static uint8_t servo_reset_step = 0;    // 舵机复位步骤
    static uint8_t pick_num = 0;            // 当前已拿起个数
    static float temp_yaw = 0.0f;
    static uint16_t lose_time = 0;          // 等待超时时间
    static uint16_t lock_time = 0;          // 等待超时时间

    // 不在粗加工区 退出
    if (task_state.task_location != PROCESS) {
        return;
    }

    switch (pick_step)
    {
        case 0:     // 复位升降高度
            pick_finish = 0;
            catch_lift_to_height(LIFT_NORMAL);
            task_state.car_work = PUTTING;  // 小车当前处于夹取物料状态
            pick_step = 1;
            break;
        case 1:     // 伸缩到放置位置
            catch_stretch_to_pos(STRETCH_PUT_MID);
            pick_step = 2;
            break;
        case 2:     // 复位转盘、云台、夹爪三个舵机
            serve_wait_time++;
            if (serve_wait_time >= 10 && servo_reset_step == 0) {
                holder_rotate(HOLDER_CATCH);
                servo_reset_step = 1;
            }
            else if (serve_wait_time >= 20 && servo_reset_step == 1) {
                turntable_rotate(ONE_LOCATION);
                servo_reset_step = 2;
            }
            else if (serve_wait_time >= 30 && servo_reset_step == 2) {
                claw_ctrl(CLAW_LOOSE);
                if (!car_is_busy()) {
                    servo_reset_step = 0;
                    serve_wait_time = 0;
                    pick_step = 3;
                }
            }
            break;
        case 3:     // 回到一号圆环
            car_run_distance(MOVE_SPEED,0.0f,-NUM_DELTA_DIST);
            my_control.move_state = 1;
            pick_step = 4;
            break;
        case 4:     // 给maix发送识别指令
            my_control.is_send_onup = 1;
            temp_yaw = my_yaw;
            catch_stretch_to_pos(STRETCH_PUT_MID);
            pick_step = 5;
            break;
        case 5:
            target_state_get(); // 获取状态
            if (task_state.is_find_target == 1) {   // 目标丢失 进入保护 回退步骤继续等待发现目标
                follow_target(temp_yaw, TARGET_DX, TARGET_DY);
                if (catch_is_ok() == 1) {  // 达到抓取条件
                    car_stop();
                    PID_Reset(&yaw_pid);   // 清除误差和积分
                    PID_Reset(&x_pid);
                    PID_Reset(&y_pid);
                    task_state.remindState = GREEN_REMIND;   // 闪灯提示
                    pick_step = 6;
                }
                else {
                    if (++lock_time >= 800) {  // 超时直接抓
                        car_stop();
                        PID_Reset(&yaw_pid);    // 清除误差和积分
                        PID_Reset(&x_pid);
                        PID_Reset(&y_pid);
                        task_state.remindState = RED_REMIND;   // 闪灯提示
                        lock_time = 0;
                        pick_step = 6;
                    }
                }
            }
            else {
                car_stop();
                if (++lose_time >= 800) { // 超时
                    lose_time = 0;
                    my_control.is_send_onup = 1;   // 重发同一个环（put_pos_num 没变，不会串环）
                }
            }
            break;
        case 6:     // 下降抓取
            if (!lift_is_busy()) {
                catch_lift_to_height(LIFT_PUT_ONE);
                pick_step = 7;
            }
            break;
        case 7:     // 夹爪抓取
            claw_ctrl(CLAW_CATCH);
            servo_wait_ms(200); // 等待200ms 让夹爪完全夹住
            pick_step = 8;
            break;
        case 8:     // 夹爪夹紧后 升高
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                pick_step = 9;
            }
            break;
        case 9:     // 伸缩到放置位置 云台转到放置位置
            catch_stretch_to_pos(STRETCH_PUT);
            holder_rotate(HOLDER_PUT);
            servo_wait_ms(HOLDER_WAIT); // 等待云台转到目标位置
            pick_step = 10;
            break;
        case 10:     // 待位置稳定
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_PUT); // 下降放置
                pick_step = 11;
            }
            break;
        case 11:     // 夹爪松开放置
            pick_num += 1;
            claw_ctrl(CLAW_LOOSE);
            servo_wait_ms(200);
            pick_step = 12;
            break;
        case 12:    // 夹爪松开后 上升
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                pick_step = 13;
            }
            break;
        case 13:    // 云台转回去
            holder_rotate(HOLDER_CATCH);
            servo_wait_ms(HOLDER_WAIT);
            pick_step = 14;
            break;
        case 14:    // 等云台转回去后 根据剩余物料判断是否继续
            if (!servo_is_busy()) {
                switch (pick_num)   // 根据已经放置的物料数判断转盘位置
                {
                    case 1:
                        turntable_rotate(TWO_LOCATION);
                        break;
                    case 2:
                        turntable_rotate(THREE_LOCATION);
                        break;
                    case 3:
                        turntable_rotate(SAFE_LOCATION);
                        break;
                    default:
                        break;
                }
                catch_stretch_to_pos(STRETCH_PUT_MID);  // 放置中心圆环位置
                pick_step = 15;
            }
            break;
        case 15:    // 根据已经放好的物料数决定是否结束
            if (pick_num < 3) {
                car_run_distance(MOVE_SPEED,0.0f,NUM_DELTA_DIST);  // 到达下一个圆环
                my_control.move_state = 1;
                pick_step = 4;
            }
            else if (pick_num >= 3) {
                car_run_distance(MOVE_SPEED,0.0f,-NUM_DELTA_DIST);   // 回到中心圆环
                my_control.move_state = 1;
                pick_step = 16;
            }
            break;
        case 16:    // 结束
            maix_stop();    // maix停止视觉任务
            pick_num = 0;
            serve_wait_time = 0;
            servo_reset_step = 0;
            if (task_state.task_location == PROCESS) {    // 在粗加工区
                task_state.car_work = RUNNING; // 小车结束拿起状态 去往下一个点位
                my_control.step = CALIBRATE_YAW_STEP;   // 跳到校准角度步骤 校准完再离开
                if (task_state.one_finish && !task_state.two_finish) {
                    task_state.two_finish = 1;
                }
            }
            holder_rotate(HOLDER_PUT);    // 云台转回放置位置
            pick_finish = 1;
            pick_step = 0;
            break;
        default:
            break;
    }
}

// 整套控制流程
void control_process(void)
{
    static int8_t wait_time = 80;

    // 位移进行中：等结束
    if (my_control.move_state == 1) {
        if (!car_is_busy()) my_control.move_state = 0;
        else return;
    }

    // 旋转进行中：car_rotate_yaw 必须每拍持续调用
    if (my_control.rotate_state == 1) {
        car_rotate_yaw(my_control.target_yaw);
        if (!car_is_busy()) my_control.rotate_state = 0;   // 内部到位稳定后已置 IDLE
        return;
    }

    // 升降装置进行中：等结束
    if (my_control.lift_state == 1) {
        if (!lift_is_busy()) {
            my_control.lift_state = 0;
        }
        return;
    }
    // 伸缩装置进行中：等结束
    if (my_control.stretch_state == 1) {
        if (!stretch_is_busy()) {
            my_control.stretch_state = 0;
        }
        return;
    }

    if (car_run && my_control.step >= 90) {
        my_control.step = 0;
        my_control.path_index = 0;
        my_control.move_state = 0;
        my_control.rotate_state = 0;
        my_control.lift_state = 0;
        my_control.stretch_state = 0;
        my_control.target_index = 0;
    }

    switch (my_control.step)
    {
        case 0:
            if (car_run) {
                hwt101_yaw_reset();
                if (start_location == 0) {
                    task_state.cur_x = 0;
                    task_state.cur_y = 0;
                }
                else if (start_location == 1) {
                    task_state.cur_x = 4;
                    task_state.cur_y = 0;
                }
                my_control.head_x = 1;
                my_control.head_y = 0;
                my_control.step = 1;
            }
            break;

        case 1:     // 等待一会儿
            wait_time --;
            if (wait_time <= 0) {
                wait_time = 80;
                my_control.step = LEAVE_PARKING_STEP;
            }
            break;

        case LEAVE_PARKING_STEP:    // 离开启停区
            leave_parking();
            break;

        case BACK_PARKING_STEP:     // 回到启停区
            back_parking();
            break;

        case TARGET_PARK_STEP:      // 规划去往启停区
            track_to_parking();
            break;

        case TARGET_SCAN_STEP:      // 规划去往扫码区
            track_to_scan();
            break;

        case TARGET_META_STEP:      // 规划去往原料区
            track_to_material();
            break;

        case TARGET_TEMP_STEP:      // 规划去往暂存区
            track_to_temporary();
            break;

        case TARGET_PROC_STEP:      // 规划去往粗加工区
            track_to_process();
            break;

        case CALIBRATE_YAW_STEP:    // 校准角度
            calibrate_yaw();
            break;

        case NORMAL_ROAD_MOVE_STEP: // 常规道路行驶
            normal_road_move();
            break;

        case FINAL_ROAD_MOVE_STEP:  // 最后一段道路行驶
            final_road_move();
            break;

        case SCAN_GET_TASK_STEP:    // 扫码获取任务
            scan_get_task();
            break;

        case CATCH_MATERIAL_STEP:   // 抓取物料
            catch_material();
            break;

        case PUT_MATERIAL_STEP:     // 放置物料
            put_material();
            break;

        case PICK_MATERIAL_STEP:    // 拿起物料
            pick_material();
            break;

        case 99:
        default:
            if (car_run) {
                car_stop();
            }
            car_run = false;
            my_control.step = 99;
            break;
    }
}
