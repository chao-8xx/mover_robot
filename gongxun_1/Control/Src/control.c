#include "control.h"

#define TARGET_DX   0           // 目标dx
#define TARGET_DY   0           // 目标dy

#define LEAVE_PARK_DX   11.0f   // 离开启停区的横向距离
#define LEAVE_PARK_DY   13.0f   // 离开启停区的纵向距离

#define MOVE_SPEED   250        // 底盘非任务区运行速度

// 结构体定义
Control_Variate my_control;

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


// 校准物料、圆环
// 需设置目标角度，以便于校准时陀螺仪调整姿态
void calibrate_target(float target_yaw)
{
    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    Positional_PID_Cal(&x_pid,TARGET_DX,maix_result.dx);
    Positional_PID_Cal(&y_pid,TARGET_DY,maix_result.dy);
    Positional_PID_Cal(&yaw_pid,target_yaw, my_yaw);

    car_speed_move((int16_t)(-y_pid.output),(int16_t)(-x_pid.output),(int16_t)(yaw_pid.output));
}

// 追踪物料 原料区使用，伸缩装置与车身配合
// 需设置目标角度，以便于追踪时陀螺仪调整姿态
void follow_target(float target_yaw)
{
    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    Positional_PID_Cal(&x_pid,TARGET_DX,maix_result.dx);
    Positional_PID_Cal(&y_pid,TARGET_DY,maix_result.dy);
    Positional_PID_Cal(&follow_pid,TARGET_DY,maix_result.dy);
    Positional_PID_Cal(&yaw_pid,target_yaw, my_yaw);

    // 防止超限导致堵转
    if (task_state.cur_stretch_dist + (-follow_pid.output) < MAX_STRETCH_DIST &&
        task_state.cur_stretch_dist + (-follow_pid.output) > 0.0f) {
        catch_stretch_pos(-follow_pid.output);
    }
    // 移动跟踪
    car_speed_move((int16_t)(-y_pid.output),(int16_t)(-x_pid.output),(int16_t)(yaw_pid.output));
}


// 离开启停区 步骤2
void leave_parking(void)
{
    if (start_location == 0 && task_state.task_location == PARKING_2) {    // 启停区2 (0,0)
        car_run_distance(my_control.speed, -LEAVE_PARK_DX, LEAVE_PARK_DX);
        my_control.move_state = 1;
    }
    else if (start_location == 1 && task_state.task_location == PARKING_1) {   // 启停区1 (4,0)
        car_run_distance(my_control.speed, -LEAVE_PARK_DX, -LEAVE_PARK_DX);
        my_control.move_state = 1;
    }
    my_control.step = my_control.target_pos[my_control.target_index];
}

// 回到启停区 步骤3
void back_parking(void)
{
    if (start_location == 0) {   // 启停区2 (0,0)
        car_run_distance(my_control.speed, LEAVE_PARK_DX, -LEAVE_PARK_DX);
        my_control.move_state = 1;
    }
    else if (start_location == 1) {  // 启停区1 (4,0)
        car_run_distance(my_control.speed, LEAVE_PARK_DX, LEAVE_PARK_DX);
        my_control.move_state = 1;
    }
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
    switch (my_control.target_pos[my_control.target_index])
    {
        case TARGET_PARK_STEP:      // 启停区
            if (start_location == 0) {
                task_state.cur_x = 0;
                task_state.cur_y = 0;
                task_state.task_location = PARKING_2;
            }
            else if (start_location == 1) {
                task_state.cur_x = 4;
                task_state.cur_y = 0;
                task_state.task_location = PARKING_1;
            }
            break;
        case TARGET_SCAN_STEP:      // 扫码区
            task_state.cur_x = 2;
            task_state.cur_y = 0;
            task_state.task_location = SCAN;
            break;
        case TARGET_META_STEP:      // 原料区
            task_state.cur_x = 4;
            task_state.cur_y = 2;
            task_state.task_location = MATERIAL;
            break;
        case TARGET_TEMP_STEP:      // 暂存区
            task_state.cur_x = 2;
            task_state.cur_y = 4;
            task_state.task_location = TEMPORARY;
            break;
        case TARGET_PROC_STEP:      // 粗加工区
            task_state.cur_x = 0;
            task_state.cur_y = 2;
            task_state.task_location = PROCESS;
            break;
        default:
            task_state.task_location = ROAD;
            break;
    }

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

    my_control.target_index += 1;
    if (my_control.target_index < TARGET_STEP_LEN) {
        my_control.step = my_control.target_pos[my_control.target_index];
        remindState = GREEN_REMIND;
    }
    else {
        my_control.step = BACK_PARKING_STEP;
        remindState = BLUE_REMIND;
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
        float dist = (float)run_len * BLOCK_DISTANCE;
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

    car_run_distance(MOVE_SPEED,car_x * BLOCK_DISTANCE,car_y * BLOCK_DISTANCE);
    my_control.move_state = 1;
    my_control.step = CALIBRATE_YAW_STEP;
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

    if (car_run && my_control.step >= 90) {
        my_control.step = 0;
        my_control.path_index = 0;
        my_control.move_state = 0;
        my_control.rotate_state = 0;
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

        case 99:
        default:
            car_stop();
            car_run = false;
            my_control.step = 99;
            break;
    }
}
