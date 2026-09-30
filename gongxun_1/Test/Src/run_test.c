#include "headfile.h"

// 底盘绕场一周测试 10ms中断
void chassis_run_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();
    // step_motor_init();
    // my_hwt101_init();
    // chassis_init();
    // HAL_Delay(100);

    static uint16_t work_time = 250;
    static uint8_t step = 0;
    int16_t speed = 120;

    switch (step)
    {
        case 0:   // 开始前等待
            work_time--;
            if (work_time <= 0) {
                work_time = 250;
                step = 1;
            }
            break;

        case 1:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 2;
            }
            break;
        case 2:   // 旋转 -90°（必须持续调用）
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-90.0f);
                if (!car_is_busy()) {
                    step = 3;
                }
            }
            break;
        case 3:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 4;
            }
            break;
        case 4:   // 旋转 -180°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-180.0f);
                if (!car_is_busy()) {
                    step = 5;
                }
            }
            break;
        case 5:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 6;
            }
            break;
        case 6:   // 旋转 -270°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(-270.0f);
                if (!car_is_busy()) {
                    step = 7;
                }
            }
            break;
        case 7:   // 直行200
            if (!car_is_busy()) {
                car_run_distance(speed, 0.0f, 200.0f);
                step = 8;
            }
            break;
        case 8:   // 转回 0°
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(0.0f);
                if (!car_is_busy()) {
                    step = 9;
                }
            }
            break;
        case 9:
            car_stop();
            break;
        default:
            car_stop();
            step = 9;
            break;
    }
}

// 用BFS算法让车子到达目标坐标处
void bfs_track_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();       // 定时器初始化
    // my_hwt101_init();   // 陀螺仪
    // my_hmi_init();      // 串口屏
    // step_motor_init();  // 闭环步进电机
    // HAL_Delay(100);     // 等待一会儿
    // chassis_init();     // 底盘装置
    // HAL_Delay(100);     // 等待一会儿
    // bfs_init();

    static int16_t  test_speed = 200;
    static uint8_t  step = 0;
    static uint8_t  path_index = 0;
    static uint8_t  cur_x = 0, cur_y = 0;
    static uint8_t  next_x = 0, next_y = 0;
    static float    heading_deg = 0.0f;   // 累计目标偏航角
    static float    dir_deg = 0.0f;       // 当前朝向对应的地图角度
    static float    target_dir_deg = 0.0f;
    static uint8_t run_len = 0;   // 当前连续直行的格子数
    static int8_t dx = 0;
    static int8_t dy = 0;

    static uint16_t work_time = 200;

    uint8_t len;

    switch (step)
    {
        case 0:     // 等待一会儿
            work_time --;
            if (work_time <= 0) {
                work_time = 200;
                step = 1;
            }
            break;
        case 1:   // 规划一次路径
            bfs_set_obstacle(1, 1, 1);
            bfs_set_obstacle(3, 1, 1);
            bfs_set_obstacle(1, 3, 1);
            bfs_set_obstacle(3, 3, 1);
            if (bfs_find_path(0, 0, 4, 4) == 1)
            {
                path_index = 1;            // 第0步是起点
                cur_x = 0;
                cur_y = 0;
                heading_deg = 0.0f;        // 上电陀螺仪已复位
                dir_deg = 0.0f;            // 初始朝地图 y 正
                step = 2;
            }
            else
            {
                step = 99;
            }
            break;

        case 2:   // 检查是否走完
            len = bfs_get_path_len();
            if (path_index >= len)
            {
                step = 98;
                break;
            }
            bfs_get_path_step(path_index - 1, &cur_x, &cur_y);
            bfs_get_path_step(path_index, &next_x, &next_y);

            // 计算当前格到下一格的地图绝对方向
            dx = (int8_t)(next_x - cur_x);
            dy = (int8_t)(next_y - cur_y);

            if (dx == 1  && dy == 0) target_dir_deg = 90.0f;    // 右
            else if (dx == -1 && dy == 0) target_dir_deg = 270.0f; // 左
            else if (dx == 0 && dy == 1) target_dir_deg = 0.0f;    // 上(y正)
            else if (dx == 0 && dy == -1) target_dir_deg = 180.0f; // 下
            else
            {
                step = 99;   // 路径不连续，异常
                break;
            }

            // 方向没变就直接走；变了就进入旋转
            if (target_dir_deg != dir_deg)
            {
                // 计算最短转角差
                float delta = target_dir_deg - dir_deg;
                if (delta > 180.0f)  delta -= 360.0f;
                if (delta <= -180.0f) delta += 360.0f;

                heading_deg += delta;   // 目标偏航角累加
                dir_deg = target_dir_deg;
                step = 3;
            }
            else
            {
                step = 4;
            }
            break;

        case 3:   // 旋转：每拍持续调用 car_rotate_yaw
            if (car_get_state() != CHASSIS_MOVE) {
                car_rotate_yaw(heading_deg);
                if (!car_is_busy())
                {
                    step = 4;
                }
            }
            break;

        case 4:   // 车头已对准，前进
            if (!car_is_busy())
            {
                // 从当前 path_index 往后数，同方向能连续走几格
                run_len = 1;
                len = bfs_get_path_len();
                while (path_index + run_len < len)
                {
                    uint8_t ax, ay, bx, by;
                    bfs_get_path_step(path_index + run_len - 1, &ax, &ay);
                    bfs_get_path_step(path_index + run_len,&bx,&by);
                    int8_t ddx = (int8_t)((int16_t)bx - (int16_t)ax);
                    int8_t ddy = (int8_t)((int16_t)by - (int16_t)ay);
                    if (ddx == dx && ddy == dy) run_len++;
                    else break;
                }

                // 一次直行 run_len 格，中间不停
                car_run_distance(test_speed, 0.0f,
                                 (float)run_len * BLOCK_DISTANCE);
                step = 5;
            }
            break;

        case 5:   // 等走完，更新坐标
            if (!car_is_busy())
            {
                path_index += run_len;   // 原来是一格一格 +1，现在整段跳过
                step = 2;
            }
            break;

        case 98:  // 到达终点
            car_stop();
            car_run = false;
            step = 98;
            break;

        case 99:  // 无路或异常
        default:
            car_stop();
            car_run = false;
            step = 99;
            break;
    }
}

// 起点 -> (0,0) -> (4,2) -> (2,4) -> (0,2) -> (0,0) -> 起点
// 10ms中断
// 返回 (dx,dy) 地图方向对应的目标偏航角
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
    return (d > -1.0f && d < 1.0f);
}
// 把目标角折算到离当前 my_yaw 最近的角度，确保 car_rotate_yaw 走最短路径
static float track_rotate_target(float target_yaw)
{
    float delta = target_yaw - my_yaw;
    while (delta > 180.0f)  delta -= 360.0f;
    while (delta <= -180.0f) delta += 360.0f;
    return my_yaw + delta;
}

// BFS 多点测试：左下角 -> (0,0) -> (4,2)朝南 -> (2,4)朝西 -> (0,2)朝北 -> (0,0) -> 左下角
// 10ms 中断直接调用；障碍由 HMI 的 BB 指令设置，路径自动避开
void bfs_multi_track_test(void)
{
    static const int16_t test_speed = 160;

    static const uint8_t leg_tx[4] = {4, 2, 0, 0};
    static const uint8_t leg_ty[4] = {2, 4, 2, 0};
    static const int8_t  leg_hx[4] = {0, 1, 0, 1};
    static const int8_t  leg_hy[4] = {-1, 0, 1, 0};

    static uint8_t step = 0;
    static uint8_t leg_idx = 0;
    static uint8_t path_len = 0;
    static uint8_t path_index = 0;
    static uint8_t run_start = 0;
    static uint8_t final_run_len = 0;
    static uint8_t cur_x = 0, cur_y = 0;
    static int8_t  last_dx = 0, last_dy = 0;
    static uint8_t sub = 0;
    static uint8_t moving = 0;       // 1=等待位移完成
    static uint8_t rotating = 0;     // 1=旋转闭环进行中
    static float   rotate_target = 0.0f;

    uint8_t ax, ay, bx, by;
    int8_t dx, dy;
    float target_yaw;

    // 位移进行中：等结束
    if (moving) {
        if (!car_is_busy()) moving = 0;
        else return;
    }

    // 旋转进行中：car_rotate_yaw 必须每拍持续调用
    if (rotating) {
        car_rotate_yaw(rotate_target);
        if (!car_is_busy()) rotating = 0;   // 内部到位稳定后已置 IDLE
        return;
    }

    if (car_run && step >= 90) {
        step = 0;
        leg_idx = 0;
        path_index = 0;
        sub = 0;
        moving = 0;
        rotating = 0;
    }

    switch (step)
    {
        case 0:   // 等发车键，车头先朝东放好
            if (car_run) {
                hwt101_yaw_reset();
                cur_x = 0;
                cur_y = 0;
                step = 1;
            }
            break;

        case 1:   // 从左下角斜挪到 (0,0) 中心
            car_run_distance(test_speed, -13.5f, 13.5f);
            moving = 1;
            step = 2;
            break;

        case 2:   // 规划当前段（BFS 自动避障）
            if (bfs_find_path(cur_x, cur_y,
                              leg_tx[leg_idx], leg_ty[leg_idx]) == 0) {
                step = 99;
                break;
            }
            path_len = bfs_get_path_len();
            path_index = 1;

            if (path_len < 2) {
                step = 5;
                break;
            }

            // 找最后一段同方向直线
            bfs_get_path_step(path_len - 2, &ax, &ay);
            bfs_get_path_step(path_len - 1, &bx, &by);
            last_dx = (int8_t)((int16_t)bx - (int16_t)ax);
            last_dy = (int8_t)((int16_t)by - (int16_t)ay);

            run_start = path_len - 1;
            while (run_start > 1) {
                bfs_get_path_step(run_start - 2, &ax, &ay);
                bfs_get_path_step(run_start - 1, &bx, &by);
                if ((int8_t)((int16_t)bx - (int16_t)ax) == last_dx &&
                    (int8_t)((int16_t)by - (int16_t)ay) == last_dy) {
                    run_start--;
                } else {
                    break;
                }
            }
            final_run_len = path_len - run_start;

            step = 3;
            break;

        case 3:   // 普通段：转好头后连续直行
            if (path_index >= run_start) {
                step = 4;
                break;
            }

            bfs_get_path_step(path_index - 1, &ax, &ay);
            bfs_get_path_step(path_index, &bx, &by);
            dx = (int8_t)((int16_t)bx - (int16_t)ax);
            dy = (int8_t)((int16_t)by - (int16_t)ay);
            target_yaw = track_dir_to_yaw(dx, dy);

            if (!track_yaw_ok(target_yaw)) {
                rotate_target = track_rotate_target(target_yaw);
                rotating = 1;
                return;
            }
            if (car_is_busy()) break;

            {
                uint8_t run_len = 1;
                while (path_index + run_len < run_start) {
                    bfs_get_path_step(path_index + run_len - 1, &ax, &ay);
                    bfs_get_path_step(path_index + run_len, &bx, &by);
                    if ((int8_t)((int16_t)bx - (int16_t)ax) == dx &&
                        (int8_t)((int16_t)by - (int16_t)ay) == dy) {
                        run_len++;
                    } else {
                        break;
                    }
                }
                car_run_distance(test_speed, 0.0f,
                                 (float)run_len * BLOCK_DISTANCE);
                path_index += run_len;
                moving = 1;
            }
            break;

        case 4:   // 最后一段：转到要求车头后进点
        {
            // 最后一段：先转到要求车头 h
            int8_t hx = leg_hx[leg_idx];
            int8_t hy = leg_hy[leg_idx];

            // 无朝向要求的段，就当“正向走完”，h 取路径最后方向
            if (hx == 0 && hy == 0) {
                hx = last_dx;
                hy = last_dy;
            }

            target_yaw = track_dir_to_yaw(hx, hy);
            if (!track_yaw_ok(target_yaw)) {
                rotate_target = track_rotate_target(target_yaw);
                rotating = 1;
                return;
            }
            if (car_is_busy()) break;

            // 地图位移（格）转车系位移
            // 车头 h=(hx,hy)，右侧=(hy,-hx)
            float map_dx = (float)(last_dx * final_run_len);
            float map_dy = (float)(last_dy * final_run_len);

            float car_x = map_dx * (float)hy - map_dy * (float)hx;   // 右向分量（sin）
            float car_y = map_dx * (float)hx + map_dy * (float)hy;   // 前向分量（cos）

            car_run_distance(test_speed,
                             car_x * BLOCK_DISTANCE,
                             car_y * BLOCK_DISTANCE);
            moving = 1;
            step = 5;
            break;
        }

        case 5:   // 本段到点，校准一下角度，进入下一段
            cur_x = leg_tx[leg_idx];
            cur_y = leg_ty[leg_idx];
            if (leg_hx[leg_idx] != 0 || leg_hy[leg_idx] != 0) {
                target_yaw = track_dir_to_yaw(leg_hx[leg_idx], leg_hy[leg_idx]);
                if (!track_yaw_ok(target_yaw)) {
                    rotate_target = track_rotate_target(target_yaw);
                    rotating = 1;
                    return;
                }
            }

            leg_idx++;
            if (leg_idx < 4) {
                step = 2;
                task_state.remindState = GREEN_REMIND;
            } else {
                step = 6;
                task_state.remindState = BLUE_REMIND;
            }
            break;

        case 6:   // 从 (0,0) 回左下角
            if (sub == 0) {
                target_yaw = 0.0f;
                if (!track_yaw_ok(target_yaw)) {
                    rotate_target = track_rotate_target(target_yaw);
                    rotating = 1;
                    return;
                }
                if (!car_is_busy()) {
                    car_run_distance(test_speed, 13.0f, -13.0f);
                    moving = 1;
                    sub = 2;
                }
            }
            else {
                task_state.remindState = RED_REMIND;
                car_stop();
                car_run = false;
                step = 98;
            }
            break;

        case 99:
        default:
            car_stop();
            car_run = false;
            step = 99;
            break;
    }
}

// 小车麦轮跟踪锁定物料测试
void car_follow_material_test(void)
{
    // 用于主循环和中断外 即初始化部分
    // cam_init(&huart1);          // OpenMV maixcam 摄像头
    // HAL_Delay(1000);
    // holder_rotate(HOLDER_CATCH);
    // HAL_Delay(1000);
    // maix_start_blob(MAIX_COLOR_RED);

    // 放入主循环
    // cam_task();                     // 解析摄像头数据

    // static uint8_t time = 0;
    // 更新物料发现状态
    // if (maix_vision.ok == 1) {
    //     time ++;
    //     task_state.is_find_target = 1;
    //     task_state.follow_x = maix_vision.dx;
    //     task_state.follow_y = maix_vision.dy;
    //     if (time >= 200) {
    //         task_state.remindState = GREEN_REMIND;
    //     }
    // }
    // else {
    //     time ++;
    //     task_state.is_find_target = 0;
    //     task_state.follow_x = 0;
    //     task_state.follow_y = 0;
    //     if (time >= 200) {
    //         task_state.remindState = RED_REMIND;
    //     }
    //
    // }
    // 更新物料发现状态
    target_state_get();
    // 追踪物料
    if (task_state.is_find_target == 1 && task_state.is_stable_target == 1) {
        follow_target(0.0f,(int16_t)0,(int16_t)0);
        buzzer_off();
    }
    else {
        car_stop();
        buzzer_off();
    }
}

// 抓取颜色指令 主循环调用
uint8_t color_send = 0;
uint8_t color_num = 0;
void catch_color_send(void)
{
    if (!color_send) {
        return;
    }
    switch (color_num) {
        case 1:
            maix_start_blob(MAIX_COLOR_RED);
            HAL_Delay(10);
            break;
        case 2:
            maix_start_blob(MAIX_COLOR_GREEN);
            HAL_Delay(10);
            break;
        case 3:
            maix_start_blob(MAIX_COLOR_BLUE);
            HAL_Delay(10);
            break;
        default:
            break;
    }
    color_send = 0;
}

// 物料识别跟踪并抓取测试   if (car_run)
void car_follow_catch_material_test(void)
{
    // 用于主循环和中断外 即初始化部分
    // system_init();
    // HAL_Delay(1000);

    // 放入主循环
    // cam_task();                     // 解析摄像头数据
    // catch_color_send();

    static uint8_t test_step = 0;       // 步骤
    static uint8_t catch_num = 0;       // 已经抓取的个数
    static uint8_t serve_wait_time = 0;     // 舵机复位等待时间
    static uint8_t servo_reset_step = 0;    // 舵机复位步骤

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

    switch (test_step)
    {
        case 0:     // 复位升降高度
            catch_lift_to_height(LIFT_NORMAL);
            test_step = 1;
            break;
        case 1:     // 伸缩到抓取位置
            catch_stretch_to_pos(STRETCH_CATCH);
            test_step = 2;
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
                test_step = 3;
            }
            break;
        case 3:     // 给maix发送识别指令
            color_num += 1;
            color_send = 1;
            test_step = 4;
            break;
        case 4:     // 追踪物料
            car_follow_material_test();
            if (catch_is_ok() == 1) {
                car_stop();
                PID_Reset(&yaw_pid);    // 清除误差和积分
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                test_step = 5;
            }
            break;
        case 5:     // 下放夹爪抓取
            if (!lift_is_busy()) {
                catch_lift_to_height(LIFT_CATCH);
                if (task_state.cur_lift_height == LIFT_CATCH) {
                    test_step = 6;         // 确认已下发才往下走
                }
            }
            break;
        case 6:     // 夹爪夹住
            claw_ctrl(CLAW_CATCH);
            servo_wait_ms(700);  // 等待700ms 使其夹爪闭合完全
            catch_num += 1;
            test_step = 7;
            break;
        case 7:
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
                test_step = 8;
            }
            break;
        case 8:     // 伸缩到合适放置位置
            catch_stretch_to_pos(STRETCH_PUT);
            test_step = 9;
            break;
        case 9:    // 让云台转到合适放置位置
            holder_rotate(HOLDER_PUT);
            servo_wait_ms(1000);  // 等待1000ms 使其云台转到合适放置位置
            test_step = 10;
            break;
        case 10:    // 上升
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_PUT);
                test_step = 11;
            }
            break;
        case 11:    // 夹爪松开
            claw_ctrl(CLAW_LOOSE);
            servo_wait_ms(700);  // 等待700ms 使其夹爪开张完全
            test_step = 12;
            break;
        case 12:    // 夹爪松开后 上升高度
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);   // 上升指令
                test_step = 13;
            }
            break;
        case 13:    // 夹爪松开后 上升高度
            holder_rotate(HOLDER_CATCH);    // 云台转回抓取位置
            servo_wait_ms(1000);  // 等待1000ms 使其云台转到合适放置位置
            test_step = 14;
            break;
        case 14:    // 等待云台归位
            if (!servo_is_busy()) {
                if (catch_num < 3) {    // 未抓完
                    test_step = 3; // 回到给maix发指令，开始抓取下一个
                }
                else if (catch_num >= 3) {  // 已抓完
                    test_step = 15;
                }
            }
            break;
        case 15:
            catch_num = 0;
            color_num = 0;
            serve_wait_time = 0;
            servo_reset_step = 0;
            test_step = 0;
            car_run = false;
            break;
        default:
            break;
    }
}

// 放置圆环指令 主循环调用
uint8_t material_put_send = 0;
uint8_t circle_num = 0;
void put_circle_send(void)
{
    if (!material_put_send) {
        return;
    }
    switch (circle_num) {
        case 1:
            maix_start_ring(MAIX_RING_1);
            HAL_Delay(10);
            break;
        case 2:
            maix_start_ring(MAIX_RING_2);
            HAL_Delay(10);
            break;
        case 3:
            maix_start_ring(MAIX_RING_3);
            HAL_Delay(10);
            break;
        default:
            break;
    }
    material_put_send = 0;
}

// 物料放置测试
void car_follow_put_material_test(void)
{
    // 用于主循环和中断外 即初始化部分
    // system_init();
    // HAL_Delay(1000);

    // 放入主循环
    // cam_task();                     // 解析摄像头数据
    // put_circle_send();

    static uint8_t test_step = 0;           // 步骤
    static uint8_t put_num = 0;             // 已经放置的个数
    static uint8_t serve_wait_time = 0;     // 舵机复位等待时间
    static uint8_t servo_reset_step = 0;    // 舵机复位步骤

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

    switch (test_step)
    {
        case 0:     // 复位升降高度
            catch_lift_to_height(LIFT_NORMAL);
            test_step = 1;
            break;
        case 1:     // 伸缩到抓取位置
            catch_stretch_to_pos(STRETCH_PUT_MID);
            test_step = 2;
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
                test_step = 3;
            }
            break;
        case 3:     // 给maix发送识别指令
            circle_num += 1;
            material_put_send = 1;
            switch (put_num)    // 转盘转到对应位置
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
            test_step = 4;
            break;
        case 4:     // 追踪圆环
            car_follow_material_test();
            if (catch_is_ok() == 1) {
                car_stop();
                PID_Reset(&yaw_pid);    // 清除误差和积分
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                test_step = 5;
            }
            break;
        case 5:     // 云台转回转盘处 开始放置物料
            holder_rotate(HOLDER_PUT);
            servo_wait_ms(1000);  // 等待1000ms 待云台完成转向
            test_step = 6;
            break;
        case 6:     // 等云台转完后
            if  (!servo_is_busy()) {
                // claw_ctrl(CLAW_LOOSE);  // 夹爪松开
                catch_stretch_to_pos(STRETCH_PUT);  // 伸缩到放置位置
                test_step = 7;
            }
            break;
        case 7:     // 下降高度
            catch_lift_to_height(LIFT_PUT);
            test_step = 8;
            break;
        case 8:     // 夹爪夹紧 抓物料
            claw_ctrl(CLAW_CATCH);  // 夹爪夹紧
            servo_wait_ms(600); // 等待600ms 夹爪完全夹住
            test_step = 9;
            break;
        case 9:     // 等待夹爪夹紧 升起
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                test_step = 10;
            }
            break;
        case 10:    // 云台转到放置处
            holder_rotate(HOLDER_CATCH);
            servo_wait_ms(1000); // 等待1000ms 待云台转到目标位置
            catch_stretch_to_pos(STRETCH_PUT_MID);  // 伸缩到置放置中心圆环位置
            test_step = 11;
            break;
        case 11:    // 等云台到达目标位置 开始放置
            if (!servo_is_busy()) {
                if (task_state.one_finish && task_state.two_finish) {
                    catch_lift_to_height(LIFT_PUT_TWO); // 放置第二组
                }
                else {
                    catch_lift_to_height(LIFT_PUT_ONE); // 放置第一组
                }
                test_step = 12;
            }
            break;
        case 12:    // 夹爪松开
            claw_ctrl(CLAW_LOOSE);  // 夹爪松开
            servo_wait_ms(600); // 等待600ms 待夹爪完全松开
            test_step = 13;
            break;
        case 13:    // 夹爪松开完全后
            if (!servo_is_busy()) {
                catch_lift_to_height(LIFT_NORMAL);
                put_num += 1;
                test_step = 14;
            }
            break;
        case 14:
            if (put_num < 3) {
                if (!car_is_busy()) {                       // 确认上一动作结束
                    car_run_distance(200, 0.0f, 12.0f);
                    my_control.move_state = 1;              // 让顶部守卫等它走完
                    test_step = 3;
                }
            }
            else if (put_num >= 3) {
                if (!car_is_busy()) {
                    car_run_distance(200, 0.0f, -12.0f);
                    my_control.move_state = 1;
                    test_step = 15;
                }
            }
            break;
        case 15:
            test_step = 0;
            circle_num = 0;
            put_num = 0;
            serve_wait_time = 0;
            servo_reset_step = 0;
            car_run = false;
            break;
        default:
            break;
    }
}
