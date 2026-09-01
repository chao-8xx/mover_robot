#include "test.h"

// 虚拟串口 USB CDC 测试函数，验证 CDC 是否正常工作
// 现已变为无线串口 串口六输出了
void cdc_test(void) {
    // led_toggle();         // LED 闪烁表示程序在跑
    // printf("Hello from CDC!\r\n");
    printf("hello world\r\n");
    HAL_Delay(1000);
}

// openmv扫码测试 key1 按下后开始扫码，扫码结果通过无线串口输出
void openmv_qr_test(void)
{
    openmv_send_start_scan();
    printf("OpenMV QR test start\r\n");

    while (1)
    {
        openmv_qr_task();

        if (key_getlongpress() == 1)
        {
            openmv_send_stop_scan();
            // led_off();
            printf("OpenMV QR test stop\r\n");
            break;
        }

        HAL_Delay(10);
    }
}

/*
// 之前的openmv测试代码
void openmv_qr_test_legacy(void) {
    // key
    // void key_handle(void)
    // {
    //   uint8_t shortpress = key_getshortpress ();
    //   uint8_t longpress = key_getlongpress ();
    //
    //   if (shortpress != 0 )
    //   {
    //     switch(shortpress )
    //     {
    //       case 1:
    //         led_off();
    //         openmv_send_start_scan();
    //         break ;
    //     }
    //   }
    //   if (longpress != 0 )
    //   {
    //     switch(longpress )
    //     {
    //       // case 1:
    //       //  led_tick ();
    //     }
    //   }
    // }

    //  主函数
    //   while (1)
    //   {
    //     key_handle();
    //     openmv_qr_task();
    //   }
}
*/

// maixcam测试函数，按键1开始识别物料，按键1长按停止识别，识别结果通过虚拟串口CDC输出
void maix_blob_test(void)
{
    uint8_t started = 0;
    maix_vision_t vision;

    while (1)
    {
        if (!started)
        {
            led_on();
            maix_start_blob(MAIX_COLOR_GREEN);
            printf("Maix blob test start\r\n");
            started = 1;
        }

        if (maix_get_result(&vision))
        {
            printf("Maix result: cmd=%02X target=%02X ok=%d dx=%d dy=%d tick=%lu\r\n",
                   vision.cmd, vision.target, vision.ok,
                   vision.dx, vision.dy, vision.tick);
        }

        if (key_getlongpress() == 1)
        {
            led_off();
            maix_stop();
            printf("Maix blob test stop\r\n");
            break;
        }

        HAL_Delay(10);
    }
}

// LiDAR 测试函数，按键1开始扫描，按键1长按停止扫描，扫描结果通过虚拟串口CDC输出
void lidar_test(void) {
    printf("================================\r\n");
    // 遍历输出 0-359 度的雷达数据
    for(int i = 0; i < 360; i++)
    {
        // 格式对齐：@后面跟3位角度，5位距离
        printf("@%3d %5d \r\n", i, lidar_data[i]);
    }
    printf("================================\r\n");
}

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

// 提示装置测试代码 5ms定时中断
void remind_test(void)
{
    // 初始化部分 放入主函数
    // timer_init();
    // remind_init();

    static uint16_t work_time = 600;
    static uint8_t step = 0;

    switch (step)
    {
        case 0:
            remindState = NO_REMIND;
            step = 1;
            break;
        case 1:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = RED_REMIND;
                step = 2;
            }
            break;
        case 2:
            if (remindState == NO_REMIND) {
                step = 3;
            }
            break;
        case 3:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = GREEN_REMIND;
                step = 4;
            }
            break;
        case 4:
            if (remindState == NO_REMIND) {
                step = 5;
            }
            break;
        case 5:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = BLUE_REMIND;
                step = 6;
            }
            break;
        case 6:
            if (remindState == NO_REMIND) {
                step = 7;
            }
            break;
        case 7:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = YELLOW_REMIND;
                step = 8;
            }
            break;
        case 8:
            if (remindState == NO_REMIND) {
                step = 9;
            }
            break;
        case 9:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = PINK_REMIND;
                step = 10;
            }
            break;
        case 10:
            if (remindState == NO_REMIND) {
                step = 11;
            }
            break;
        case 11:
            work_time --;
            if (work_time <= 0) {
                work_time = 600;
                remindState = WHITE_REMIND;
                step = 12;
            }
            break;
        case 12:
            step = 1;
            break;
        default:
            break;
    }

    remind_control();
}

// bfs路径规划测试（无实物，仅串口打印路径）
// 放主函数while外
void bfs_test(void)
{
    uint8_t x, y;

    // 1. 初始化地图
    bfs_init();

    // 2. 设几个障碍（比如中间挡路）
    bfs_set_obstacle(4, 0, 1);
    bfs_set_obstacle(0, 4, 1);
    bfs_set_obstacle(3, 3, 1);
    bfs_set_obstacle(3, 1, 1);
    bfs_set_obstacle(1, 3, 1);
    bfs_set_obstacle(1, 1, 1);

    // 3. 规划路径
    printf("BFS: (0,0) -> (3,4)\r\n");
    printf("obstacle: (4,0) (0,4) (3,3) (3,1) (1,3) (1,1)\r\n");
    printf("X  *  *  *  T\r\n");
    printf("*  X  *  X  *\r\n");
    printf("*  *  *  *  *\r\n");
    printf("*  X  *  X  *\r\n");
    printf("O  *  *  *  X\r\n");

    uint32_t time = HAL_GetTick();

    if (bfs_find_path(0, 0, 4, 4) == 1)
    {
        uint32_t now = HAL_GetTick() - time;
        printf("use_ms: %d \r\n", (int)now);    // 打印耗时

        printf("found, len=%d\r\n", bfs_get_path_len());

        // 4. 把整条路径打出来
        for (uint8_t i = 0; i < bfs_get_path_len(); i++)
        {
            bfs_get_path_step(i, &x, &y);
            printf("step %d -> (%d,%d)\r\n", i, x, y);
        }
    }
    else
    {
        printf("no path\r\n");
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
    static const int16_t test_speed = 120;

    static const uint8_t leg_tx[4] = {4, 2, 0, 0};
    static const uint8_t leg_ty[4] = {2, 4, 2, 0};
    static const int8_t  leg_hx[4] = {0, 1, 0, 0};
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
            car_run_distance(test_speed, -12.5f, 12.5f);
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
            float map_dx = (float)last_dx * final_run_len;
            float map_dy = (float)last_dy * final_run_len;

            float car_x = map_dx * hy - map_dy * hx;   // 右向分量（sin）
            float car_y = map_dx * hx + map_dy * hy;   // 前向分量（cos）

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
            } else {
                step = 6;
            }
            break;

        case 6:   // 从 (0,0) 回左下角
            if (sub == 0) {
                target_yaw = -180.0f;
                if (!track_yaw_ok(target_yaw)) {
                    rotate_target = track_rotate_target(target_yaw);
                    rotating = 1;
                    return;
                }
                if (!car_is_busy()) {
                    car_run_distance(test_speed, 0.0f, 12.5f);
                    moving = 1;
                    sub = 1;
                }
            } else if (sub == 1) {
                target_yaw = -90.0f;
                if (!track_yaw_ok(target_yaw)) {
                    rotate_target = track_rotate_target(target_yaw);
                    rotating = 1;
                    return;
                }
                if (!car_is_busy()) {
                        car_run_distance(test_speed, 0.0f, 12.5f);
                    moving = 1;
                    sub = 2;
                }
            } else {
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
