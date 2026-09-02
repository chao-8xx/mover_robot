//
// Created by mechreuo on 2026/9/2.
//

#include "headfile.h"


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
    // timer_init();
    // step_motor_init();
    // my_hwt101_init();
    // chassis_init();
    // HAL_Delay(100);

    static int16_t  test_speed = 120;
    static uint8_t  step = 0;
    static uint8_t  path_index = 0;
    static uint8_t  cur_x = 0, cur_y = 0;
    static uint8_t  next_x = 0, next_y = 0;
    static float    heading_deg = 0.0f;   // 累计目标偏航角
    static float    dir_deg = 0.0f;       // 当前朝向对应的地图角度
    static float    target_dir_deg = 0.0f;

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
            bfs_init();
            bfs_set_obstacle(4, 0, 1);  // 需要障碍就加
            bfs_set_obstacle(0, 4, 1);
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
            bfs_get_path_step(path_index, &next_x, &next_y);

            // 计算当前格到下一格的地图绝对方向
            int8_t dx = (int8_t)(next_x - cur_x);
            int8_t dy = (int8_t)(next_y - cur_y);

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

        case 4:   // 车头已对准，前进一格
            if (!car_is_busy())
            {
                car_run_distance(test_speed, 0.0f, BLOCK_DISTANCE);
                step = 5;
            }
            break;

        case 5:   // 等走完，更新坐标
            if (!car_is_busy())
            {
                cur_x = next_x;
                cur_y = next_y;
                path_index++;
                step = 2;
            }
            break;

        case 98:  // 到达终点
            car_stop();
            step = 98;
            break;

        case 99:  // 无路或异常
        default:
            car_stop();
            step = 99;
            break;
    }
}
