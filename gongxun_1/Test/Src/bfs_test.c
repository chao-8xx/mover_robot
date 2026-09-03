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

