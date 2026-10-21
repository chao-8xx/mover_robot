#include "headfile.h"


static uint8_t map_test_fail = 0;  // 本轮未通过的测试项数量 0表示全部通过

// 输出一项检查结果  ok非0表示通过 name是串口上显示的测试名称
// 多个条件用&&连接 表示这些条件必须全部满足 任何一个失败则本项FAIL
static void map_test_check(uint8_t ok, const char *name) {
    if (ok) printf("[MAP TEST] PASS %s\r\n", name);
    else {
        printf("[MAP TEST] FAIL %s\r\n", name);
        map_test_fail++;  // 记录失败项数量 继续运行后面的检查，最后统一汇总
    }
}


// PASS表示当前这项软件规则通过 不代表已经完成真实雷达和实车避障验证
uint8_t map_stage_test(void) {
    uint8_t cell_x, cell_y, road_id;       // 5x5区域下标和道路编号
    map_cell_t cell;                      // 当前区域的类型和编号
    map_node_t node;                      // 当前节点的地图坐标 单位mm
    map_obs_t obs_a = {0}, obs_b = {0}, read_obs;  // 两个测试障碍和读回的数据
    road_state_t state;                   // 读回的道路状态 未知、可通行、封锁
    map_path_t old_path, new_path;         // 封路前的路径和重新搜索的路径

    // 模拟R8道路上的一个20mm*20mm障碍框 所有坐标单位都是mm
    // 六个值依次是：中心X、中心Y、最小X、最小Y、最大X、最大Y
    map_scan_obs_t box = {1200.0f, 1625.0f, 1190.0f, 1615.0f,
                          1210.0f, 1635.0f};

    map_test_fail = 0;  // 每次调用先清除上一轮的失败数量

    ///////////////////////////////////// 1 坐标与区域 ////////////////////////////////////
    // 毫米坐标(1200,1625)应落在区域下标(2,3)，该区域应是道路R8
    // coordinate to R8：检查地图坐标能否找到正确道路
    map_test_check(map_xy_to_cell(1200.0f, 1625.0f, &cell_x, &cell_y) &&
                   cell_x == 2 && cell_y == 3 &&
                   map_cell_get(cell_x, cell_y, &cell) &&
                   cell.cell_type == cell_road && cell.cell_id == 8,
                   "coordinate to R8");
    // field and forbidden area：检查禁入区和场地边界
    // (750,750)在禁入区；X=2400越界；X=0且Y=1200仍属于场地内正常区域
    map_test_check(map_check_area(750.0f, 750.0f) == map_forbidden &&
                   map_check_area(2400.0f, 1200.0f) == map_outside &&
                   map_check_area(0.0f, 1200.0f) == map_normal,
                   "field and forbidden area");
    // N4 and R8 topology：中心节点N4在(1200,1200)，N1与N4之间由R8连接
    map_test_check(map_node_get(4, &node) && node.x == 1200.0f &&
                   node.y == 1200.0f &&
                   map_road_find(1, 4, &road_id) && road_id == 8,
                   "N4 and R8 topology");

    ///////////////////////////////////// 2 遍历整张静态地图 //////////////////////////////
    uint8_t node_seen[MAP_NODE_COUNT] = {0};  // 每个节点编号在25个区域里出现的次数
    uint8_t road_seen[MAP_ROAD_COUNT] = {0};  // 每条道路编号在25个区域里出现的次数
    uint8_t forbidden_count = 0;             // 禁入区域数量 应为4
    uint8_t all_cells_ok = 1;                // 整张地图是否检查通过
    // X、Y各有5个区域带 选每个区域带的中心坐标，组合检查全部25个区域
    const float band_center[5] = {275.0f, 775.0f, 1200.0f, 1625.0f, 2125.0f};
    for (uint8_t y = 0; y < MAP_CELL_HEIGHT; y++) {
        for (uint8_t x = 0; x < MAP_CELL_WIDTH; x++) {
            if (!map_xy_to_cell(band_center[x], band_center[y], &cell_x, &cell_y) ||
                cell_x != x || cell_y != y || !map_cell_get(x, y, &cell)) {
                all_cells_ok = 0;  // 坐标转换失败、下标不符或区域读取失败
                continue;
            }
            if (cell.cell_type == cell_node && cell.cell_id < MAP_NODE_COUNT)
                node_seen[cell.cell_id]++;
            else if (cell.cell_type == cell_road && cell.cell_id < MAP_ROAD_COUNT)
                road_seen[cell.cell_id]++;
            else if (cell.cell_type == cell_forbidden && cell.cell_id == MAP_NO_ID)
                forbidden_count++;
            else all_cells_ok = 0;
        }
    }
    // 9个节点和12条道路应各出现一次 不能漏编号，也不能重复编号
    for (uint8_t i = 0; i < MAP_NODE_COUNT; i++)
        if (node_seen[i] != 1) all_cells_ok = 0;
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++)
        if (road_seen[i] != 1) all_cells_ok = 0;
    // all 25 cells and unique IDs：25个区域对应9节点、12道路、4禁入区
    map_test_check(all_cells_ok && forbidden_count == 4,
                   "all 25 cells and unique IDs");

    ///////////////////////////////////// 3 障碍物存储 ////////////////////////////////////
    map_data_init();  // 道路全部重置为未知 清空障碍物记录
    obs_a.state = obs_unknown;  // 未确认的候选障碍
    // 用两个不同位置的点状障碍检查数组存取 此处不经过雷达或跨圈确认
    obs_a.center_x = obs_a.min_x = obs_a.max_x = 500.0f;
    obs_a.center_y = obs_a.min_y = obs_a.max_y = 500.0f;
    obs_b = obs_a;
    obs_b.center_x = obs_b.min_x = obs_b.max_x = 600.0f;
    // clear one obstacle only：写入0号、1号记录，删除0号后1号应仍保持X=600
    map_test_check(map_obs_set(0, &obs_a) && map_obs_set(1, &obs_b) &&
                   map_obs_clear(0) && map_obs_get(1, &read_obs) &&
                   read_obs.center_x == 600.0f,
                   "clear one obstacle only");

    ///////////////////////////////////// 4 地图初始化与未知道路 //////////////////////////
    // 按区域封路不再填写膨胀参数；初始化清空道路、扫描历史和变更次数
    map_update_init();
    map_test_check(map_update_is_ready() && map_update_change_get() == 0 &&
                   map_road_get(8, &state) && state == road_unknown,
                   "map initialization clears state");
    // unknown road policy：N1到N4的直达道路是R8，此时R8还是未知
    // allow允许把未知路纳入候选路径，但进路决策仍须扫描；stop禁止搜索未知路
    map_test_check(map_plan_find(1, 4, map_unknown_allow, &old_path) &&
                   old_path.road_count == 1 && old_path.roads[0] == 8 &&
                   map_plan_road_state(&old_path, 0, 102) == map_step_scan &&
                   !map_plan_find(1, 4, map_unknown_stop, &new_path),
                   "unknown road policy");
    // start equals goal：起点终点都是N1时，只需要1个节点，不需要经过任何道路
    map_test_check(map_plan_find(1, 1, map_unknown_stop, &new_path) &&
                   new_path.node_count == 1 && new_path.road_count == 0 &&
                   new_path.nodes[0] == 1,
                   "start equals goal");
    map_road_set(8, road_free);  // 手动模拟已经确认R8可通行 不是根据无障碍自动判断
    // 手动FREE可参与搜索，但没有本次有效观察许可时仍需扫描
    map_test_check(map_plan_find(1, 4, map_unknown_stop, &new_path) &&
                   new_path.road_count == 1 &&
                   map_plan_road_state(&new_path, 0, 102) == map_step_scan,
                   "stored free still needs observation");
    map_road_set(8, road_unknown);  // 恢复未知状态 给后续跨圈确认测试使用

    ///////////////////////////////////// 5 跨圈确认并封路 ////////////////////////////////
    // map_update_scan参数依次为：障碍框数组、框数量、扫描编号、时间戳ms
    // 下面的1、2、3、4是模拟扫描编号；100、200等是模拟时间，不会等待真实时间
    // first scan and duplicate scan：第1圈只给R8累计一次；重复提交第1圈必须拒绝
    map_test_check(map_update_scan(&box, 1, 1, 100, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   !map_update_scan(&box, 1, 1, 101, 1),
                   "first scan and duplicate scan");
    // miss resets road confirmation：第2圈没有障碍，R8的连续次数清零
    // 第3圈重新看到只算第一次，不能把不连续的两圈凑成确认
    // 参数(0,0,2,200)表示第2圈有效但没有障碍框，不表示雷达断流
    map_test_check(map_update_scan(0, 0, 2, 200, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 300, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_change_get() == 0,
                   "miss resets road confirmation");
    // consecutive scans block R8：第3、4圈连续看到才封路，只记录一次道路变化
    map_test_check(map_update_scan(&box, 1, 4, 400, 1) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "consecutive scans block R8");
    // other roads and blocked persistence：R0保持未知；第5圈没看到障碍也不自动解封R8
    map_test_check(map_road_get(0, &state) && state == road_unknown &&
                   map_update_scan(0, 0, 5, 500, 1) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "other roads and blocked persistence");
    // blocked road causes replan：旧直达路径失效，进入R8的决策必须是停止
    // 允许未知路参与搜索时应能绕行；新路径经过多条道路且检查有效
    // 找到候选绕行路径不代表未知道路已获准通行 进入前仍需扫描确认
    map_test_check(!map_plan_check(&old_path, map_unknown_allow) &&
                   map_plan_road_state(&old_path, 0, 102) == map_step_stop &&
                   map_plan_find(1, 4, map_unknown_allow, &new_path) &&
                   new_path.road_count > 1 &&
                   map_plan_check(&new_path, map_unknown_allow),
                   "blocked road causes replan");

    ///////////////////////////////////// 6 同圈去重与漏圈 ////////////////////////////////
    // 同一圈里有几个框都只给道路累计一次，不要求这些框彼此接近
    map_update_init();  // 重置地图 这一组测试从空地图开始
    map_scan_obs_t same_scan[2] = {box, box};
    same_scan[1].center_y += 150.0f; // 第二个框也在R8上，与第一个框相隔150mm
    same_scan[1].min_y += 150.0f;
    same_scan[1].max_y += 150.0f;
    map_update_scan(same_scan, 2, 1, 100, 1);
    // one scan cannot confirm twice：单圈两个框也只能累计一次，R8保持未知
    map_test_check(map_road_get(8, &state) && state == road_unknown &&
                   map_update_change_get() == 0,
                   "one scan cannot confirm twice");
    // road confirmation needs no object matching：下一圈换个位置，与之前两个框都相距超过120mm
    // 本版不配对物体，只要连续两圈影响R8就确认这条道路
    map_scan_obs_t shifted_box = box;
    shifted_box.center_y -= 150.0f;
    shifted_box.min_y -= 150.0f;
    shifted_box.max_y -= 150.0f;
    map_test_check(map_update_scan(&shifted_box, 1, 2, 200, 1) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "road confirmation needs no object matching");

    // skipped scan resets confirmation：第1圈直接跳到第3圈，也必须重新累计
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_test_check(map_update_scan(&box, 1, 3, 300, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 4, 400, 1) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "skipped scan resets confirmation");

    // long pause resets confirmation：编号相邻但间隔超过1秒，也不能沿用上次次数
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_test_check(map_update_scan(&box, 1, 2, 1200, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 1300, 1) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "long pause resets confirmation");

    // clear restarts confirmation and keeps blocked：接收/位姿中断时清零；已经封路则保留
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_update_clear_candidates();
    map_test_check(map_update_scan(&box, 1, 2, 200, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 300, 1) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "clear restarts road confirmation");
    map_update_clear_candidates();
    map_test_check(map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "clear keeps blocked roads");

    ///////////////////////////////////// 7 节点堵塞与无路可走 ////////////////////////////
    // 把模拟障碍移到中心节点N4，该节点区域的障碍会封锁相连的四个出口
    box.center_x = 1200.0f;
    box.center_y = 1200.0f;
    box.min_x = 1190.0f;
    box.max_x = 1210.0f;
    box.min_y = 1190.0f;
    box.max_y = 1210.0f;
    map_update_init();  // 清除上一组的障碍和道路状态
    map_update_scan(&box, 1, 1, 100, 1);
    map_update_scan(&box, 1, 2, 200, 1);
    uint8_t center_blocked = 1;  // 四条道路是否全部封锁
    const uint8_t center_roads[4] = {2, 3, 8, 9};  // N4左、右、上、下连接的道路编号
    for (uint8_t i = 0; i < 4; i++) {
        if (!map_road_get(center_roads[i], &state) || state != road_blocked)
            center_blocked = 0;
    }
    // N4 four exits：四个出口均封锁，道路状态变化次数应为4
    map_test_check(center_blocked && map_update_change_get() == 4,
                   "N4 four exits");
    // N4 has no path：即使允许搜索未知路，也无法到达出口全堵的N4，结果路径应清空
    map_test_check(!map_plan_find(1, 4, map_unknown_allow, &new_path) &&
                   new_path.node_count == 0,
                   "N4 has no path");

    ///////////////////////////////////// 8 道路区域整体封锁 //////////////////////////////
    // 此点偏离R0中心线，但仍位于R0区域；本版不尝试从障碍旁边挤过去
    box.center_x = box.min_x = box.max_x = 775.0f;
    box.center_y = box.min_y = box.max_y = 1915.0f;
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_update_scan(&box, 1, 2, 200, 1);
    map_test_check(map_road_get(0, &state) && state == road_blocked,
                   "off-center road obstacle blocks whole road");

    ///////////////////////////////////// 9 边缘障碍与显式放行 ////////////////////////////
    // 中心虽然在禁区，但框本身已经伸入R8区域，不能只凭中心位置过滤
    box = (map_scan_obs_t){990.0f, 1625.0f, 980.0f, 1615.0f, 1010.0f, 1635.0f};
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_update_scan(&box, 1, 2, 200, 1);
    map_test_check(map_road_get(8, &state) && state == road_blocked,
                   "forbidden center still affects road");

    // 完全在禁区内、没有伸入道路或节点的框，不再通过膨胀影响旁边道路
    box = (map_scan_obs_t){775.0f, 1840.0f, 775.0f, 1840.0f, 775.0f, 1840.0f};
    map_update_init();
    map_update_scan(&box, 1, 1, 100, 1);
    map_update_scan(&box, 1, 2, 200, 1);
    map_test_check(map_road_get(0, &state) && state == road_unknown,
                   "forbidden-only obstacle does not block road");

    // 空扫描仍不能自动放行；这里用预设数据模拟上层已经确认R8整段观察有效
    // confirm_free参数：待进入道路编号、本次扫描编号、当前时间ms
    map_update_init();
    map_test_check(!map_update_confirm_free(8, 1, 100) &&
                   map_update_scan(0, 0, 1, 100, 1) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   !map_update_confirm_free(8, 0, 101) &&
                   !map_update_confirm_free(8, 1, 1101),
                   "free needs latest fresh observation");
    map_test_check(map_update_confirm_free(8, 1, 101) &&
                   map_update_confirm_free(8, 1, 102) &&
                   map_update_change_get() == 1 &&
                   map_plan_find(1, 4, map_unknown_stop, &new_path) &&
                   map_plan_road_state(&new_path, 0, 102) == map_step_go,
                   "explicit free allows GO once");
    map_update_clear_candidates();
    map_test_check(!map_update_confirm_free(0, 1, 103),
                   "interruption rejects old free observation");

    // 已放行道路后来连续两圈受影响，仍应封锁；移走障碍也不能用放行接口直接解封
    box = (map_scan_obs_t){1200.0f, 1625.0f, 1190.0f, 1615.0f, 1210.0f, 1635.0f};
    map_update_scan(&box, 1, 2, 200, 1);
    map_test_check(!map_update_confirm_free(8, 2, 201) &&
                   map_plan_road_state(&new_path, 0, 201) == map_step_scan,
                   "one hit rejects free confirmation");
    map_update_scan(&box, 1, 3, 300, 1);
    map_update_scan(0, 0, 4, 400, 1);
    map_test_check(!map_update_confirm_free(8, 4, 401) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 2,
                   "explicit free cannot reopen blocked road");

    ///////////////////////////////////// 10 输入容量、边界和结果完整性 ////////////////////
    map_scan_obs_t many_boxes[MAP_SCAN_OBS_MAX];
    for (uint8_t i = 0; i < MAP_SCAN_OBS_MAX; i++) many_boxes[i] = box;
    map_update_init();
    map_test_check(map_update_scan(many_boxes, MAP_SCAN_OBS_MAX, 1, 100, 1) &&
                   map_update_scan(many_boxes, MAP_SCAN_OBS_MAX, 2, 200, 1) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "scan capacity independent of old records");
    map_test_check(map_xy_to_cell(1000.0f, 750.0f, &cell_x, &cell_y) &&
                   map_cell_get(cell_x, cell_y, &cell) && cell.cell_type == cell_road &&
                   cell.cell_id == 9 && map_check_area(1000.0f, 750.0f) == map_normal,
                   "boundary queries agree");

    // complete=0仍可累计看到的障碍，但没有框也不能证明道路畅通
    map_update_init();
    map_test_check(map_update_scan(0, 0, 1, 100, 0) &&
                   !map_update_confirm_free(8, 1, 101) &&
                   map_update_scan(&box, 1, 2, 200, 0) &&
                   map_update_scan(&box, 1, 3, 300, 0) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "incomplete result blocks but cannot free");

    // 旧FREE只用于保存地图；进入前必须使用未过期、且与最新扫描对应的许可
    map_update_init();
    map_update_scan(0, 0, 1, 100, 1);
    map_update_confirm_free(8, 1, 101);
    map_plan_find(1, 4, map_unknown_stop, &new_path);
    map_test_check(map_plan_road_state(&new_path, 0, 102) == map_step_go &&
                   map_plan_road_state(&new_path, 0, 1101) == map_step_scan &&
                   map_update_scan(0, 0, 2, 200, 1) &&
                   map_plan_road_state(&new_path, 0, 201) == map_step_scan &&
                   map_update_confirm_free(8, 2, 201) &&
                   map_plan_road_state(&new_path, 0, 202) == map_step_go,
                   "GO needs fresh observation approval");

    ///////////////////////////////////// 11 输出最终结果 /////////////////////////////////
    // ALL PASS：全部检查通过；FAILED：至少一项失败，向上查对应的FAIL名称
    // 测试结束后保留最后一组测试地图 正式接雷达前应重新按实际参数初始化地图
    printf("[MAP TEST] %s\r\n", map_test_fail ? "FAILED" : "ALL PASS");
    return map_test_fail == 0;  // 返回1表示全部通过，返回0表示有失败项
}


///////////////////////////////////// 真实雷达桌面联调 ////////////////////////////////////
// 本测试使用真实扫描和聚类，但位姿是人为设置的测试值
// 临时把雷达原点当作测试车体原点，0度射线当作车头方向，不使用实际安装标定参数
// 测试位姿设在N1=(1200,2125)，转180度后，雷达0度方向指向地图中心N4
// 这样雷达前方500mm的物体会落在地图(1200,1625)，即刚才自检用的R8中间
// 这不是编码器定位结果 正式地图仍须通过map_obstacle_task接入可信位姿
#define MAP_LIDAR_TEST_X         1200.0f
#define MAP_LIDAR_TEST_Y         2125.0f
#define MAP_LIDAR_TEST_YAW         180.0f

// 桌面测试观察框：原始雷达0度方向前方400~600mm，左右各200mm
// 映射到上述固定测试位姿后，对应地图X=1000~1400、Y=1525~1725
// 只接收整个包围框都落在这里的聚类，避免桌边、近处物体把多个出口一起封锁
// 这是独立测试的选区 不是正式避障过滤规则，也不代表选区外没有障碍
#define MAP_LIDAR_TEST_MIN_X     1000.0f
#define MAP_LIDAR_TEST_MAX_X     1400.0f
#define MAP_LIDAR_TEST_MIN_Y     1525.0f
#define MAP_LIDAR_TEST_MAX_Y     1725.0f

// 打印当前候选路径 例如N1 -R8-> N4，表示从N1经R8到N4
// entry是第一条道路的进入决策 SCAN=还需扫描确认，GO=可通行，STOP=停止
static void map_lidar_test_print_path(const map_path_t *path) {
    if (path->node_count == 0) {
        printf("[LIDAR MAP] NO PATH\r\n");
        return;
    }

    printf("[LIDAR MAP] PATH N%u", (unsigned int)path->nodes[0]);
    for (uint8_t i = 0; i < path->road_count; i++) {
        printf(" -R%u-> N%u", (unsigned int)path->roads[i],
               (unsigned int)path->nodes[i + 1]);
    }
    map_step_t step = map_plan_road_state(path, 0, HAL_GetTick());
    printf(" entry=%s\r\n", step == map_step_go ? "GO" :
                            step == map_step_scan ? "SCAN" : "STOP");
}

// 未选中小柱时每2秒打印一次筛选前数据，只诊断，不改变阈值、点云或地图
static void map_lidar_test_diagnose(uint32_t tick) {
    static uint32_t last_tick = 0;
    if (tick - last_tick < 2000U) return;
    last_tick = tick;

    const lidar_scan_t *scan = lidar_get_scan();
    if (scan != NULL) {
        // 每项为 角度=距离/原始有效/点云有效；355~359、0~5覆盖原始0度两侧
        // 原始有效但点云无效时检查屏蔽和距离过滤；少于3个有效点时不会生成聚类
        printf("[PILLAR RAW] raw_scan=%lu cloud_scan=%lu",
               (unsigned long)scan->scan_num, (unsigned long)cloud.scan_num);
        for (int16_t offset = -5; offset <= 5; offset++) {
            uint16_t angle = (uint16_t)((offset + LIDAR_ANGLE_COUNT) % LIDAR_ANGLE_COUNT);
            printf(" %u=%u/%u/%u", (unsigned int)angle, (unsigned int)scan->distance[angle],
                   (unsigned int)scan->valid[angle], (unsigned int)cloud.points[angle].valid);
        }
        printf("\r\n");
    }

    // 打印全部普通聚类，包含未通过筛选的组，避免目标恰好排在第8组以后而看不到
    // pillar=尺寸/点数/距离筛选是否通过；area=整个包围框是否在桌面观察区内
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        uint8_t in_area = cluster->min_x >= MAP_LIDAR_TEST_MIN_X && cluster->max_x <= MAP_LIDAR_TEST_MAX_X &&
                          cluster->min_y >= MAP_LIDAR_TEST_MIN_Y && cluster->max_y <= MAP_LIDAR_TEST_MAX_Y;
        printf("[PILLAR CHECK] C%u a=%u~%u p=%u span=%.1f near=%.0f center=(%.0f,%.0f) pillar=%u area=%u\r\n",
               (unsigned int)i, (unsigned int)cluster->start_angle, (unsigned int)cluster->end_angle,
               (unsigned int)cluster->point_count, fmaxf(cluster->width, cluster->height),
               cluster->nearest_distance, cluster->center_point.x, cluster->center_point.y,
               (unsigned int)obstacle_cluster_is_pillar(cluster), (unsigned int)in_area);
    }
}

// 主循环持续调用 本函数不发送底盘运动指令，不把未知道路直接标为可通行
// 上电先清空测试地图，再连续执行：小柱候选及观察框筛选 -> 实际点判路 -> 跨圈确认 -> 封路重规划
// 已确认的封锁不会因移走挡板自动解除 要重新做一轮完整实验，请复位开发板
void map_lidar_test(void) {
    static uint8_t test_started = 0;
    static uint32_t last_print_tick = 0;
    static uint32_t last_change = 0;
    static uint32_t last_drop = 0;  // 接收错误打断扫描后，清零道路连续确认次数
    static map_path_t path;
    static uint16_t stat_scans = 0, stat_hits = 0; // 每100个接受的新扫描统计一次选区内候选出现情况
    static uint16_t stat_min_points = UINT16_MAX, stat_max_points = 0;
    static float stat_max_size = 0.0f;
    static const map_rect_t test_area = {
        .min_x = MAP_LIDAR_TEST_MIN_X, .max_x = MAP_LIDAR_TEST_MAX_X,
        .min_y = MAP_LIDAR_TEST_MIN_Y, .max_y = MAP_LIDAR_TEST_MAX_Y
    };
    const lidar_cluster_t *selected[CLUSTER_MAX_COUNT]; // 只保存打印用的指针，不复制点云
    uint8_t count = 0;
    uint32_t tick, change;
    road_state_t road_state = road_unknown;
    uint8_t updated;

    if (!test_started) {
        // 独立测试使用零安装变换，不把本次桌面参数作为正式安装标定结果
        tf_transform_set(&tf_base, 0.0f, 0.0f, 0.0f);
        map_update_init();
        test_started = 1;
        printf("===== LIDAR MAP DESK TEST START =====\r\n");
        printf("固定测试位姿=(1200,2125,180)，临时零安装偏移；不是实际全局定位\r\n");
        printf("观察原始0度方向前方400~600mm、左右各200mm；先移走小柱，再放到约500mm处\r\n");
        printf("只收观察框内的小柱候选，跨度上限=%.0fmm（试验值）；不再使用大挡板封路\r\n", LIDAR_PILLAR_MAX_SIZE);
        printf("按候选实际点判路，连续两圈确认后封锁，移走不自动解封\r\n");
        // 允许搜索未知路只用于观察候选绕行结果 进入未知路的决策仍为SCAN
        map_plan_find(1, 4, map_unknown_allow, &path);
        map_lidar_test_print_path(&path);
    }

    // 此任务内部持续处理DMA，按新扫描生成点云和聚类；底盘非空闲时不会更新
    updated = obstacle_find_task(MAP_LIDAR_TEST_X, MAP_LIDAR_TEST_Y, MAP_LIDAR_TEST_YAW);
    uint32_t drop = lidar_get_scan_drop_cnt();
    if (drop != last_drop) {
        last_drop = drop;
        map_update_clear_candidates();
        stat_scans = stat_hits = stat_max_points = 0;
        stat_min_points = UINT16_MAX;
        stat_max_size = 0.0f;
        printf("[LIDAR MAP] SCAN INTERRUPTED, reset road confirmation\r\n");
        return;  // 受影响的这次更新不写地图 已确认的封锁道路保留
    }
    if (!updated) return;

    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (!obstacle_cluster_is_pillar(cluster) ||
            cluster->min_x < MAP_LIDAR_TEST_MIN_X || cluster->max_x > MAP_LIDAR_TEST_MAX_X ||
            cluster->min_y < MAP_LIDAR_TEST_MIN_Y || cluster->max_y > MAP_LIDAR_TEST_MAX_Y) continue;

        selected[count++] = cluster;
    }

    tick = obstacle_result.update_tick;
    // 和正式入口使用同一套实际点判路；没有小柱候选也提交，让未确认道路的连续次数清零
    if (!map_lidar_handle_area(1, &test_area)) return;

    // hit表示选区里出现过候选的扫描数，不是已确认物体数；选区内应只放待测小柱
    // 截断圈不作为测量样本，重新统计100圈；已确认道路仍按地图策略保留
    if (obstacle_result.truncated) {
        stat_scans = stat_hits = stat_max_points = 0;
        stat_min_points = UINT16_MAX;
        stat_max_size = 0.0f;
    } else {
        stat_scans++;
        if (count != 0) stat_hits++;
        for (uint8_t i = 0; i < count; i++) {
            if (selected[i]->point_count < stat_min_points) stat_min_points = selected[i]->point_count;
            if (selected[i]->point_count > stat_max_points) stat_max_points = selected[i]->point_count;
            stat_max_size = fmaxf(stat_max_size, fmaxf(selected[i]->width, selected[i]->height));
        }
        if (stat_scans == 100) {
            printf("[PILLAR 100] hit=%u/100 points=%u~%u span_max=%.1fmm\r\n",
                   (unsigned int)stat_hits, (unsigned int)(stat_hits ? stat_min_points : 0),
                   (unsigned int)stat_max_points, stat_max_size);
            stat_scans = stat_hits = stat_max_points = 0;
            stat_min_points = UINT16_MAX;
            stat_max_size = 0.0f;
        }
    }
    change = map_update_change_get();
    if (change != last_change) {
        last_change = change;
        // 只有道路状态发生变化才重查路径，不在每圈扫描里反复打印同一条路径
        if (!map_plan_check(&path, map_unknown_allow)) {
            printf("[LIDAR MAP] OLD PATH BLOCKED -> REPLAN\r\n");
            map_plan_find(1, 4, map_unknown_allow, &path);
            map_lidar_test_print_path(&path);
        }
    }

    // 每500ms打印一次摘要 selected=选区内小柱候选数；不是所有普通聚类数量
    // truncated=1表示聚类被截断；age使用采集结束时间，不能拿处理时间冒充新鲜扫描
    // R8=UNKNOWN表示尚未确认封锁，不能解释为可通行；changes=累计道路状态变化次数
    if (tick - last_print_tick < 500U) return;
    last_print_tick = tick;
    map_road_get(8, &road_state);
    printf("[LIDAR MAP] scan=%lu clusters=%u selected=%u truncated=%u age=%lums R8=%s changes=%lu\r\n",
           (unsigned long)cloud.scan_num, (unsigned int)obstacle_result.cluster_count,
           (unsigned int)count, (unsigned int)obstacle_result.truncated,
           (unsigned long)(tick - cloud.scan_tick),
           road_state == road_blocked ? "BLOCKED" : road_state == road_free ? "FREE" : "UNKNOWN",
           (unsigned long)change);
    if (count == 0) map_lidar_test_diagnose(tick);
    for (uint8_t i = 0; i < count; i++) {
        printf("[LIDAR MAP] PILLAR%u points=%u center=(%.1f,%.1f) size=(%.1f,%.1f) near=%.0fmm\r\n",
               (unsigned int)i, (unsigned int)selected[i]->point_count,
               selected[i]->center_point.x, selected[i]->center_point.y,
               selected[i]->width, selected[i]->height, selected[i]->nearest_distance);
    }
}
