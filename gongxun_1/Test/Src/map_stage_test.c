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
    // width must be configured：宽度为0时必须拒绝初始化
    // 300mm车体外接正方形边长、50mm安全余量仅用于本测试，不是实车标定值
    // 有效的map_update_init会清空地图、扫描记录和道路变化计数
    map_test_check(!map_update_init(0.0f, 50.0f) &&
                   !map_update_is_ready() &&
                   map_update_init(300.0f, 50.0f),
                   "width must be configured");
    // unknown road policy：N1到N4的直达道路是R8，此时R8还是未知
    // allow允许把未知路纳入候选路径，但进路决策仍须扫描；stop禁止搜索未知路
    map_test_check(map_plan_find(1, 4, map_unknown_allow, &old_path) &&
                   old_path.road_count == 1 && old_path.roads[0] == 8 &&
                   map_plan_road_state(&old_path, 0) == map_step_scan &&
                   !map_plan_find(1, 4, map_unknown_stop, &new_path),
                   "unknown road policy");
    // start equals goal：起点终点都是N1时，只需要1个节点，不需要经过任何道路
    map_test_check(map_plan_find(1, 1, map_unknown_stop, &new_path) &&
                   new_path.node_count == 1 && new_path.road_count == 0 &&
                   new_path.nodes[0] == 1,
                   "start equals goal");
    map_road_set(8, road_free);  // 手动模拟已经确认R8可通行 不是根据无障碍自动判断
    // confirmed free can go：R8标为可通行后，严格策略也能找到直达路径并允许进入
    map_test_check(map_plan_find(1, 4, map_unknown_stop, &new_path) &&
                   new_path.road_count == 1 &&
                   map_plan_road_state(&new_path, 0) == map_step_go,
                   "confirmed free can go");
    map_road_set(8, road_unknown);  // 恢复未知状态 给后续跨圈确认测试使用

    ///////////////////////////////////// 5 跨圈确认并封路 ////////////////////////////////
    // map_update_scan参数依次为：障碍框数组、框数量、扫描编号、时间戳ms
    // 下面的1、2、3、4是模拟扫描编号；100、200等是模拟时间，不会等待真实时间
    // first scan and duplicate scan：第1圈只给R8累计一次；重复提交第1圈必须拒绝
    map_test_check(map_update_scan(&box, 1, 1, 100) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   !map_update_scan(&box, 1, 1, 101),
                   "first scan and duplicate scan");
    // miss resets road confirmation：第2圈没有障碍，R8的连续次数清零
    // 第3圈重新看到只算第一次，不能把不连续的两圈凑成确认
    // 参数(0,0,2,200)表示第2圈有效但没有障碍框，不表示雷达断流
    map_test_check(map_update_scan(0, 0, 2, 200) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 300) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_change_get() == 0,
                   "miss resets road confirmation");
    // consecutive scans block R8：第3、4圈连续看到才封路，只记录一次道路变化
    map_test_check(map_update_scan(&box, 1, 4, 400) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "consecutive scans block R8");
    // other roads and blocked persistence：R0保持未知；第5圈没看到障碍也不自动解封R8
    map_test_check(map_road_get(0, &state) && state == road_unknown &&
                   map_update_scan(0, 0, 5, 500) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "other roads and blocked persistence");
    // blocked road causes replan：旧直达路径失效，进入R8的决策必须是停止
    // 允许未知路参与搜索时应能绕行；新路径经过多条道路且检查有效
    // 找到候选绕行路径不代表未知道路已获准通行 进入前仍需扫描确认
    map_test_check(!map_plan_check(&old_path, map_unknown_allow) &&
                   map_plan_road_state(&old_path, 0) == map_step_stop &&
                   map_plan_find(1, 4, map_unknown_allow, &new_path) &&
                   new_path.road_count > 1 &&
                   map_plan_check(&new_path, map_unknown_allow),
                   "blocked road causes replan");

    ///////////////////////////////////// 6 同圈去重与漏圈 ////////////////////////////////
    // 同一圈里有几个框都只给道路累计一次，不要求这些框彼此接近
    map_update_init(300.0f, 50.0f);  // 重置地图 这一组测试从空地图开始
    map_scan_obs_t same_scan[2] = {box, box};
    same_scan[1].center_y += 150.0f; // 第二个框也在R8上，与第一个框相隔150mm
    same_scan[1].min_y += 150.0f;
    same_scan[1].max_y += 150.0f;
    map_update_scan(same_scan, 2, 1, 100);
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
    map_test_check(map_update_scan(&shifted_box, 1, 2, 200) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "road confirmation needs no object matching");

    // skipped scan resets confirmation：第1圈直接跳到第3圈，也必须重新累计
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_test_check(map_update_scan(&box, 1, 3, 300) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 4, 400) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "skipped scan resets confirmation");

    // long pause resets confirmation：编号相邻但间隔超过1秒，也不能沿用上次次数
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_test_check(map_update_scan(&box, 1, 2, 1200) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 1300) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "long pause resets confirmation");

    // clear restarts confirmation and keeps blocked：接收/位姿中断时清零；已经封路则保留
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_clear_candidates();
    map_test_check(map_update_scan(&box, 1, 2, 200) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 300) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "clear restarts road confirmation");
    map_update_clear_candidates();
    map_test_check(map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "clear keeps blocked roads");

    ///////////////////////////////////// 7 节点堵塞与无路可走 ////////////////////////////
    // 把模拟障碍移到中心节点N4 车体安全膨胀后会影响它连接的四个出口
    box.center_x = 1200.0f;
    box.center_y = 1200.0f;
    box.min_x = 1190.0f;
    box.max_x = 1210.0f;
    box.min_y = 1190.0f;
    box.max_y = 1210.0f;
    map_update_init(300.0f, 50.0f);  // 清除上一组的障碍和道路状态
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
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

    ///////////////////////////////////// 8 车体旋转后的安全膨胀 //////////////////////////
    // 300mm正方形从中心到角点约212mm 旋转时不能只按半宽150mm考虑碰撞
    // R0中心线Y=2125，测试障碍Y=1915，两者相距210mm
    // 即使额外安全余量设为0，也应因车体外接半径覆盖到障碍而封锁R0
    box.center_x = box.min_x = box.max_x = 775.0f;
    box.center_y = box.min_y = box.max_y = 1915.0f;
    map_update_init(300.0f, 0.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    map_test_check(map_road_get(0, &state) && state == road_blocked,
                   "rotated square safety radius");

    ///////////////////////////////////// 9 边缘障碍与显式放行 ////////////////////////////
    // 中心虽然在禁区，但障碍框加上车体范围后会影响R8，不能直接过滤
    box = (map_scan_obs_t){990.0f, 1625.0f, 980.0f, 1615.0f, 1000.0f, 1635.0f};
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    map_test_check(map_road_get(8, &state) && state == road_blocked,
                   "forbidden center still affects road");

    // 此点距R0中心线274mm，大于300mm正方形半对角线加50mm余量的262mm，不应封路
    box = (map_scan_obs_t){775.0f, 1851.0f, 775.0f, 1851.0f, 775.0f, 1851.0f};
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    map_test_check(map_road_get(0, &state) && state == road_unknown,
                   "square envelope does not overexpand");

    // 空扫描仍不能自动放行；这里用预设数据模拟上层已经确认R8整段观察有效
    // confirm_free参数：待进入道路编号、本次扫描编号、当前时间ms
    map_update_init(300.0f, 50.0f);
    map_test_check(!map_update_confirm_free(8, 1, 100) &&
                   map_update_scan(0, 0, 1, 100) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   !map_update_confirm_free(8, 0, 101) &&
                   !map_update_confirm_free(8, 1, 1101),
                   "free needs latest fresh observation");
    map_test_check(map_update_confirm_free(8, 1, 101) &&
                   map_update_confirm_free(8, 1, 102) &&
                   map_update_change_get() == 1 &&
                   map_plan_find(1, 4, map_unknown_stop, &new_path) &&
                   map_plan_road_state(&new_path, 0) == map_step_go,
                   "explicit free allows GO once");
    map_update_clear_candidates();
    map_test_check(!map_update_confirm_free(0, 1, 103),
                   "interruption rejects old free observation");

    // 已放行道路后来连续两圈受影响，仍应封锁；移走障碍也不能用放行接口直接解封
    box = (map_scan_obs_t){1200.0f, 1625.0f, 1190.0f, 1615.0f, 1210.0f, 1635.0f};
    map_update_scan(&box, 1, 2, 200);
    map_test_check(!map_update_confirm_free(8, 2, 201),
                   "one hit rejects free confirmation");
    map_update_scan(&box, 1, 3, 300);
    map_update_scan(0, 0, 4, 400);
    map_test_check(!map_update_confirm_free(8, 4, 401) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 2,
                   "explicit free cannot reopen blocked road");

    ///////////////////////////////////// 10 输出最终结果 /////////////////////////////////
    // ALL PASS：全部检查通过；FAILED：至少一项失败，向上查对应的FAIL名称
    // 测试结束后保留最后一组测试地图 正式接雷达前应重新按实际参数初始化地图
    printf("[MAP TEST] %s\r\n", map_test_fail ? "FAILED" : "ALL PASS");
    return map_test_fail == 0;  // 返回1表示全部通过，返回0表示有失败项
}


///////////////////////////////////// 真实雷达桌面联调 ////////////////////////////////////
// 本测试使用真实扫描和聚类，但位姿、车体尺寸都是人为设置的测试值
// 临时把雷达原点当作测试车体原点，0度射线当作车头方向，不使用实际安装标定参数
// 测试位姿设在N1=(1200,2125)，转180度后，雷达0度方向指向地图中心N4
// 这样雷达前方500mm的物体会落在地图(1200,1625)，即刚才自检用的R8中间
// 这不是编码器定位结果 正式地图仍须通过map_obstacle_task接入可信位姿
#define MAP_LIDAR_TEST_X         1200.0f
#define MAP_LIDAR_TEST_Y         2125.0f
#define MAP_LIDAR_TEST_YAW         180.0f
#define MAP_LIDAR_TEST_WIDTH       300.0f  // 仅用于桌面软件联调，不是实测车宽
#define MAP_LIDAR_TEST_SAFETY       50.0f  // 测试安全余量 mm

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
    map_step_t step = map_plan_road_state(path, 0);
    printf(" entry=%s\r\n", step == map_step_go ? "GO" :
                            step == map_step_scan ? "SCAN" : "STOP");
}

// 主循环持续调用 本函数不发送底盘运动指令，不把未知道路直接标为可通行
// 上电先清空测试地图，再连续执行：真实点云与聚类 -> 观察框筛选 -> 跨圈确认 -> 封路重规划
// 已确认的封锁不会因移走挡板自动解除 要重新做一轮完整实验，请复位开发板
void map_lidar_test(void) {
    static uint8_t test_started = 0;
    static uint32_t last_print_tick = 0;
    static uint32_t last_change = 0;
    static uint32_t last_drop = 0;  // 接收错误打断扫描后，清零道路连续确认次数
    static map_path_t path;
    map_scan_obs_t boxes[CLUSTER_MAX_COUNT];
    uint8_t count = 0;
    uint32_t tick, change;
    road_state_t road_state = road_unknown;
    uint8_t updated;

    if (!test_started) {
        // 独立测试使用零安装变换，不把本次桌面参数作为正式安装标定结果
        tf_transform_set(&tf_base, 0.0f, 0.0f, 0.0f);
        if (!map_update_init(MAP_LIDAR_TEST_WIDTH, MAP_LIDAR_TEST_SAFETY)) return;
        test_started = 1;
        printf("===== LIDAR MAP DESK TEST START =====\r\n");
        printf("固定测试位姿=(1200,2125,180)，临时零安装偏移；不是实际全局定位\r\n");
        printf("观察原始0度方向前方400~600mm、左右各200mm；先移走挡板，再放到约500mm处\r\n");
        printf("只收观察框内的完整聚类；道路连续两圈有障碍才封锁，移走不自动解封\r\n");
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
        printf("[LIDAR MAP] SCAN INTERRUPTED, reset road confirmation\r\n");
        return;  // 受影响的这次更新不写地图 已确认的封锁道路保留
    }
    if (!updated) return;

    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (!cluster->valid ||
            cluster->min_x < MAP_LIDAR_TEST_MIN_X || cluster->max_x > MAP_LIDAR_TEST_MAX_X ||
            cluster->min_y < MAP_LIDAR_TEST_MIN_Y || cluster->max_y > MAP_LIDAR_TEST_MAX_Y) continue;

        // 只把选区内的真实聚类转成地图输入框 不注入预设障碍
        boxes[count].center_x = cluster->center_point.x;
        boxes[count].center_y = cluster->center_point.y;
        boxes[count].min_x = cluster->min_x;
        boxes[count].min_y = cluster->min_y;
        boxes[count].max_x = cluster->max_x;
        boxes[count].max_y = cluster->max_y;
        count++;
    }

    tick = obstacle_result.update_tick;
    // count=0表示本圈观察框内没有完整聚类 仍提交，让道路连续确认次数清零
    if (!map_update_scan(boxes, count, cloud.scan_num, tick)) return;
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

    // 每500ms打印一次摘要 selected=本圈入选框数；本版不再统计单个障碍物的候选/确认数量
    // R8=UNKNOWN表示尚未确认封锁，不能解释为可通行；changes=累计道路状态变化次数
    if (tick - last_print_tick < 500U) return;
    last_print_tick = tick;
    map_road_get(8, &road_state);
    printf("[LIDAR MAP] scan=%lu clusters=%u selected=%u R8=%s changes=%lu\r\n",
           (unsigned long)cloud.scan_num, (unsigned int)obstacle_result.cluster_count,
           (unsigned int)count,
           road_state == road_blocked ? "BLOCKED" : road_state == road_free ? "FREE" : "UNKNOWN",
           (unsigned long)change);
    for (uint8_t i = 0; i < count; i++) {
        printf("[LIDAR MAP] BOX%u center=(%.1f,%.1f) size=(%.1f,%.1f)mm\r\n",
               (unsigned int)i, boxes[i].center_x, boxes[i].center_y,
               boxes[i].max_x - boxes[i].min_x, boxes[i].max_y - boxes[i].min_y);
    }
}
