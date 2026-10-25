#include "headfile.h"

// 导航任务变量
static map_nav_t map_nav;                   // 当前任务、路径和运行状态
static map_unknown_mode_t nav_unknown_mode; // 本次规划是否允许未知道路
static int16_t nav_speed;                   // 本次任务的电机指令速度 RPM
static uint32_t nav_phase_tick;             // 进入当前状态的时刻 用于超时检查
static float nav_map_yaw;                   // 当前道路的地图方向
static float nav_imu_yaw;                   // 本次转向的累计IMU目标角
static float nav_yaw_offset;                // 启动时的角度偏移  任务期间不允许重新对齐
static uint8_t nav_align_stopped;           // 对准后是否已发送停车指令 避免重复发送

// 用连续的新位姿检查停稳  不能重复使用同一份定位结果累计次数
static map_pose_t nav_settle_pose;          // 本轮停稳观察的基准位姿
static uint32_t nav_pose_tick;              // 上一次参与停稳判断的定位时间
static uint8_t nav_settle_count;            // 连续有效定位次数 达到3次后不再增加

///////////////////////////////////// 内部调用函数 ////////////////////////////////////////
// 角度归一化到[-180,180)   用于求最短转角和比较朝向变化
static float nav_angle_wrap(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle >= 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 规划到到位确认都属于任务执行中 完成和停车状态不再主动推进
static uint8_t nav_is_active(void)
{
    return map_nav.state >= map_nav_plan && map_nav.state <= map_nav_arrive;
}

// 状态切换时重新计算本阶段超时和停稳观察
static void nav_enter(map_nav_state_t state, uint32_t tick)
{
    map_nav.state = state;
    nav_phase_tick = tick;
    nav_settle_count = 0;
}

// 结束本模块的任务并记录原因 保留路径和最后确认节点供上层查询
// 未执行导航时只记录错误 不停车干预其他任务
static void nav_stop(map_nav_error_t error)
{
    if (nav_is_active()) car_stop();
    map_nav.error = error;
    map_nav.state = map_nav_stop;
    nav_settle_count = 0;
}

// 用实际位置判断是否进入节点容差范围 比较距离平方 避免开方
static uint8_t nav_at_node(const map_pose_t *pose, uint8_t node_id)
{
    map_node_t node;
    if (!map_node_get(node_id, &node))
        return 0;
    float dx = pose->x_mm - node.x;
    float dy = pose->y_mm - node.y;
    return dx * dx + dy * dy <= MAP_NAV_NODE_TOL_MM * MAP_NAV_NODE_TOL_MM;
}

// 底盘空闲且至少3次新定位在同一小范围内 才允许认为已停稳
static uint8_t nav_is_settled(const map_pose_t *pose)
{
    // 底盘仍在执行动作时 本轮停稳观察作废
    if (car_is_busy())
    {
        nav_settle_count = 0;
        return 0;
    }
    // 首次观察或定位更新间隔过长 重新以本次位姿为基准
    if (nav_settle_count == 0 ||
        (uint32_t)(pose->tick - nav_pose_tick) > MAP_POSE_TIMEOUT_MS)
    {
        nav_settle_pose = *pose;
        nav_pose_tick = pose->tick;
        nav_settle_count = 1;
        return 0;
    }
    // 主循环可能比定位更新快 同一份定位结果不能重复计数
    if (pose->tick == nav_pose_tick) return 0;
    nav_pose_tick = pose->tick;

    // 与本轮基准比较 位置或朝向变化过大时重新观察
    float dx = pose->x_mm - nav_settle_pose.x_mm;
    float dy = pose->y_mm - nav_settle_pose.y_mm;
    if (dx * dx + dy * dy > MAP_NAV_SETTLE_MM * MAP_NAV_SETTLE_MM ||
        fabsf(nav_angle_wrap(pose->yaw_deg - nav_settle_pose.yaw_deg)) > MAP_NAV_SETTLE_YAW_DEG)
    {
        nav_settle_pose = *pose;
        nav_settle_count = 1;
        return 0;
    }
    // 新定位次数和稳定持续时间都满足 才通过停稳检查
    if (nav_settle_count < 3) nav_settle_count++;
    return nav_settle_count >= 3 &&
           (uint32_t)(pose->tick - nav_settle_pose.tick) >= MAP_NAV_SETTLE_MS;
}

// 只检查尚未走完的道路  经过的道路后来封锁不影响当前剩余路径
static uint8_t nav_remaining_valid(void)
{
    for (uint8_t i = map_nav.road_index; i < map_nav.path.road_count; i++)
    {
        road_state_t state;
        if (!map_road_get(map_nav.path.roads[i], &state) || state == road_blocked ||
            (state == road_unknown && nav_unknown_mode == map_unknown_stop))
            return 0;
    }
    return 1;
}

// 从路径索引取出当前道路、下一节点及道路方向 不推进路径进度
static uint8_t nav_segment_get(void)
{
    map_node_t from, to;
    if (map_nav.road_index >= map_nav.path.road_count) return 0;
    map_nav.road_id = map_nav.path.roads[map_nav.road_index];
    map_nav.next_node = map_nav.path.nodes[map_nav.road_index + 1];
    if (!map_node_get(map_nav.current_node, &from) || !map_node_get(map_nav.next_node, &to))
        return 0;

    // 车头局部+Y向前  地图北0度、东-90度
    nav_map_yaw = atan2f(-(to.x - from.x), to.y - from.y) * 180.0f / TF_PI;
    return 1;
}

// 本版固定赛道道路均为水平或竖直  检查车辆是否仍在本段附近
static uint8_t nav_on_segment(const map_pose_t *pose)
{
    map_node_t from, to;
    if (!map_node_get(map_nav.current_node, &from) || !map_node_get(map_nav.next_node, &to))
        return 0;
    // 竖直道路检查横向偏差 水平道路检查纵向偏差
    // 沿行驶方向只允许超出两端节点少量容差
    if (from.x == to.x)
        return fabsf(pose->x_mm - from.x) <= MAP_NAV_PATH_TOL_MM &&
               pose->y_mm >= fminf(from.y, to.y) - MAP_NAV_NODE_TOL_MM &&
               pose->y_mm <= fmaxf(from.y, to.y) + MAP_NAV_NODE_TOL_MM;
    return fabsf(pose->y_mm - from.y) <= MAP_NAV_PATH_TOL_MM &&
           pose->x_mm >= fminf(from.x, to.x) - MAP_NAV_NODE_TOL_MM &&
           pose->x_mm <= fmaxf(from.x, to.x) + MAP_NAV_NODE_TOL_MM;
}

///////////////////////////////////// 导航接口 ////////////////////////////////////////
// 初始化导航状态 若本模块正在执行任务 先取消当前动作
void map_nav_init(void)
{
    if (nav_is_active()) car_stop();
    memset(&map_nav, 0, sizeof(map_nav));
    map_nav.current_node = MAP_NO_ID;
    map_nav.target_node = MAP_NO_ID;
    map_nav.next_node = MAP_NO_ID;
    map_nav.road_id = MAP_NO_ID;
    nav_settle_count = 0;
    map_nav_observe_sync();
}

// 接收一个节点到节点的任务 检查通过后进入规划状态 此处不下发移动
uint8_t map_nav_start(uint8_t start, uint8_t goal, map_unknown_mode_t mode, int16_t speed)
{
    map_pose_t pose;
    uint32_t tick = HAL_GetTick();
    // 执行中不接受覆盖任务 新任务参数也不能改变旧任务状态
    if (nav_is_active()) return 0;
    if (start >= MAP_NODE_COUNT || goal >= MAP_NODE_COUNT || speed <= 0 ||
        speed > MAP_NAV_MAX_SPEED || (mode != map_unknown_allow && mode != map_unknown_stop))
    {
        nav_stop(map_nav_error_param);
        return 0;
    }
    if (car_is_busy())
    {
        nav_stop(map_nav_error_busy);
        return 0;
    }
    // 必须有未过期的实际定位 且车体原点位于地图正常区域
    if (!map_pose_get(tick, &pose) || map_check_area(pose.x_mm, pose.y_mm) != map_normal)
    {
        nav_stop(map_nav_error_pose);
        return 0;
    }
    // 起点由上层指定 但必须与实际位置相符 不能把道路中间当成节点起步
    if (!nav_at_node(&pose, start))
    {
        nav_stop(map_nav_error_start);
        return 0;
    }
    if (!map_update_is_ready())
    {
        nav_stop(map_nav_error_road);
        return 0;
    }

    // 保存本次任务参数 路径由后续plan状态生成
    memset(&map_nav, 0, sizeof(map_nav));
    map_nav.current_node = start;
    map_nav.target_node = goal;
    map_nav.next_node = MAP_NO_ID;
    map_nav.road_id = MAP_NO_ID;
    nav_speed = speed;
    nav_unknown_mode = mode;
    nav_yaw_offset = map_pose_yaw_offset_get();
    nav_enter(map_nav_plan, tick);
    return 1;
}

// 周期推进状态  每次只处理一个阶段 不阻塞等待车辆动作
void map_nav_task(void)
{
    map_pose_t pose;
    uint32_t tick = HAL_GetTick();
    if (!nav_is_active()) return;
    // 执行期间定位失效、进入禁区或重新设置角度参考 都需要终止本次任务
    if (!map_pose_get(tick, &pose) || map_check_area(pose.x_mm, pose.y_mm) != map_normal ||
        fabsf(nav_angle_wrap(map_pose_yaw_offset_get() - nav_yaw_offset)) > 0.001f)
    {
        nav_stop(map_nav_error_pose);
        return;
    }
    // 单段移动单独限时 其他状态使用等待超时 每次状态切换都会重新计时
    uint32_t timeout = map_nav.state == map_nav_move ? MAP_NAV_MOVE_TIMEOUT_MS : MAP_NAV_WAIT_TIMEOUT_MS;
    if ((uint32_t)(tick - nav_phase_tick) > timeout)
    {
        nav_stop(map_nav_error_timeout);
        return;
    }

    switch (map_nav.state)
    {
        case map_nav_plan:      // 在已确认节点重新规划
            if (car_is_busy()) { nav_stop(map_nav_error_busy); break; }
            if (!nav_at_node(&pose, map_nav.current_node)) { nav_stop(map_nav_error_deviation); break; }
            if (!map_plan_find(map_nav.current_node, map_nav.target_node, nav_unknown_mode, &map_nav.path))
            {
                nav_stop(map_nav_error_no_path);
                break;
            }
            map_nav.road_index = 0;
            // 起终点相同时无需移动 但仍要经过实际到位和停稳确认
            if (map_nav.path.road_count == 0)
            {
                map_nav.next_node = map_nav.current_node;
                map_nav.road_id = MAP_NO_ID;
                nav_enter(map_nav_arrive, tick);
            }
            else if (nav_segment_get()) nav_enter(map_nav_scan, tick);
            else nav_stop(map_nav_error_road);
            break;

        case map_nav_scan:      // 等待停稳并检查下一条道路许可
        {
            if (car_is_busy()) { nav_stop(map_nav_error_busy); break; }
            if (!nav_at_node(&pose, map_nav.current_node)) { nav_stop(map_nav_error_deviation); break; }
            if (!nav_is_settled(&pose)) break;
            // 尚未走完的道路失效时 从当前已确认节点重新规划
            if (!nav_remaining_valid())
            {
                nav_enter(map_nav_plan, tick);
                break;
            }
            // 候选路径可搜索不等于允许进入 必须取得本次有效通行许可
            if (map_plan_road_state(&map_nav.path, map_nav.road_index, tick) != map_step_go) break;
            if (fabsf(nav_angle_wrap(nav_map_yaw - pose.yaw_deg)) > MAP_NAV_YAW_TOL_DEG)
            {
                if (!isfinite(my_yaw)) { nav_stop(map_nav_error_pose); break; }
                // 地图角转换为当前累计IMU角附近的目标 避免跨多圈转向
                nav_imu_yaw = my_yaw + nav_angle_wrap(nav_map_yaw - nav_yaw_offset - nav_angle_wrap(my_yaw));
                nav_align_stopped = 0;
                map_update_clear_candidates(); // 转向打断静止观察  结束后必须重新放行
                nav_enter(map_nav_align, tick);
                break;
            }

            map_node_t target;
            if (!map_node_get(map_nav.next_node, &target)) { nav_stop(map_nav_error_road); break; }
            float dx = target.x - pose.x_mm;
            float dy = target.y - pose.y_mm;
            float yaw_rad = nav_map_yaw * TF_PI / 180.0f;
            // 把目标相对位置投影到道路前进方向 得到还需要前进的距离
            float forward_mm = -dx * sinf(yaw_rad) + dy * cosf(yaw_rad);
            if (forward_mm <= 0.0f) { nav_stop(map_nav_error_deviation); break; }

            // 一次只下发当前道路的移动  地图mm换算为底盘cm
            car_run_distance(nav_speed, 0.0f, forward_mm / 10.0f);
            if (car_get_state() != CHASSIS_MOVE) { nav_stop(map_nav_error_busy); break; }
            map_update_clear_candidates(); // 下一节点重新静止观察  不沿用本次许可
            nav_enter(map_nav_move, tick);
            break;
        }

        case map_nav_align:     // 持续执行转向 对准后停车并重新观察道路
            if (car_get_state() == CHASSIS_MOVE) { nav_stop(map_nav_error_busy); break; }
            if (!nav_at_node(&pose, map_nav.current_node)) { nav_stop(map_nav_error_deviation); break; }
            if (!isfinite(my_yaw)) { nav_stop(map_nav_error_pose); break; }
            if (!nav_remaining_valid())
            {
                car_stop();
                nav_enter(map_nav_scan, tick);
                break;
            }
            if (fabsf(nav_angle_wrap(nav_map_yaw - pose.yaw_deg)) > MAP_NAV_YAW_TOL_DEG)
            {
                // 仍有角度误差时持续调用原有转向控制 不积累停稳次数
                nav_settle_count = 0;
                nav_align_stopped = 0;
                car_rotate_yaw(nav_imu_yaw);
            }
            else
            {
                // 角度进入容差后只发一次停车 再用新定位确认稳定
                if (!nav_align_stopped)
                {
                    car_stop();
                    nav_align_stopped = 1;
                }
                if (nav_is_settled(&pose)) nav_enter(map_nav_scan, tick);
            }
            break;

        case map_nav_move:      // 本段移动指令已经发送 只检查结果 不重复下发
        case map_nav_arrive:    // 底盘结束动作后 等待实际到位和停稳
        {
            if (map_nav.road_index < map_nav.path.road_count)
            {
                // 当前道路变为未知或封锁时停车 同时检查车辆是否偏离本段
                road_state_t state;
                if (!map_road_get(map_nav.road_id, &state) || state != road_free)
                {
                    nav_stop(map_nav_error_road);
                    break;
                }
                if (!nav_on_segment(&pose)) { nav_stop(map_nav_error_deviation); break; }
            }
            if (car_get_state() == CHASSIS_ROTATE) { nav_stop(map_nav_error_busy); break; }
            if (map_nav.state == map_nav_move)
            {
                // 底盘忙状态结束只能进入到位确认 不能直接更新当前节点
                if (!car_is_busy()) nav_enter(map_nav_arrive, tick);
                break;
            }
            // 本段已结束却再次出现移动 表示有其他控制源下发了指令
            if (car_is_busy()) { nav_stop(map_nav_error_busy); break; }
            if (!nav_at_node(&pose, map_nav.next_node))
            {
                nav_settle_count = 0;
                break;
            }
            if (!nav_is_settled(&pose)) break;

            // 真实到位且停稳后 才推进当前节点和路径进度
            map_nav.current_node = map_nav.next_node;
            if (map_nav.road_index < map_nav.path.road_count) map_nav.road_index++;
            if (map_nav.current_node == map_nav.target_node)
                nav_enter(map_nav_finish, tick);
            else if (nav_segment_get()) nav_enter(map_nav_scan, tick);
            else nav_stop(map_nav_error_road);
            break;
        }

        default:
            nav_stop(map_nav_error_param);
            break;
    }
}

// 主动取消任务 已完成或已停车时不重复发送停车指令
void map_nav_cancel(void)
{
    if (nav_is_active()) nav_stop(map_nav_error_cancel);
}

// 复制当前导航结果 上层读取状态、路径进度和停车原因
uint8_t map_nav_get_result(map_nav_t *out)
{
    if (out == NULL) return 0;
    *out = map_nav;
    return 1;
}

///////////////////////////////////// 停车观察与小柱确认 ////////////////////////////////////////

// 只保存候选摘要和5位命中历史，不缓存多圈点云、不分配堆内存。
static map_nav_observe_result_t observation;
static uint32_t observe_last_scan, observe_last_tick, observe_start_tick;

static float observe_wrap(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle >= 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float observe_positive(float angle) {
    angle = fmodf(angle, 360.0f);
    return angle < 0.0f ? angle + 360.0f : angle;
}

static uint8_t observe_hits(uint8_t mask) {
    uint8_t count = 0;
    for (; mask; mask >>= 1) count += mask & 1U;
    return count;
}

// 清窗口时保留当前任务，扫描屏障更新为已发布的cloud，不能重算旧圈。
void map_nav_observe_reset(void) {
    memset(observation.candidates, 0, sizeof(observation.candidates));
    observation.scan_num = observation.scan_tick = observation.frames = 0;
    observation.process_us = 0;
    observation.first_confirm_ms = UINT32_MAX;
    observation.pending_roads = observation.confirmed_roads = 0;
    observation.frame_candidates = observation.match_rejects = 0;
    observation.candidate_count = observation.confirmed_count = observation.overflow = 0;
    observe_last_scan = cloud.scan_num;
    observe_last_tick = observe_start_tick = 0;
    map_update_clear_candidates();
}

void map_nav_observe_end(void) {
    memset(&observation, 0, sizeof(observation));
    observation.current_node = observation.next_node = observation.road_id = MAP_NO_ID;
    map_nav_observe_reset();
}

uint8_t map_nav_observe_begin(uint8_t current_node, uint8_t next_node) {
    uint8_t road;
    if (!map_road_find(current_node, next_node, &road)) {
        map_nav_observe_end();
        return 0;
    }
    if (observation.active && observation.current_node == current_node && observation.next_node == next_node)
        return 1;
    observation.active = 1;
    observation.current_node = current_node;
    observation.next_node = next_node;
    observation.road_id = road;
    map_nav_observe_reset();
    return 1;
}

uint8_t map_nav_observe_is_active(void) { return observation.active; }
uint16_t map_nav_observe_task_key(void) {
    return observation.active ? (uint16_t)((observation.current_node << 8) | observation.next_node) : UINT16_MAX;
}

uint8_t map_nav_observe_check_timeout(uint32_t tick) {
    if (!observation.active || !observation.frames || tick - observation.scan_tick <= MAP_SCAN_TIMEOUT_MS)
        return 0;
    map_nav_observe_reset();
    return 1;
}

uint8_t map_nav_observe_get_result(map_nav_observe_result_t *out) {
    if (out == NULL) return 0;
    *out = observation;
    return 1;
}

// 导航提供实际雷达位置；道路选区和角区几何交给map_plan。
static uint8_t observe_sector(const map_rect_t *area, float *center, float *half_width) {
    float rad = cloud.car_angle * TF_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);
    float sensor_x = cloud.car_x + cs * tf_base.tx - sn * tf_base.ty;
    float sensor_y = cloud.car_y + sn * tf_base.tx + cs * tf_base.ty;
    if (!map_plan_observe_sector(area, sensor_x, sensor_y,
                                cloud.car_angle + tf_base.yaw_deg,
                                MAP_OBSERVE_ANGLE_MARGIN_DEG, center, half_width)) return 0;
    observation.angle_start_deg = observe_positive(*center - *half_width);
    observation.angle_end_deg = observe_positive(*center + *half_width);
    return 1;
}

// 完整聚类先经过尺寸筛选，再检查它真实点的角度和道路归属。
static uint16_t observe_cluster_roads(const lidar_cluster_t *cluster, const map_rect_t *area,
                                      float center, float half_width) {
    uint16_t roads = 0;
    uint16_t span = (cluster->end_angle + 360U - cluster->start_angle) % 360U;
    for (uint16_t i = 0; i <= span; i++) {
        uint16_t angle = (cluster->start_angle + i) % 360U;
        const lidar_point_t *point = &cloud.points[angle];
        float x = point->map_point.x, y = point->map_point.y;
        if (!point->valid || !isfinite(x) || !isfinite(y) ||
            x < area->min_x || x >= area->max_x || y < area->min_y || y >= area->max_y ||
            fabsf(observe_wrap((float)angle - center)) > half_width) continue;
        uint16_t actual = map_lidar_point_roads(x, y);
        if (actual & (1U << observation.road_id)) roads |= actual;
    }
    return roads;
}

// 最近位置一对一匹配，固定anchor防止链式漂移；同一圈同一候选最多一次。
static void observe_add(const lidar_cluster_t *cluster, uint16_t roads) {
    map_nav_observe_candidate_t *match = NULL, *empty = NULL;
    float best = MAP_OBSERVE_MATCH_MM * MAP_OBSERVE_MATCH_MM;
    uint8_t already_matched = 0, had_candidate = 0;
    for (uint8_t i = 0; i < MAP_OBSERVE_MAX_CANDIDATES; i++) {
        map_nav_observe_candidate_t *candidate = &observation.candidates[i];
        if (!candidate->valid) { if (empty == NULL) empty = candidate; continue; }
        had_candidate = 1;
        float dx = cluster->center_point.x - candidate->anchor.x;
        float dy = cluster->center_point.y - candidate->anchor.y;
        float square = dx * dx + dy * dy;
        if (square <= MAP_OBSERVE_MATCH_MM * MAP_OBSERVE_MATCH_MM && candidate->current_hit)
            already_matched = 1;
        if (!candidate->current_hit && square <= best) { best = square; match = candidate; }
    }
    if (match == NULL) {
        if (already_matched) return;
        if (had_candidate) observation.match_rejects++;
        if (empty == NULL) { observation.overflow = 1; return; }
        match = empty;
        memset(match, 0, sizeof(*match));
        match->anchor = cluster->center_point;
        match->valid = 1;
    }
    match->hit_window |= 1U;
    match->road_history[0] = roads;
    match->current_hit = 1;
    match->position = cluster->center_point;
    match->last_hit_scan = cloud.scan_num;
    match->point_count = cluster->point_count;
    match->nearest_distance = cluster->nearest_distance;
    match->width = cluster->width;
    match->height = cluster->height;
}

uint8_t map_nav_observe_update(uint8_t pose_valid) {
    map_rect_t area = {0};
    float center, half_width;
    uint8_t cell_x, cell_y;
    map_cell_t cell;
    if (!observation.active) return 0;
    if (!pose_valid || !map_update_is_ready() || cloud.scan_num == 0 ||
        !isfinite(cloud.car_x) || !isfinite(cloud.car_y) || !isfinite(cloud.car_angle) ||
        !isfinite(tf_base.tx) || !isfinite(tf_base.ty) || !isfinite(tf_base.yaw_deg) ||
        !map_xy_to_cell(cloud.car_x, cloud.car_y, &cell_x, &cell_y) ||
        !map_cell_get(cell_x, cell_y, &cell) || cell.cell_type != cell_node ||
        cell.cell_id != observation.current_node ||
        obstacle_result.cloud_flag == 0 || obstacle_result.cloud_flag != cloud.update_flag ||
        obstacle_result.cluster_count > CLUSTER_MAX_COUNT ||
        HAL_GetTick() - cloud.scan_tick > MAP_SCAN_TIMEOUT_MS) {
        map_nav_observe_reset();
        return 0;
    }
    if (observe_last_scan && (int32_t)(cloud.scan_num - observe_last_scan) <= 0) return 0;
#if defined(DWT) && defined(CoreDebug)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before_cycles = DWT->CYCCNT;
#endif
    // 编号跳变和长间隔无法作为连续窗口，当前真实新圈可以开启新窗口。
    if (observation.frames && (cloud.scan_num - observe_last_scan != 1U ||
        cloud.scan_tick - observe_last_tick > MAP_SCAN_TIMEOUT_MS)) map_nav_observe_reset();
    if (!map_plan_observe_area(observation.road_id, observation.next_node, &area) || !observe_sector(&area, &center, &half_width)) {
        map_nav_observe_reset(); return 0;
    }
    if (!observation.frames) observe_start_tick = cloud.scan_tick;
    observation.frame_candidates = observation.match_rejects = 0;
    observation.pending_roads = observation.confirmed_roads = 0;
    observation.candidate_count = observation.confirmed_count = 0;
    observation.overflow = obstacle_result.truncated;
    for (uint8_t i = 0; i < MAP_OBSERVE_MAX_CANDIDATES; i++) {
        map_nav_observe_candidate_t *candidate = &observation.candidates[i];
        candidate->hit_window = (candidate->hit_window << 1) & ((1U << MAP_OBSERVE_WINDOW) - 1U);
        candidate->current_hit = 0;
        for (uint8_t j = MAP_OBSERVE_WINDOW - 1; j > 0; j--)
            candidate->road_history[j] = candidate->road_history[j - 1];
        candidate->road_history[0] = 0;
        if (candidate->hit_window == 0) memset(candidate, 0, sizeof(*candidate));
    }
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (!obstacle_cluster_is_pillar(cluster)) continue;
        if (cluster->start_angle >= 360 || cluster->end_angle >= 360 ||
            !isfinite(cluster->center_point.x) || !isfinite(cluster->center_point.y)) {
            observation.overflow = 1; continue;
        }
        uint16_t roads = observe_cluster_roads(cluster, &area, center, half_width);
        if (!roads) continue;
        observation.frame_candidates++;
        observe_add(cluster, roads);
    }
    for (uint8_t i = 0; i < MAP_OBSERVE_MAX_CANDIDATES; i++) {
        map_nav_observe_candidate_t *candidate = &observation.candidates[i];
        if (!candidate->valid) continue;
        uint16_t roads = 0;
        candidate->hits = observe_hits(candidate->hit_window);
        for (uint8_t j = 0; j < MAP_OBSERVE_WINDOW; j++) roads |= candidate->road_history[j];
        observation.candidate_count++;
        observation.pending_roads |= roads;
        if (candidate->hits >= MAP_OBSERVE_MIN_HITS) {
            observation.confirmed_count++;
            // 只在本圈有新回波时提交确认，历史状态不冒充本圈测量。
            if (candidate->current_hit) observation.confirmed_roads |= roads;
        }
    }
    // 方向选区仅用于确认障碍，无法证明整条道路无障碍，始终不授权FREE。
    if (!map_update_confirmed_roads(observation.confirmed_roads, observation.pending_roads,
                                    cloud.scan_num, cloud.scan_tick, 0)) {
        map_nav_observe_reset(); return 0;
    }
    observation.scan_num = observe_last_scan = cloud.scan_num;
    observation.scan_tick = observe_last_tick = cloud.scan_tick;
    observation.frames++;
    if (observation.confirmed_roads && observation.first_confirm_ms == UINT32_MAX)
        observation.first_confirm_ms = cloud.scan_tick - observe_start_tick;
#if defined(DWT) && defined(CoreDebug)
    observation.process_us = (DWT->CYCCNT - before_cycles) / (SystemCoreClock / 1000000U);
#endif
    return 1;
}

// 导航只在scan阶段拥有观察方向；其他任务仍可手动begin，不受空闲导航覆盖。
void map_nav_observe_sync(void) {
    static uint8_t owned_by_navigation = 0;
    if (map_nav.state == map_nav_scan) {
        map_nav_observe_begin(map_nav.current_node, map_nav.next_node);
        owned_by_navigation = 1;
    }
    else if (owned_by_navigation) {
        map_nav_observe_end();
        owned_by_navigation = 0;
    }
}
