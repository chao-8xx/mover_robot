#include "map_nav.h"
#include "headfile.h"
#include <stdio.h>

float my_yaw;
static uint32_t tick;
static uint32_t scan_num;
static Chassis_Mode mode;
static unsigned move_count, rotate_count, stop_count, failures, checks;
static float last_distance, last_target, yaw_offset;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t car_is_busy(void) { return mode != CHASSIS_IDLE; }
Chassis_Mode car_get_state(void) { return mode; }
void car_stop(void) { mode = CHASSIS_IDLE; stop_count++; }
void car_run_distance(int16_t speed, float x_cm, float y_cm)
{
    (void)speed;
    (void)x_cm;
    move_count++;
    last_distance = y_cm;
    mode = CHASSIS_MOVE;
}
void car_rotate_yaw(float target_yaw)
{
    rotate_count++;
    last_target = target_yaw;
    mode = fabsf(target_yaw - my_yaw) > YAW_THRESH ? CHASSIS_ROTATE : CHASSIS_IDLE;
}

static void test_check(uint8_t ok, const char *name)
{
    checks++;
    if (!ok)
    {
        failures++;
        printf("[NAV TEST] FAIL %s\n", name);
    }
}

static map_nav_t result(void)
{
    map_nav_t out;
    memset(&out, 0, sizeof(out));
    map_nav_get_result(&out);
    return out;
}

static void publish(float x, float y, float map_yaw)
{
    my_yaw = map_yaw - yaw_offset;
    map_pose_update(x, y, my_yaw, tick, 1);
}

static void step(uint32_t dt, float x, float y, float map_yaw)
{
    tick += dt;
    publish(x, y, map_yaw);
    map_nav_task();
}

static void setup(float x, float y, float map_yaw)
{
    // 清理上一用例  再单独统计本轮底盘调用
    map_nav_init();
    map_pose_init();
    map_update_init();
    tick = 100;
    scan_num = 0;
    yaw_offset = MAP_POSE_DEFAULT_YAW_OFFSET;
    mode = CHASSIS_IDLE;
    move_count = rotate_count = stop_count = 0;
    last_distance = last_target = 0.0f;
    publish(x, y, map_yaw);
}

// 模拟上层已确认观察范围  导航模块本身不能擅自确认FREE
static void allow_roads(void)
{
    map_update_roads(0, ++scan_num, tick, 1);
    for (uint8_t r = 0; r < MAP_ROAD_COUNT; r++)
        map_update_confirm_free(r, scan_num, tick);
}

static void wait_for_move(float x, float y, float map_yaw)
{
    unsigned before = move_count;
    for (unsigned i = 0; i < 15 && move_count == before; i++)
        step(100, x, y, map_yaw);
}

static void settle_at(float x, float y, float map_yaw)
{
    mode = CHASSIS_IDLE;
    for (unsigned i = 0; i < 8; i++)
        step(100, x, y, map_yaw);
}

static void test_segment_progress(void)
{
    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    test_check(map_nav_start(1, 7, map_unknown_allow, 100), "accept valid start");
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    test_check(move_count == 1 && fabsf(last_distance - 92.5f) < 0.01f,
               "one road is one 92.5cm command");
    test_check(result().current_node == 1 && result().road_index == 0,
               "issuing movement does not advance current node");
    for (unsigned i = 0; i < 5; i++) step(50, 1200.0f, 1800.0f, -180.0f);
    test_check(move_count == 1, "main loop never repeats movement command");
    mode = CHASSIS_IDLE;
    step(10, 1200.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_arrive && result().current_node == 1,
               "busy ending midway cannot report arrival");
    step(100, 1200.0f, 1200.0f, -180.0f);
    test_check(result().current_node == 1, "single target sample cannot confirm arrival");
    for (unsigned i = 0; i < 10; i++) map_nav_task();
    test_check(result().current_node == 1, "same pose cannot accumulate settling time");
    settle_at(1200.0f, 1200.0f, -180.0f);
    test_check(result().current_node == 4 && result().road_index == 1 && move_count == 1,
               "confirmed arrival advances exactly one node and waits for observation");

    map_road_set(8, road_blocked);
    allow_roads();
    wait_for_move(1200.0f, 1200.0f, -180.0f);
    test_check(move_count == 2 && result().road_id == 9 && result().current_node == 4,
               "blocked road behind vehicle does not invalidate remaining route");
    settle_at(1200.0f, 275.0f, -180.0f);
    test_check(result().state == map_nav_finish && result().current_node == 7 &&
               result().road_index == 2 && move_count == 2,
               "second actual arrival completes route");
}

static void test_permission_and_alignment(void)
{
    setup(1200.0f, 2125.0f, -180.0f);
    map_nav_start(1, 4, map_unknown_allow, 100);
    for (unsigned i = 0; i < 8; i++) step(100, 1200.0f, 2125.0f, -180.0f);
    test_check(result().state == map_nav_scan && move_count == 0,
               "unknown roads wait without moving");
    map_road_set(8, road_free);
    step(100, 1200.0f, 2125.0f, -180.0f);
    test_check(move_count == 0, "stored FREE is insufficient for movement");
    allow_roads();
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    test_check(move_count == 1, "fresh explicit permission permits movement");

    setup(1200.0f, 2125.0f, -90.0f);
    yaw_offset = 45.0f;
    map_pose_yaw_offset_set(yaw_offset);
    publish(1200.0f, 2125.0f, -90.0f);
    test_check(fabsf(map_pose_yaw_offset_get() - 45.0f) < 0.01f, "read actual configured yaw offset");
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    for (unsigned i = 0; i < 8 && rotate_count == 0; i++)
        step(100, 1200.0f, 2125.0f, -90.0f);
    test_check(rotate_count > 0 && fabsf(last_target - (-225.0f)) < 0.01f && move_count == 0,
               "map heading converts to nearest IMU target with custom offset");
    for (unsigned i = 0; i < 8; i++) step(100, 1200.0f, 2125.0f, -180.0f);
    test_check(result().state == map_nav_scan && move_count == 0,
               "rotation completion returns to fresh road observation");
    allow_roads();
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    test_check(move_count == 1, "movement starts after realignment and new permission");
}

static void test_replan_and_errors(void)
{
    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    map_nav_task();
    map_road_set(8, road_blocked);
    for (unsigned i = 0; i < 10 && rotate_count == 0; i++)
        step(100, 1200.0f, 2125.0f, -180.0f);
    test_check(result().path.node_count == 4 && result().next_node == 0 && move_count == 0,
               "blocked future road replans from confirmed node");

    setup(1200.0f, 2125.0f, -180.0f);
    map_road_set(0, road_blocked);
    map_road_set(1, road_blocked);
    map_road_set(8, road_blocked);
    map_nav_start(1, 4, map_unknown_allow, 100);
    map_nav_task();
    test_check(result().state == map_nav_stop && result().error == map_nav_error_no_path && move_count == 0,
               "no route stops with reason");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    map_road_set(8, road_blocked);
    step(10, 1200.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_road &&
               result().current_node == 1 && stop_count == 1,
               "current road blocked midway stops without pretending to be at a node");
    map_nav_task();
    test_check(stop_count == 1, "stopped task does not repeatedly send stop commands");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    tick += MAP_POSE_TIMEOUT_MS + 1;
    map_nav_task();
    test_check(result().state == map_nav_stop && result().error == map_nav_error_pose && stop_count == 1,
               "stale localization stops active movement");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    step(MAP_NAV_MOVE_TIMEOUT_MS + 1, 1200.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_timeout,
               "motion timeout stops even with fresh localization");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    step(10, 1350.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_deviation,
               "leaving planned segment stops");

    setup(1200.0f, 2125.0f, -180.0f);
    map_nav_start(1, 4, map_unknown_allow, 100);
    map_nav_task();
    step(MAP_NAV_WAIT_TIMEOUT_MS + 1, 1200.0f, 2125.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_timeout,
               "observation wait has a timeout");
}

static void test_start_and_cancel(void)
{
    setup(1200.0f, 2125.0f, -180.0f);
    test_check(!map_nav_get_result(NULL), "null result rejected");
    test_check(!map_nav_start(255, 4, map_unknown_allow, 100), "invalid start rejected");
    test_check(!map_nav_start(1, 4, (map_unknown_mode_t)2, 100), "invalid policy rejected");
    test_check(!map_nav_start(1, 4, map_unknown_allow, 0), "zero speed rejected");
    test_check(!map_nav_start(1, 4, map_unknown_allow, MAP_NAV_MAX_SPEED + 1), "excessive speed rejected");
    test_check(!map_nav_start(0, 4, map_unknown_allow, 100) && move_count == 0,
               "start must match actual pose");
    mode = CHASSIS_MOVE;
    test_check(!map_nav_start(1, 4, map_unknown_allow, 100) && stop_count == 0,
               "busy chassis belonging to another task is not taken over");
    mode = CHASSIS_IDLE;
    map_nav_start(1, 4, map_unknown_allow, 100);
    test_check(!map_nav_start(1, 7, map_unknown_allow, 100) && result().target_node == 4,
               "active task rejects replacement without cancellation");
    map_nav_cancel();
    map_nav_cancel();
    test_check(result().state == map_nav_stop && result().error == map_nav_error_cancel && stop_count == 1,
               "cancellation is idempotent");

    setup(1200.0f, 2125.0f, -180.0f);
    map_nav_start(1, 1, map_unknown_allow, 100);
    settle_at(1200.0f, 2125.0f, -180.0f);
    test_check(result().state == map_nav_finish && move_count == 0 && rotate_count == 0,
               "same start and goal settles without issuing motion");
}

static void test_directions_and_settling(void)
{
    const uint8_t goals[] = {1, 5, 7, 3};
    const float headings[] = {0.0f, -90.0f, -180.0f, 90.0f};
    for (uint8_t i = 0; i < 4; i++)
    {
        setup(1200.0f, 1200.0f, headings[i]);
        allow_roads();
        map_nav_start(4, goals[i], map_unknown_allow, 100);
        wait_for_move(1200.0f, 1200.0f, headings[i]);
        test_check(move_count == 1 && rotate_count == 0 && fabsf(last_distance - 92.5f) < 0.01f,
                   "all four aligned headings command forward one road");
    }

    setup(1220.0f, 1200.0f, -90.0f);
    allow_roads();
    map_nav_start(4, 5, map_unknown_allow, 100);
    wait_for_move(1220.0f, 1200.0f, -90.0f);
    test_check(move_count == 1 && fabsf(last_distance - 90.5f) < 0.01f,
               "distance is computed from actual pose instead of assuming exact node center");
    for (unsigned i = 0; i < 5; i++) step(100, 2125.0f, 1200.0f, -90.0f);
    test_check(result().state == map_nav_move && result().current_node == 4,
               "target coordinates alone cannot finish while chassis remains busy");
    mode = CHASSIS_IDLE;
    step(100, 2125.0f, 1200.0f, -90.0f);
    for (unsigned i = 0; i < 6; i++)
        step(100, i % 2 ? 2115.0f : 2125.0f, 1200.0f, -90.0f);
    test_check(result().current_node == 4, "target pose jitter resets settling confirmation");
    settle_at(2125.0f, 1200.0f, -90.0f);
    test_check(result().state == map_nav_finish && result().current_node == 5,
               "stable target pose eventually completes arrival");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    mode = CHASSIS_IDLE;
    step(100, 1200.0f, 1800.0f, -180.0f);
    step(MAP_NAV_WAIT_TIMEOUT_MS + 1, 1200.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_timeout && move_count == 1,
               "premature busy completion waits for actual arrival then times out without retrying");

    setup(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    mode = CHASSIS_IDLE;
    step(100, 1200.0f, 1800.0f, -180.0f);
    mode = CHASSIS_MOVE;
    step(100, 1200.0f, 1800.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_busy,
               "unexpected movement during arrival confirmation stops conflicting control");

    setup(1200.0f, 2125.0f, -90.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    for (unsigned i = 0; i < 8 && rotate_count == 0; i++)
        step(100, 1200.0f, 2125.0f, -90.0f);
    step(MAP_NAV_WAIT_TIMEOUT_MS + 1, 1200.0f, 2125.0f, -90.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_timeout,
               "rotation timeout stops without translating");

    setup(1200.0f, 2125.0f, -180.0f);
    map_nav_start(1, 4, map_unknown_allow, 100);
    map_pose_yaw_offset_set(0.0f);
    yaw_offset = 0.0f;
    step(100, 1200.0f, 2125.0f, -180.0f);
    test_check(result().state == map_nav_stop && result().error == map_nav_error_pose,
               "changing yaw reference during active task requires restarting navigation");

    setup(1200.0f, 2125.0f, -180.0f);
    tick = UINT32_MAX - 200;
    publish(1200.0f, 2125.0f, -180.0f);
    allow_roads();
    map_nav_start(1, 4, map_unknown_allow, 100);
    wait_for_move(1200.0f, 2125.0f, -180.0f);
    test_check(move_count == 1 && result().state == map_nav_move,
               "settling and phase timers allow millisecond counter rollover");
}

int main(void)
{
    test_segment_progress();
    test_permission_and_alignment();
    test_replan_and_errors();
    test_start_and_cancel();
    test_directions_and_settling();
    printf("map nav: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
