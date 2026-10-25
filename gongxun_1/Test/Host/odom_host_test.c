#include "headfile.h"
#include <stdlib.h>

static uint32_t tick;
static unsigned checks;
static DMA_HandleTypeDef dma2, dma3;
UART_HandleTypeDef huart2 = {USART2, &dma2, HAL_UART_STATE_READY};
UART_HandleTypeDef huart3 = {USART3, &dma3, HAL_UART_STATE_READY};
Task_State task_state;
bool car_run;
static uint8_t key_pressed;
static unsigned pose_task_count;
static uint8_t map_task_pose_valid;
static uint8_t *rx2;
static uint8_t tx_addr[128], tx_cmd[128];
static unsigned tx_count;

#define CHECK(expr) do { checks++; if (!(expr)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)
#define NEAR(a, b) (fabsf((a) - (b)) < 0.02f)
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(void *port, uint16_t pin) {
    CHECK(port == GPIOC && pin == GPIO_PIN_1);
    return key_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
const lidar_scan_t *lidar_get_scan(void) { return NULL; }
uint32_t lidar_get_scan_drop_cnt(void) { return 0; }
void openmv_qr_test(void) { CHECK(0); }
void maix_blob_test(void) { CHECK(0); }
uint8_t obstacle_find_task(float x, float y, float yaw) {
    (void)x; (void)y; (void)yaw; CHECK(0); return 0;
}
uint8_t map_obstacle_pose_task(void) {
    map_pose_t pose;
    pose_task_count++;
    map_task_pose_valid = map_pose_get(tick, &pose);
    return 0;
}
void HAL_Delay(uint32_t delay) {
    tick += delay;
    while (Emm_V5_Tx_Count()) Emm_V5_tx_callback(&huart2);
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len) {
    (void)uart; (void)len;
    if (tx_count < 128) { tx_addr[tx_count] = data[0]; tx_cmd[tx_count++] = data[1]; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len) {
    (void)uart; (void)data; (void)len; return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len) {
    if (uart == &huart2) rx2 = data;
    uart->hdmarx->remaining = len; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart) {
    (void)uart; return HAL_UART_RXEVENT_IDLE;
}

static void receive(const uint8_t *data, uint16_t len) {
    memcpy(rx2, data, len);
    Emm_V5_rx_callback(&huart2, len);
}
static void position(uint8_t addr, int32_t pos) {
    uint32_t raw = pos < 0 ? (uint32_t)(-(int64_t)pos) : (uint32_t)pos;
    uint8_t frame[] = {addr, 0x36, pos < 0, raw >> 24, raw >> 16, raw >> 8, raw, 0x6B};
    receive(frame, sizeof(frame));
}
static void angle(uint8_t type, int16_t raw) {
    uint8_t frame[] = {0x55, type, 0, 0, 0, 0, raw, (uint16_t)raw >> 8, 0, 0, 0};
    for (unsigned i = 0; i < 10; i++) frame[10] += frame[i];
    for (unsigned i = 0; i < sizeof(frame); i++) hwt101_byte_handle(frame[i]);
    imu_get();
}
static void wheels(int32_t a, int32_t b, int32_t c, int32_t d) {
    position(1, a); position(2, b); position(3, c); position(4, d);
    chassis_odom_fetch(); chassis_odom_update();
}
static void origin(float yaw) {
    tick += 1000;
    hwt101_init(&huart3);
    angle(0x53, 0); my_yaw = yaw;
    CHECK(chassis_odom_origin_set(100.0f, 100.0f));
    tick += 100;
    angle(0x53, 0); my_yaw = yaw;
    wheels(0, 0, 0, 0);
}
static void protocol_test(void) {
    int32_t pos; uint32_t stamp;
    const uint8_t frame[] = {1, 0x36, 1, 0, 1, 0, 0, 0x6B};
    tick = 1000;
    receive(frame, 3);
    CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    receive(frame + 3, 5);
    CHECK(Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    CHECK(pos == -65536 && stamp == tick);
    CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    for (unsigned split = 1; split < 8; split++) {
        Emm_V5_Rx_Reset();
        receive(frame, split); receive(frame + split, 8 - split);
        CHECK(Emm_V5_Get_Pos_Sample(1, &pos, &stamp) && pos == -65536);
    }
    position(2, 0x12345678);
    CHECK(Emm_V5_Get_Pos_Sample(2, &pos, &stamp) && pos == 0x12345678);
    position(3, INT32_MIN);
    CHECK(Emm_V5_Get_Pos_Sample(3, &pos, &stamp) && pos == INT32_MIN);
    position(6, 65536);
    CHECK(Emm_V5_Get_Pos(6, &pos) && pos == 256); // 保持夹取模块既有标定读数
    position(6, -65536);
    CHECK(Emm_V5_Get_Pos(6, &pos) && pos == 256); // 旧接口继续沿用原标定语义
    receive(frame, 4); Emm_V5_Rx_Reset(); receive(frame + 4, 4);
    CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    position(1, 1); CHECK(!Emm_V5_Get_Pos_Sample(1, NULL, &stamp));
    CHECK(Emm_V5_Get_Pos_Sample(1, &pos, &stamp) && pos == 1);
    uint8_t bad[] = {1, 0x36, 2, 0, 1, 0, 0, 0x6B};
    receive(bad, 8); CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    bad[2] = 0; bad[7] = 0;
    receive(bad, 8); CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    bad[7] = 0x6B; bad[3] = 0x80;
    receive(bad, 8); CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    uint8_t noise[] = {0xFF, 0x00, 0x03};
    receive(noise, 3); position(4, -1234567);
    CHECK(Emm_V5_Get_Pos_Sample(4, &pos, &stamp) && pos == -1234567);
    uint8_t pair[] = {1, 0x36, 0, 0, 0, 0, 1, 0x6B, 2, 0x36, 0, 0, 0, 0, 2, 0x6B};
    receive(pair, 16);
    CHECK(Emm_V5_Get_Pos_Sample(1, &pos, &stamp) && pos == 1);
    CHECK(Emm_V5_Get_Pos_Sample(2, &pos, &stamp) && pos == 2);
    receive(frame, 4); tick += 21; receive(frame + 4, 4);
    CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
    Emm_V5_Rx_Reset(); position(1, 3); Emm_V5_Rx_Reset();
    CHECK(!Emm_V5_Get_Pos_Sample(1, &pos, &stamp));
}
static void odom_test(void) {
    const float circumference = 3.14159265f * 7.9f;
    map_pose_t pose;
    for (unsigned heading = 0; heading < 4; heading++) {
        float yaw = (float)heading * 90.0f;
        origin(yaw);
        CHECK(map_pose_get(tick, &pose));
        CHECK(NEAR(pose.x_mm, 1000) && NEAR(pose.y_mm, 1000));
        tick += 80; angle(0x53, 0); my_yaw = yaw;
        position(1, 65536); position(2, -65536); position(3, 65536);
        chassis_odom_fetch(); chassis_odom_update();
        CHECK(NEAR(task_state.x_cm, 100) && NEAR(task_state.y_cm, 100));
        position(4, -65536); chassis_odom_fetch(); chassis_odom_update();
        CHECK(NEAR(task_state.x_cm, 100 - circumference * sinf(yaw * PI / 180)));
        CHECK(NEAR(task_state.y_cm, 100 + circumference * cosf(yaw * PI / 180)));
        CHECK(map_pose_get(tick, &pose) && pose.tick == tick);
        uint32_t stamp = pose.tick;
        tick += 10; chassis_odom_update();
        CHECK(map_pose_get(tick, &pose) && pose.tick == stamp);
        tick += 100; angle(0x53, 0); my_yaw = yaw;
        wheels(0, 0, 0, 0);
        CHECK(NEAR(task_state.x_cm, 100) && NEAR(task_state.y_cm, 100));
    }
    origin(0);
    tick += 80; angle(0x53, 0);
    wheels(65536, 65536, -65536, -65536);
    CHECK(NEAR(task_state.x_cm, 100 + circumference / 1.10f) && NEAR(task_state.y_cm, 100));
    chassis_odom_hold(); task_state.odom_on = 1;
    tick += 600; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(!map_pose_get(tick, &pose)); // 不能用旧坐标自动恢复全局位姿
    origin(0);
    tick += 501; angle(0x53, 0); chassis_odom_update();
    CHECK(!map_pose_get(tick, &pose));
    wheels(65536, -65536, 65536, -65536);
    CHECK(!map_pose_get(tick, &pose));
    origin(0);
    tick += 101; angle(0x52, 0); wheels(65536, -65536, 65536, -65536);
    CHECK(!map_pose_get(tick, &pose)); // 角速度帧不能延长角度有效期
    origin(0);
    chassis_odom_reset(); tick += 100; angle(0x53, 0); wheels(999, 999, 999, 999);
    CHECK(NEAR(task_state.x_cm, 100) && NEAR(task_state.y_cm, 100));
    CHECK(map_pose_get(tick, &pose)); // 仅编码器清零不改全局坐标
    CHECK(!chassis_odom_origin_set(NAN, 10));
    chassis_odom_update();
    CHECK(!map_pose_get(tick, &pose));
}
static void freshness_test(void) {
    map_pose_t pose;
    origin(0);
    tick += 80;
    angle(0x53, 0);
    wheels(0, 0, 0, 0); // 停车时的新反馈可以发布新位姿
    CHECK(map_pose_get(tick, &pose) && pose.tick == tick);
    uint32_t stamp = tick;
    tick += 20;
    angle(0x53, 0); position(1, 0);
    chassis_odom_fetch(); chassis_odom_update();
    CHECK(map_pose_get(tick, &pose) && pose.tick == stamp);
    tick += 81;
    angle(0x53, 0); position(2, 0); position(3, 0); position(4, 0);
    chassis_odom_fetch(); chassis_odom_update();
    CHECK(map_pose_get(tick, &pose) && pose.tick == stamp); // 四轮时间差过大不更新

    origin(0);
    stamp = tick;
    tick += 40; position(1, 0); position(2, 0); position(3, 0); position(4, 0);
    tick += 20; angle(0x53, 0); chassis_odom_fetch(); chassis_odom_update();
    CHECK(map_pose_get(tick, &pose) && pose.tick == stamp + 40); // 使用收到反馈的时间

    // 清零前留在软件缓存的旧反馈  不能成为清零后的基准
    position(1, 500); position(2, 500); position(3, 500); position(4, 500);
    tick += 1; chassis_odom_reset(); tick += 100; angle(0x53, 0);
    chassis_odom_fetch(); chassis_odom_update();
    CHECK(!map_pose_get(tick, &pose));
    wheels(0, 0, 0, 0);
    CHECK(map_pose_get(tick, &pose) && NEAR(pose.x_mm, 1000));

    origin(0);
    angle(0x52, 0); CHECK(hwt101_yaw_valid());
    hwt101_error_callback(&huart3);
    CHECK(!hwt101_yaw_valid());
    chassis_odom_update(); CHECK(!map_pose_get(tick, &pose));

    origin(0);
    hwt101_error_callback(&huart3); hwt101_rx_handle(); angle(0x53, 0);
    CHECK(hwt101_yaw_valid());
    wheels(0, 0, 0, 0); CHECK(!map_pose_get(tick, &pose)); // 错误已恢复也不能漏掉中断

    origin(0);
    hwt101_yaw_reset_request();
    CHECK(!hwt101_yaw_valid());
    chassis_odom_update(); CHECK(!map_pose_get(tick, &pose));
    hwt101_tick(); tick += 200; hwt101_tick(); hwt101_tick();
    tick += 500; hwt101_tick(); hwt101_tick();
    CHECK(!hwt101_yaw_valid()); // 保存完成后仍要等新角度帧
    angle(0x53, 0); CHECK(hwt101_yaw_valid());
    wheels(0, 0, 0, 0); CHECK(!map_pose_get(tick, &pose)); // 归零后需要重新确认起点

    // 在两次主循环之间完成整次归零  也不能漏掉朝向基准改变
    origin(0);
    hwt101_yaw_reset_request(); hwt101_tick(); tick += 200;
    hwt101_tick(); hwt101_tick(); tick += 500; hwt101_tick(); hwt101_tick();
    angle(0x53, 0); chassis_odom_reset(); tick += 100; angle(0x53, 0);
    wheels(0, 0, 0, 0); CHECK(!map_pose_get(tick, &pose));

    origin(0);
    CHECK(map_pose_yaw_offset_set(-90));
    chassis_odom_update(); CHECK(!map_pose_get(tick, &pose));
    origin(0); tick += 80; angle(0x53, 0);
    wheels(65536, -65536, 65536, -65536);
    CHECK(task_state.x_cm > 124 && NEAR(task_state.y_cm, 100));
    CHECK(map_pose_yaw_offset_set(MAP_POSE_DEFAULT_YAW_OFFSET));

    origin(0);
    chassis_odom_hold(); task_state.odom_on = 1;
    tick += 100; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(!map_pose_get(tick, &pose));
    CHECK(chassis_odom_origin_set(100, 100));
    car_speed_move(0, 1, 0);
    tick += 100; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(!map_pose_get(tick, &pose)); // 起点基准未建立就动了
    car_stop();
    for (unsigned i = 0; i < 20; i++) { car_run_tick(); HAL_Delay(1); }

    CHECK(chassis_odom_origin_set(100, 100));
    car_run_distance(100, 0, 10); // 尚未建立起点轮子基准就运动
    car_stop(); // 即使反馈到来时已经停车  也不能忘记中间的运动
    tick += 100; angle(0x53, 0); wheels(5000, -5000, 5000, -5000);
    CHECK(!map_pose_get(tick, &pose));
    for (unsigned i = 0; i < 20; i++) { car_run_tick(); HAL_Delay(1); }

    // 毫秒计数回绕不会使新反馈变成旧反馈
    tick = UINT32_MAX - 150;
    hwt101_init(&huart3); angle(0x53, 0);
    CHECK(chassis_odom_origin_set(100, 100));
    tick += 100; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(map_pose_get(tick, &pose));
    tick += 80; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(map_pose_get(tick, &pose) && pose.tick == tick);
}
static void motor_test(void) {
    Emm_V5_Tx_Reset(); tx_count = 0;
    motor_set_speed((Motor_Num)-1, 0);
    motor_set_speed((Motor_Num)STEP_MOTOR_COUNT, 0);
    CHECK(tx_count == 0);
    step_motor_init();
    unsigned stops = 0;
    for (unsigned i = 0; i < tx_count; i++) if (tx_cmd[i] == 0xF6) {
        CHECK(tx_addr[i] == ++stops);
    }
    CHECK(stops == 6);
    tx_count = 0; tick += 100;
    chassis_odom_poll();
    CHECK(tx_count == 1 && tx_cmd[0] == 0x36); // 静止也要读取反馈
}
static void key_step(uint8_t pressed, uint32_t dt) {
    key_pressed = pressed;
    tick += dt;
    angle(0x53, 0);
    map_pose_test();
}
static void key_press(void) {
    key_step(1, 0); key_step(1, 35);
    key_step(0, 0); key_step(0, 35);
}
static void pose_key_test(void) {
    map_pose_t pose;
    origin(0); map_update_init(); car_run = false;
    CHECK(map_road_set(11, road_blocked));
    CHECK(map_update_roads((uint16_t)(1U << 8), 1, tick, 1));
    Emm_V5_Tx_Reset(); tx_count = 0;
    key_step(0, 0); key_step(0, 35);
    CHECK(NEAR(task_state.x_cm, 100) && NEAR(task_state.y_cm, 100));

    // 10ms抖动不能确认起点；保持按下也不能重复确认。
    key_step(1, 0); key_step(0, 10); key_step(0, 35);
    CHECK(NEAR(task_state.x_cm, 100));
    key_step(1, 0); key_step(1, 35); key_step(1, 100);
    CHECK(NEAR(task_state.x_cm, 100));
    key_step(0, 0); key_step(0, 35);
    CHECK(NEAR(task_state.x_cm, 225) && NEAR(task_state.y_cm, 15));
    CHECK(task_state.odom_on && !car_run && !car_is_busy() && tx_count == 0);
    CHECK(!map_task_pose_valid && !map_pose_get(tick, &pose)); // 起点确认不冒充四轮新反馈
    road_state_t road;
    CHECK(map_road_get(11, &road) && road == road_blocked); // 确认起点不擅自解封道路

    tick += 100; angle(0x53, 0); wheels(0, 0, 0, 0);
    CHECK(map_pose_get(tick, &pose) && NEAR(pose.x_mm, 2250) && NEAR(pose.y_mm, 150));
    CHECK(map_update_roads((uint16_t)(1U << 8), 2, tick, 1));
    CHECK(map_road_get(8, &road) && road == road_unknown); // 新旧起点的观察不能凑成两圈
    key_step(0, 40);
    CHECK(map_pose_get(tick, &pose)); // 一次按键只能确认一次

    // 运行中或IMU失效时拒绝设置；保持原来的起点，必须重新按键。
    origin(0); car_run = true; key_press();
    CHECK(NEAR(task_state.x_cm, 100) && car_run);
    car_run = false;
    car_run_distance(100, 0, 10); key_press();
    CHECK(NEAR(task_state.x_cm, 100) && car_is_busy());
    car_stop();
    for (unsigned i = 0; i < 20; i++) { car_run_tick(); HAL_Delay(1); }
    origin(0);
    car_speed_move(1, 0, 0); key_press(); // 速度模式可能显示IDLE，也要拒绝
    CHECK(NEAR(task_state.x_cm, 100));
    car_stop();
    for (unsigned i = 0; i < 20; i++) { car_run_tick(); HAL_Delay(1); }
    origin(0);
    key_step(1, 0); key_step(1, 35); key_step(0, 0);
    tick += 101; map_pose_test(); // 不补角度帧，模拟角度超时
    CHECK(NEAR(task_state.x_cm, 100));
    key_step(0, 35);
    CHECK(NEAR(task_state.x_cm, 100)); // IMU恢复不能自动补触发
    key_press();
    CHECK(NEAR(task_state.x_cm, 225) && NEAR(task_state.y_cm, 15) && !car_run);
    CHECK(pose_task_count > 20); // 等待或拒绝按键时仍然服务雷达地图入口
}
int main(void) {
    Emm_V5_Rx_Start(); map_pose_init(); chassis_init();
    protocol_test(); odom_test(); freshness_test(); motor_test(); pose_key_test();
    printf("Odometry host checks passed: %u\n", checks);
    return 0;
}
