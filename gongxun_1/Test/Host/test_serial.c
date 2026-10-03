// 实际驱动源码，替身只提供可控HAL返回值、DMA内容和时间。
#include "serial_headfile/headfile.h"
#include "../../BSP/Src/camera.c"
#include "../../BSP/Src/hwt101.c"
#include "../../BSP/Src/debug_printf.c"
#include "../../BSP/Src/hmi.c"
#include "../../APP/Src/hmi_app.c"

static DMA_HandleTypeDef imu_dma, camera_dma, log_dma;
UART_HandleTypeDef huart3 = {USART3, &imu_dma, HAL_UART_STATE_READY};
static UART_HandleTypeDef camera_uart = {USART1, &camera_dma, HAL_UART_STATE_READY};
static UART_HandleTypeDef log_uart = {0, &log_dma, HAL_UART_STATE_READY};
UART_HandleTypeDef huart5 = {UART5, &log_dma, HAL_UART_STATE_READY};
static uint8_t *screen_dma_buffer;
static uint16_t screen_dma_len;
static uint32_t now_tick, primask;
static uint16_t rx_failures, tx_failures, tx_calls, rx_calls, dma_calls, dma_failures;
static uint8_t inject_imu_frame;
static HAL_UART_RxEventTypeTypeDef rx_event;
static uint16_t checks, failures;
HostTaskState task_state;
bool car_run, car_reset;
uint8_t start_location;
static uint8_t host_obstacle[BFS_MAP_SIZE][BFS_MAP_SIZE];

static void check(uint8_t ok, const char *name) {
    checks++;
    if (!ok) failures++;
    printf("[SERIAL] %s %s\n", ok ? "PASS" : "FAIL", name);
}

uint32_t host_primask_get(void) { return primask; }
void host_primask_set(uint32_t value) { primask = value; }
void led_on(void) {}
void HAL_GPIO_WritePin(void *port, uint16_t pin, uint32_t state) { (void)port; (void)pin; (void)state; }
void HAL_GPIO_TogglePin(void *port, uint16_t pin) { (void)port; (void)pin; }
void bfs_obstacle_clear(void) { memset(host_obstacle, 0, sizeof(host_obstacle)); }
void bfs_set_obstacle(uint8_t x, uint8_t y, uint8_t value) { host_obstacle[x][y] = value; }
uint8_t Emm_V5_Tx_Count(void) { return 0; }
void catch_lift(float cm) { (void)cm; }
void catch_stretch_pos(float cm) { (void)cm; }

uint32_t HAL_GetTick(void) {
    if (inject_imu_frame) now_tick++;
    return now_tick;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    rx_calls++;
    if (rx_failures) { rx_failures--; return HAL_ERROR; }
    uart->hdmarx->remaining = size;
    if (uart == &huart3 && inject_imu_frame) {
        uint8_t frame[11] = {0x55, 0x53, 0, 0, 0, 0, 0, 0x20, 0, 0, 0};
        for (uint8_t i = 0; i < 10; i++) frame[10] += frame[i];
        memcpy(buffer, frame, sizeof(frame));
        uart->hdmarx->remaining = size - sizeof(frame);
        hwt101_rx_flag = 1;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart) { (void)uart; return rx_event; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t len, uint32_t timeout) {
    (void)uart; (void)buffer; (void)len; (void)timeout;
    tx_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t len) {
    (void)uart; (void)buffer; (void)len;
    tx_calls++;
    if (tx_failures) { tx_failures--; return HAL_BUSY; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t len) {
    (void)uart; (void)buffer; (void)len;
    dma_calls++;
    if (dma_failures) { dma_failures--; return HAL_BUSY; }
    if (uart->Instance == UART5) {
        screen_dma_buffer = buffer;
        screen_dma_len = len;
        uart->gState = 1;
    }
    return HAL_OK;
}

static void test_rx_restart(void) {
    rx_failures = 1;
    hwt101_init(&huart3);
    check(hwt101_error_flag, "imu initial receive failure requests retry");
    uint16_t before = rx_calls;
    hwt101_rx_handle();
    check(!hwt101_error_flag && rx_calls == before + 1, "imu initial retry succeeds");
    hwt101_error_callback(&huart3);
    rx_failures = 1;
    hwt101_rx_handle();
    check(hwt101_error_flag, "imu restart failure preserves request");
    before = rx_calls;
    hwt101_rx_handle();
    check(!hwt101_error_flag && rx_calls == before + 1, "imu failed restart retries successfully");

    rx_failures = 1;
    cam_init(&camera_uart);
    check(cam_error_flag, "camera initial receive failure requests retry");
    before = rx_calls;
    cam_rx_handle();
    check(!cam_error_flag && rx_calls == before + 1, "camera initial retry succeeds");
    cam_error_callback(&camera_uart);
    rx_failures = 1;
    cam_rx_handle();
    check(cam_error_flag, "camera restart failure preserves request");
    before = rx_calls;
    cam_rx_handle();
    check(!cam_error_flag && rx_calls == before + 1, "camera failed restart retries successfully");
}

static void test_imu_startup(void) {
    now_tick = 0;
    inject_imu_frame = 1;
    my_hwt101_init();
    inject_imu_frame = 0;
    check(hwt101_data_valid() && fabsf(hwt101.yaw - 45.0f) < 0.01f && now_tick < 100,
          "imu startup parses received frame before timeout");
    check(hwt101_rst_request, "valid startup requests sensor yaw reset");
}

static void test_reset_tx(void) {
    hwt101_rst_state = HWT101_RST_IDLE;
    hwt101_rst_request = 1;
    tx_failures = 1;
    now_tick = 100;
    hwt101_tick();
    check(hwt101_rst_state == HWT101_RST_UNLOCK, "failed unlock send cannot advance reset");
    hwt101_tick();
    check(hwt101_rst_state == HWT101_RST_WAIT_UNLOCK && hwt101_rst_tick == 100,
          "successful unlock starts wait period");
    now_tick = 300;
    hwt101_tick();
    tx_failures = 1;
    hwt101_tick();
    check(hwt101_rst_state == HWT101_RST_ZERO, "failed zero send stays pending");
    hwt101_tick();
    now_tick = 800;
    hwt101_tick();
    tx_failures = 1;
    hwt101_tick();
    check(hwt101_rst_state == HWT101_RST_SAVE, "failed save send stays pending");
    hwt101_has_prev_wz = 1;
    hwt101_tick();
    check(hwt101_rst_state == HWT101_RST_IDLE, "successful save completes command sequence");
    check(!hwt101_has_prev_wz, "sensor reset also restarts gyro derivative baseline");
}

static void test_debug_queue(void) {
    static uint8_t message[DEBUG_TX_BUF_SIZE - 1];
    memset(message, 'A', sizeof(message));
    debug_uart_init(&log_uart);
    dma_calls = 0;
    dma_failures = 1;
    check(debug_printf_write(message, sizeof(message)) == sizeof(message) && !debug_dma_busy,
          "failed log DMA retains queued data");
    check(debug_printf_write((const uint8_t *)"B", 1) == 0 && debug_dma_busy && dma_calls == 2,
          "full log queue still retries sending instead of deadlocking");
    debug_tx_callback(&log_uart);
    check(debug_rd == debug_wr && !debug_dma_busy, "log completion releases queued bytes");
    dma_failures = 1;
    debug_printf_write((const uint8_t *)"last line", 9);
    check(!debug_dma_busy, "last log line can remain pending after DMA busy");
    debug_printf_task();
    check(debug_dma_busy, "periodic log task retries without requiring a new log line");
    debug_tx_callback(&log_uart);
    uint16_t before = tx_calls;
    maix_send_frame(MAIX_CMD_BLOB, NULL, 1);
    check(tx_calls == before, "null command payload rejects before transmit");
}

static void test_recovery_results(void) {
    maix_now_cmd = MAIX_CMD_BLOB;
    maix_now_target = MAIX_COLOR_RED;
    maix_result.valid = 1;
    maix_filter_count = MAIX_WINDOW_SIZE;
    maix_has_last_frame = 1;
    maix_vision.new_data = 1;
    qr_ok = 1;
    cam_error_callback(&camera_uart);
    cam_rx_handle();
    check(!maix_result.valid && !maix_filter_count && !maix_has_last_frame && !maix_vision.new_data && !qr_ok,
          "camera error clears old raw results and filter history");
    check(maix_result.new_data && maix_result.cmd == maix_now_cmd && maix_result.target == maix_now_target,
          "camera recovery reports invalidation and preserves requested target");
    hwt101.valid = 1;
    hwt101_error_callback(&huart3);
    hwt101_rx_handle();
    check(!hwt101_data_valid(), "imu error invalidates old received data");
}

static void test_screen_dma(void) {
    uint8_t first[HMI_TX_SIZE];
    char long_text[HMI_TX_SIZE + 20];
    uint16_t before = dma_calls;
    hmi_SendCmd("page initial");
    check(dma_calls == before, "uninitialized screen send is ignored");
    hmi_init(&huart5);
    dma_failures = 0;
    hmi_SendCmd("page 1");
    memcpy(first, screen_dma_buffer, screen_dma_len);
    before = dma_calls;
    hmi_SendCmd("different command");
    check(dma_calls == before && !memcmp(first, screen_dma_buffer, screen_dma_len),
          "busy screen command cannot overwrite active DMA bytes");
    check(screen_dma_buffer == (uint8_t *)hmi_txbuffer && screen_dma_len == 9 &&
          screen_dma_buffer[6] == 0xFF && screen_dma_buffer[8] == 0xFF,
          "screen DMA reads persistent buffer and three end bytes");
    huart5.gState = HAL_UART_STATE_READY;
    memset(long_text, 'A', sizeof(long_text) - 1);
    long_text[sizeof(long_text) - 1] = 0;
    before = dma_calls;
    hmi_SendCmd("%s", long_text);
    check(dma_calls == before, "overlength screen command rejects instead of sending truncated syntax");
    hmi_SendCmd(NULL);
    check(dma_calls == before, "null screen format rejects");
    memset(long_text, 'B', HMI_TX_SIZE - HMI_END_CMD_LEN);
    long_text[HMI_TX_SIZE - HMI_END_CMD_LEN] = 0;
    hmi_SendCmd("%s", long_text);
    check(screen_dma_len == HMI_TX_SIZE && screen_dma_buffer[HMI_TX_SIZE - 1] == 0xFF && !primask,
          "maximum screen command fits and restores interrupt state");
}

static void screen_feed(const uint8_t *data, uint16_t size) {
    memcpy(hmi_rxbuffer, data, size);
    hmi_solve(&huart5, size);
}

static void test_screen_receive(void) {
    const uint8_t start[] = {0x65, 1, 4, 0, 0xFF, 0xFF, 0xFF};
    const uint8_t joined[] = {0, 0xBB, 2, 3, 0xFF, 0xFF, 0xFF, 0xCC, 1, 0xFF, 0xFF, 0xFF};
    const uint8_t lift[] = {0xDD, 0x11, 0, 0x64, 0, 0xFF, 0xFF, 0xFF};
    const uint8_t invalid_start[] = {0xCC, 2, 0xFF, 0xFF, 0xFF};
    const uint8_t invalid_lift[] = {0xDD, 0x11, 1, 0, 0x80, 0xFF, 0xFF, 0xFF};
    rx_failures = 1;
    hmi_init(&huart5);
    check(hmi_error_flag, "screen initial receive failure requests retry");
    uint16_t before = rx_calls;
    hmi_rx_task();
    check(!hmi_error_flag && rx_calls == before + 1, "screen periodic task recovers initial receive");
    car_run = false;
    car_reset = true;
    screen_feed(start, 3);
    check(!car_run, "screen incomplete touch cannot start vehicle");
    screen_feed(start + 3, sizeof(start) - 3);
    check(car_run && !car_reset, "screen touch split across receive batches parses once");
    screen_feed(joined, sizeof(joined));
    check(host_obstacle[2][3] && task_state.cur_x == 4 && task_state.task_location == PARKING_1,
          "screen noise plus two concatenated commands both dispatch");
    hmi_lift_state = 0;
    screen_feed(lift, 1);
    screen_feed(lift + 1, sizeof(lift) - 1);
    check(hmi_lift_state && hmi_lift_mm == -100, "screen fragmented lift command preserves signed distance");
    task_state.cur_x = 2;
    screen_feed(invalid_start, sizeof(invalid_start));
    check(task_state.cur_x == 2, "invalid screen parking choice cannot change current coordinate");
    hmi_lift_state = 0;
    screen_feed(invalid_lift, sizeof(invalid_lift));
    check(!hmi_lift_state, "screen distance exceeding signed range rejects");
    screen_feed(start, 3);
    hmi_error_callback(&huart5);
    rx_failures = 1;
    hmi_rx_task();
    check(hmi_error_flag && !hmi_frame_idx, "screen error clears partial frame and retains failed restart");
    hmi_rx_task();
    check(!hmi_error_flag, "screen failed restart retries successfully");
    car_run = false;
    screen_feed(start + 3, sizeof(start) - 3);
    check(!car_run, "screen cannot join pre-error prefix to post-error suffix");
    screen_feed(start, sizeof(start));
    check(car_run, "screen complete touch works after recovery");
    rx_failures = 1;
    hmi_rx_callback(&huart5, 0);
    check(hmi_error_flag, "normal receive callback preserves restart failure");
    hmi_rx_task();
    check(!hmi_error_flag && !primask, "screen callback failure recovers and restores interrupts");
    car_run = false;
    screen_feed(start, 3);
    now_tick += HMI_FRAME_TIMEOUT_MS + 1;
    screen_feed(start + 3, sizeof(start) - 3);
    check(!car_run, "expired screen prefix cannot combine with late suffix");
    screen_feed(start, sizeof(start));
    check(car_run, "complete screen command resynchronizes after partial timeout");
}

static void test_gyro_difference(void) {
    uint8_t frame[11] = {0x55, 0x52, 0, 0, 0, 0, 0, 0x10, 0, 0, 0};
    hwt101_init(&huart3);
    now_tick = 500;
    for (uint8_t i = 0; i < 10; i++) frame[10] += frame[i];
    for (uint8_t i = 0; i < 11; i++) hwt101_byte_handle(frame[i]);
    check(hwt101_data_valid() && hwt101.yaw_accel == 0, "first gyro sample cannot differentiate against invented zero");
    now_tick += 10;
    frame[7] = 0x20;
    frame[10] = 0;
    for (uint8_t i = 0; i < 10; i++) frame[10] += frame[i];
    for (uint8_t i = 0; i < 11; i++) hwt101_byte_handle(frame[i]);
    check(fabsf(hwt101.yaw_accel - 25000.0f) < 0.1f, "two real gyro samples use actual time difference");
    hwt101_error_callback(&huart3);
    hwt101_rx_handle();
    now_tick += 10;
    for (uint8_t i = 0; i < 11; i++) hwt101_byte_handle(frame[i]);
    check(hwt101.yaw_accel == 0, "gyro difference restarts after receive error");
}

int main(void) {
    test_rx_restart();
    test_imu_startup();
    test_reset_tx();
    test_debug_queue();
    test_recovery_results();
    test_screen_dma();
    test_screen_receive();
    test_gyro_difference();
    printf("[SERIAL] RESULT %u passed, %u failed\n", checks - failures, failures);
    return failures != 0;
}
