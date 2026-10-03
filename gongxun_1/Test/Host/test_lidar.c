/* 直接编译现有雷达解析源码，硬件替身仅提供 DMA 字节流、时钟和 HAL 状态。 */
#include "lidar_headfile/headfile.h"
#include "../../BSP/Src/lidar.c"
#include "../../BSP/Src/uart_dma.c"

static DMA_HandleTypeDef host_dma;
UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5, huart6;
static uint32_t host_tick, host_primask;
static unsigned receive_calls, receive_active, receive_failures;
static unsigned service_tc_immediately;
static uint16_t producer_pos;
static unsigned failures;

uint32_t HAL_GetTick(void) { return host_tick; }
uint32_t host_get_primask(void) { return host_primask; }
void host_disable_irq(void) { host_primask = 1; }
void host_set_primask(uint32_t value) { host_primask = value; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t len) {
    (void)buf;
    receive_calls++;
    if (receive_failures) { receive_failures--; return HAL_ERROR; }
    uart->hdmarx->remaining = len;
    uart->hdmarx->pending_tc = 0;
    producer_pos = 0;
    receive_active = 1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t l) {
    (void)u; (void)b; (void)l; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t l, uint32_t t) {
    (void)u; (void)b; (void)l; (void)t; return HAL_OK;
}
#define HOST_NOOP_UART(name) void name(UART_HandleTypeDef *u) { (void)u; }
HOST_NOOP_UART(cam_init)
HOST_NOOP_UART(hmi_init)
HOST_NOOP_UART(debug_uart_init)
HOST_NOOP_UART(Emm_V5_tx_callback)
HOST_NOOP_UART(debug_tx_callback)
HOST_NOOP_UART(cam_error_callback)
HOST_NOOP_UART(hwt101_error_callback)
HOST_NOOP_UART(debug_error_callback)
void my_hwt101_init(void) {}
void Emm_V5_Tx_Reset(void) {}
void debug_printf_write(const uint8_t *p, uint16_t n) { (void)p; (void)n; }
#define HOST_NOOP_RX(name) void name(UART_HandleTypeDef *u, uint16_t n) { (void)u; (void)n; }
HOST_NOOP_RX(cam_rx_callback)
HOST_NOOP_RX(hwt101_rx_callback)
HOST_NOOP_RX(hmi_rx_callback)

static void check(int condition, const char *name) {
    printf("%s %s\n", condition ? "PASS" : "FAIL", name);
    if (!condition) failures++;
}

static void reset(void) {
    memset(&host_dma, 0, sizeof(host_dma));
    memset(&huart4, 0, sizeof(huart4));
    huart4.Instance = UART4;
    huart4.hdmarx = &host_dma;
    host_tick = host_primask = 0;
    receive_calls = receive_active = receive_failures = 0;
    service_tc_immediately = 1;
    lidar_init(&huart4);
}

static void put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static uint16_t make_frame(uint8_t *out, uint8_t start, uint8_t count,
                           float first, float last, uint16_t distance) {
    uint16_t checksum, length = (uint16_t)(10 + count * 3);
    memset(out, 0, length);
    out[0] = 0xAA; out[1] = 0x55;
    out[2] = start; out[3] = count;
    put_u16(out + 4, (uint16_t)(((uint16_t)(first * 64.0f) << 1) | 1));
    put_u16(out + 6, (uint16_t)(((uint16_t)(last * 64.0f) << 1) | 1));
    for (uint8_t i = 0; i < count; i++) {
        out[10 + i * 3] = (uint8_t)(37U << 2);
        out[11 + i * 3] = (uint8_t)((distance & 63U) << 2);
        out[12 + i * 3] = (uint8_t)(distance >> 6);
    }
    checksum = (uint16_t)0x55AA ^ ((uint16_t)count << 8 | start);
    checksum ^= (uint16_t)(out[4] | out[5] << 8);
    checksum ^= (uint16_t)(out[6] | out[7] << 8);
    for (uint8_t i = 0; i < count; i++) {
        checksum ^= (uint16_t)(out[10 + i * 3] | out[11 + i * 3] << 8);
        checksum ^= out[12 + i * 3];
    }
    put_u16(out + 8, checksum);
    return length;
}

static void feed(const uint8_t *data, uint16_t count, int process) {
    if (!receive_active) { host_tick++; if (process) lidar_process(); return; }
    for (uint16_t i = 0; i < count; i++) {
        dma_rx_buf[producer_pos++] = data[i];
        if (producer_pos == DMA_BUF_LEN) {
            producer_pos = 0;
            host_dma.remaining = DMA_BUF_LEN;
            host_dma.pending_tc = 1;
            if (service_tc_immediately) {
                host_dma.pending_tc = 0; // 本地HAL先清TC标志，再调用RxCpltCallback
                HAL_UART_RxCpltCallback(&huart4);
            }
        } else host_dma.remaining = (uint16_t)(DMA_BUF_LEN - producer_pos);
    }
    host_tick++;
    if (process) lidar_process();
}

static void frame(uint8_t start, uint8_t count, float first, float last, uint16_t distance) {
    uint8_t out[10 + 255 * 3];
    uint16_t length = make_frame(out, start, count, first, last, distance);
    feed(out, length, 1);
}

static void test_packet_basics(void) {
    const lidar_scan_t *scan;
    reset();
    frame(1, 1, 0, 0, 1000);
    check(lidar_get_scan() == NULL, "first start collects without publishing");
    frame(0, 2, 359, 1, 700);
    frame(0, 0, 10, 10, 0);
    frame(0, 25, 100, 124, 900);
    frame(1, 1, 0, 0, 1100);
    scan = lidar_get_scan();
    check(scan && scan->raw_count == 28 && scan->valid_count == 28,
          "zero-sample packet safe and 25-sample frame accepted");
    check(scan && scan->distance[359] == 700 && scan->distance[1] == 700,
          "two-point 359 to 1 degree packet crosses zero correctly");
    check(scan && scan->intensity[100] == 37 && scan->distance[100] == 900,
          "D6 distance and intensity decoding");
    frame(0, 2, 20, 20.5f, 400);
    frame(0, 1, 20.25f, 20.25f, 300);
    frame(1, 1, 0, 0, 1000);
    scan = lidar_get_scan();
    check(scan && scan->raw_count == 4 && scan->valid_count == 2 && scan->distance[20] == 300,
          "same integer angle preserves nearest point within one scan");
}

static void test_checksum_zero(void) {
    uint8_t out[13];
    int found = 0;
    reset();
    for (unsigned d = 0; d < 16384 && !found; d++) {
        make_frame(out, 1, 1, 0, 0, (uint16_t)d);
        /* 对最后两字节距离/强度遍历，将合法 CS 构造到零。 */
        for (unsigned intensity = 0; intensity < 256; intensity++) {
            out[10] = (uint8_t)((intensity & 63) << 2);
            out[11] = (uint8_t)(((d & 63) << 2) | (intensity >> 6));
            if (lidar_xor_check(out, 1) == 0) { found = 1; break; }
        }
    }
    check(found, "construct legal zero-XOR packet");
    if (found) {
        put_u16(out + 8, 0);
        feed(out, 13, 1);
        check(lidar_get_xor_ok_cnt() == 1 && lidar_get_xor_err_cnt() == 0,
              "zero checksum is accepted by comparison with frame CS");
    }
}

static void test_corrupt_start(void) {
    uint8_t out[13];
    reset();
    frame(1, 1, 0, 0, 1000);
    frame(0, 1, 90, 90, 1000);
    make_frame(out, 1, 1, 0, 0, 500);
    out[8] ^= 1;
    feed(out, 13, 1);
    frame(0, 1, 90, 90, 500);
    frame(1, 1, 0, 0, 600);
    check(lidar_get_scan() == NULL, "corrupt start must not publish a scan spanning two revolutions");
    frame(0, 1, 90, 90, 600);
    frame(1, 1, 0, 0, 700);
    const lidar_scan_t *scan = lidar_get_scan();
    check(scan && scan->distance[0] == 600 && scan->distance[90] == 600 && scan->raw_count == 2,
          "valid start after corruption resumes clean scan");
}

static void test_bad_length(void) {
    reset();
    frame(1, 1, 0, 0, 1000);
    frame(0, 26, 10, 35, 800);
    frame(1, 1, 0, 0, 500);
    check(lidar_get_scan() == NULL, "unsupported 26-sample length discards interrupted scan safely");
    frame(0, 1, 90, 90, 500);
    frame(1, 1, 0, 0, 600);
    const lidar_scan_t *scan = lidar_get_scan();
    check(scan && scan->distance[90] == 500, "parser resynchronizes after oversized length");
}

static void test_error_recovery(void) {
    reset();
    frame(1, 1, 0, 0, 1000);
    /* 仿照本地 HAL 的 DMA 模式 UART 错误中止之后调用用户回调。 */
    receive_active = 0;
    huart4.RxState = 0;
    HAL_UART_ErrorCallback(&huart4);
    lidar_process();
    check(receive_active && receive_calls == 2, "UART4 error restarts DMA from main loop");
    frame(1, 1, 0, 0, 500);
    check(lidar_get_scan() == NULL, "recovery cannot publish the interrupted pre-error scan");
    frame(0, 1, 90, 90, 500);
    frame(1, 1, 0, 0, 600);
    const lidar_scan_t *scan = lidar_get_scan();
    check(scan && scan->distance[0] == 500 && scan->distance[90] == 500,
          "clean scan publishes after UART recovery");
}

static void test_observed_wrap(void) {
    uint8_t noise[1024];
    reset();
    frame(1, 1, 0, 0, 1000);
    memset(noise, 0, sizeof(noise));
    /* 显式服务每个真实模拟 TC 回调，不把 NDTR 同值当作溢出证据。 */
    feed(noise, sizeof(noise), 1);
    frame(1, 1, 0, 0, 500);
    check(lidar_get_scan() == NULL, "observed multiple DMA wraps discard interrupted scan");
    frame(0, 1, 90, 90, 500);
    frame(1, 1, 0, 0, 600);
    check(lidar_get_scan() && lidar_get_scan()->distance[90] == 500,
          "clean scan resumes after observed DMA overwrite");
}

static void test_pending_tc(void) {
    uint8_t noise[500], out[13];
    reset();
    memset(noise, 0, sizeof(noise));
    feed(noise, sizeof(noise), 1);
    service_tc_immediately = 0;
    make_frame(out, 1, 1, 0, 0, 500);
    feed(out, 13, 1); // DMA跨过512，但TC中断此时尚未进入
    check(lidar_get_dma_overrun_cnt() == 0,
          "pending TC plus NDTR avoids false overflow at normal wrap");
    host_dma.pending_tc = 0;
    HAL_UART_RxCpltCallback(&huart4);
    service_tc_immediately = 1;
    lidar_process();
    frame(0, 1, 90, 90, 500);
    frame(1, 1, 0, 0, 600);
    check(lidar_get_scan() && lidar_get_scan()->distance[90] == 500 &&
          lidar_get_dma_overrun_cnt() == 0,
          "servicing pending TC does not double-count production");
}

static void test_restart_failure(void) {
    reset();
    frame(1, 1, 0, 0, 1000);
    frame(1, 1, 0, 0, 500); // 建立一个可供显示的已发布扫描
    uint32_t old_number = lidar_get_scan()->scan_num;
    receive_active = 0;
    receive_failures = 1;
    HAL_UART_ErrorCallback(&huart4);
    lidar_process();
    check(!receive_active && receive_calls == 2 &&
          lidar_get_scan() && lidar_get_scan()->scan_num == old_number,
          "failed DMA restart preserves latest published scan for display");
    lidar_process();
    check(receive_active && receive_calls == 3 && lidar_get_rx_error_cnt() == 1,
          "failed DMA restart retries on following main-loop call");
    frame(1, 1, 0, 0, 700);
    frame(0, 1, 90, 90, 700);
    frame(1, 1, 0, 0, 800);
    check(lidar_get_scan() && lidar_get_scan()->scan_num > old_number &&
          lidar_get_scan()->distance[90] == 700,
          "scan numbering remains monotonic across reception recovery");
}

static void test_invalid_angles(void) {
    reset();
    frame(1, 1, 0, 0, 1000);
    frame(0, 1, 400, 400, 500);
    frame(1, 1, 0, 0, 600);
    check(lidar_get_scan() == NULL,
          "out-of-range but checksum-valid angle cannot alias to 40 degrees");
    frame(0, 1, 90, 90, 600);
    frame(1, 1, 0, 0, 700);
    check(lidar_get_scan() && lidar_get_scan()->distance[40] == 0,
          "invalid angle leaves no aliased ghost point in resumed scan");
}

int main(void) {
    test_packet_basics();
    test_checksum_zero();
    test_corrupt_start();
    test_bad_length();
    test_error_recovery();
    test_observed_wrap();
    test_pending_tc();
    test_restart_failure();
    test_invalid_angles();
    printf("LiDAR host: %u failures\n", failures);
    return failures ? 1 : 0;
}
