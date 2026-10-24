/* 不连接相机/电机，以合成串口帧和可控时钟验证实际camera.c。 */
#include "headfile.h"
#include <assert.h>

static uint32_t tick;
static unsigned tx_count;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart = {USART1, &dma};
extern uint16_t cam_rd;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)p; u->hdmarx->remaining = n; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *p, uint16_t n, uint32_t wait)
{ (void)u; (void)p; (void)n; (void)wait; tx_count++; return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u)
{ (void)u; return HAL_UART_RXEVENT_IDLE; }
void led_on(void) {}

static void start(uint8_t cmd)
{
    tick = 1000;
    cam_init(&uart);
    if (cmd == MAIX_CMD_BLOB) maix_start_blob(MAIX_COLOR_GREEN);
    else if (cmd == MAIX_CMD_RING) maix_start_ring(MAIX_RING_2);
    else maix_start_onup(MAIX_COLOR_GREEN);
}

static void feed(uint8_t cmd, uint8_t id, uint8_t ok, int16_t dx, int16_t dy)
{
    uint8_t frame[] = {HEAD1, HEAD2, cmd, 6, id, ok,
        (uint8_t)dx, (uint8_t)((uint16_t)dx >> 8),
        (uint8_t)dy, (uint8_t)((uint16_t)dy >> 8), 0};
    for (unsigned i = 2; i < 10; i++) frame[10] += frame[i];
    tick += 28;
    for (unsigned i = 0; i < sizeof(frame); i++) cam_byte_handle(frame[i]);
}

static void b2_found_only(void)
{
    start(MAIX_CMD_BLOB);
    for (uint8_t i = 1; i <= 5; i++) feed(MAIX_CMD_BLOB, i, 1, -12, 30);
    assert(maix_result.ok == 1 && maix_result.valid && !maix_result.stable);
    assert(maix_result.valid_count == 5 && maix_result.dx == -12 && maix_result.dy == 30);
}

static void b2_mixed_alignment(void)
{
    start(MAIX_CMD_BLOB);
    feed(MAIX_CMD_BLOB, 1, 2, 10, -10);
    feed(MAIX_CMD_BLOB, 2, 1, 200, 200);
    feed(MAIX_CMD_BLOB, 3, 2, 11, -11);
    feed(MAIX_CMD_BLOB, 4, 1, 201, 201);
    assert(maix_result.valid_count == 4 && maix_result.valid && !maix_result.stable);
    feed(MAIX_CMD_BLOB, 5, 2, 12, -12);
    assert(maix_result.ok == 2 && maix_result.stable && maix_result.valid_count == 5);
    assert(maix_result.dx == 11 && maix_result.dy == -11);
}

static void b2_span_and_eviction(void)
{
    start(MAIX_CMD_BLOB);
    feed(MAIX_CMD_BLOB, 1, 2, 0, 0);
    feed(MAIX_CMD_BLOB, 2, 2, 6, 6);
    feed(MAIX_CMD_BLOB, 3, 2, 12, 12);
    assert(maix_result.valid && !maix_result.stable);
    feed(MAIX_CMD_BLOB, 4, 2, 7, 7);
    feed(MAIX_CMD_BLOB, 5, 2, 8, 8);
    assert(maix_result.stable && maix_result.dx == 7);
    for (uint8_t i = 6; i <= 8; i++) feed(MAIX_CMD_BLOB, i, 1, 7, 7);
    assert(maix_result.valid && !maix_result.stable);
    feed(MAIX_CMD_BLOB, 9, 2, 7, 7);
    assert(!maix_result.stable); /* 旧OK=2已经滑出窗口，不能借OK=1凑稳定。 */
}

static void jitter_keeps_window(void)
{
    start(MAIX_CMD_BLOB);
    for (uint8_t i = 1; i <= 3; i++) feed(MAIX_CMD_BLOB, i, 2, 20, -5);
    assert(maix_result.stable);
    feed(MAIX_CMD_BLOB, 4, 1, 20, -5);
    assert(maix_result.ok == 1 && maix_result.valid && !maix_result.stable);
    feed(MAIX_CMD_BLOB, 5, 0, 123, -123);
    assert(!maix_vision.ok && maix_vision.dx == 0 && maix_vision.dy == 0);
    assert(maix_result.ok == 0 && maix_result.valid && !maix_result.stable);
    assert(maix_result.sample_count == 5 && maix_result.valid_count == 4);
    feed(MAIX_CMD_BLOB, 6, 2, 20, -5);
    assert(maix_result.stable && maix_result.valid_count == 4);
    for (uint8_t i = 7; i <= 11; i++) feed(MAIX_CMD_BLOB, i, 0, 0, 0);
    assert(!maix_result.valid && !maix_result.stable && maix_result.valid_count == 0);
    assert(maix_result.dx == 0 && maix_result.dy == 0);
}

static void b3_b4_and_status_validation(void)
{
    for (uint8_t cmd = MAIX_CMD_RING; cmd <= MAIX_CMD_ONUP; cmd++) {
        start(cmd);
        feed(cmd, 1, 1, 0, 0);
        feed(cmd, 2, 1, MAIX_STABLE_DX_SPAN, MAIX_STABLE_DY_SPAN);
        feed(cmd, 3, 1, 2, 2);
        assert(maix_result.ok == 1 && maix_result.valid && maix_result.stable);
        maix_vision.new_data = maix_result.new_data = 0;
        uint32_t saved_tick = maix_result.tick;
        feed(cmd, 4, 2, 2, 2);
        assert(!maix_vision.new_data && !maix_result.new_data && maix_result.tick == saved_tick);
        feed(cmd, 4, 0, 0, 0);
        assert(maix_result.valid && !maix_result.stable && maix_result.sample_count == 4);
    }
    start(MAIX_CMD_BLOB);
    feed(MAIX_CMD_BLOB, 1, 3, 0, 0);
    feed(MAIX_CMD_BLOB, 1, 255, 0, 0);
    assert(maix_result.sample_count == 0 && !maix_vision.new_data);
}

static void duplicates_and_timeout(void)
{
    start(MAIX_CMD_BLOB);
    for (uint8_t i = 1; i <= 3; i++) feed(MAIX_CMD_BLOB, i, 2, -300, 400);
    uint32_t saved_tick = maix_result.tick;
    maix_vision.new_data = maix_result.new_data = 0;
    feed(MAIX_CMD_BLOB, 3, 1, 0, 0);
    assert(!maix_vision.new_data && !maix_result.new_data);
    assert(maix_vision.ok == 2 && maix_vision.dx == -300 && maix_vision.tick == saved_tick);
    tick = saved_tick + MAIX_TIMEOUT_MS;
    maix_result_task();
    assert(maix_result.stable);
    tick++;
    maix_result_task();
    maix_vision_t raw;
    maix_result_t res;
    assert(maix_get_result(&raw) && !raw.ok && !raw.dx && !raw.dy);
    assert(maix_get_filtered_result(&res) && !res.ok && !res.valid && !res.stable);
    assert(res.sample_count == 0 && res.valid_count == 0 && !res.dx && !res.dy);
    assert(!maix_get_result(&raw) && !maix_get_filtered_result(&res));
    feed(MAIX_CMD_BLOB, 3, 2, -300, 400);
    assert(!maix_vision.new_data && maix_result.sample_count == 0);
    feed(MAIX_CMD_BLOB, 4, 2, -300, 400);
    assert(maix_result.sample_count == 1 && maix_result.ok == 2 && !maix_result.valid);
}

static void wrap_skip_and_gap(void)
{
    start(MAIX_CMD_BLOB);
    feed(MAIX_CMD_BLOB, 254, 2, 0, 0);
    feed(MAIX_CMD_BLOB, 255, 2, 0, 0);
    feed(MAIX_CMD_BLOB, 0, 2, 0, 0);
    feed(MAIX_CMD_BLOB, 7, 2, 0, 0);
    assert(maix_result.stable && maix_result.sample_count == 4);
    tick += MAIX_TIMEOUT_MS;
    feed(MAIX_CMD_BLOB, 8, 2, 0, 0);
    assert(maix_result.sample_count == 1 && !maix_result.valid && !maix_result.stable);
    start(MAIX_CMD_BLOB);
    tick = UINT32_MAX - 50;
    feed(MAIX_CMD_BLOB, 1, 2, 0, 0);
    tick += MAIX_TIMEOUT_MS + 1u;
    maix_result_task();
    assert(!maix_vision.ok && !maix_result.sample_count);
}

static void switch_stop_and_partial_frame(void)
{
    start(MAIX_CMD_BLOB);
    for (uint8_t i = 1; i <= 3; i++) feed(MAIX_CMD_BLOB, i, 2, 1, 1);
    cam_byte_handle(HEAD1); cam_byte_handle(HEAD2); cam_byte_handle(MAIX_CMD_BLOB);
    cam_byte_handle(6); /* 模拟切颜色时积压半帧和DMA字节。 */
    dma.remaining = CAM_LEN - 17;
    maix_start_blob(MAIX_COLOR_RED);
    assert(cam_rd == 17 && !maix_vision.ok && !maix_result.sample_count);
    feed(MAIX_CMD_BLOB, 3, 1, -32768, 32767);
    assert(maix_vision.target == MAIX_COLOR_RED && maix_vision.dx == -32768 && maix_vision.dy == 32767);
    maix_start_ring(MAIX_RING_2);
    feed(MAIX_CMD_BLOB, 4, 2, 0, 0);
    assert(maix_result.sample_count == 0);
    feed(MAIX_CMD_RING, 3, 1, 0, 0);
    unsigned sent = tx_count;
    maix_start_ring(MAIX_RING_1); maix_start_ring(MAIX_RING_3);
    assert(tx_count == sent && maix_vision.target == MAIX_RING_2 && maix_result.sample_count == 1);
    maix_start_onup(MAIX_COLOR_BLUE);
    assert(!maix_vision.ok && !maix_result.sample_count && maix_result.target == MAIX_COLOR_BLUE);
    feed(MAIX_CMD_ONUP, 3, 1, 0, 0);
    maix_stop();
    assert(!maix_vision.new_data && !maix_vision.ok && !maix_result.ok && !maix_result.sample_count);
    feed(MAIX_CMD_ONUP, 4, 1, 0, 0);
    assert(!maix_vision.new_data && !maix_result.sample_count);
}

static void malformed_frames(void)
{
    start(MAIX_CMD_BLOB);
    maix_frame_handle(MAIX_CMD_BLOB, NULL, 6);
    uint8_t payload[] = {1, 2, 0, 0, 0, 0};
    maix_frame_handle(MAIX_CMD_BLOB, payload, 5);
    uint8_t bad[] = {HEAD1, HEAD2, MAIX_CMD_BLOB, 6, 1, 2, 0, 0, 0, 0, 0};
    for (unsigned i = 0; i < sizeof(bad); i++) cam_byte_handle(bad[i]);
    assert(!maix_vision.new_data && maix_result.sample_count == 0);
    feed(MAIX_CMD_BLOB, 1, 2, -1, -2);
    assert(maix_vision.ok == 2 && maix_vision.dx == -1 && maix_vision.dy == -2);
}

int main(void)
{
    b2_found_only();
    b2_mixed_alignment();
    b2_span_and_eviction();
    jitter_keeps_window();
    b3_b4_and_status_validation();
    duplicates_and_timeout();
    wrap_skip_and_gap();
    switch_stop_and_partial_frame();
    malformed_frames();
    puts("PASS: 9 camera test groups (actual camera.c, simulated HAL/UART/time)");
    return 0;
}
