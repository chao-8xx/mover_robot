//
// Created by mechreuo on 2026/9/2.
//

#include "headfile.h"

#define POLL_MS         5
#define STOP_GUARD_MS   200
#define QR_TIMEOUT_MS   30000
#define MAIX_TEST_TIMEOUT_MS 15000
#define LOSS_TIMEOUT_MS 10000
#define MIN_FRAMES      20
#define EXTRA_MS        150

// 等待并处理串口数据
void service(void)
{
    if (cam_rx_flag) {
        cam_rx_flag = 0;
        cam_rx_handle();
    }
    maix_result_task();
}

// 持续服务指定时间（毫秒）
void service_for(uint32_t ms)
{
    uint32_t end = HAL_GetTick() + ms;
    while (HAL_GetTick() < end) {
        service();
        HAL_Delay(POLL_MS);
    }
}

// 停止所有视觉任务并清空残留数据
static void stop_and_clear(void)
{
    openmv_send_stop_scan();
    maix_stop();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();
}

// 等待二维码（超时30秒）
uint8_t wait_qr(void)
{
    uint32_t start = HAL_GetTick();
    qr_result_t qr;
    while ((HAL_GetTick() - start) < QR_TIMEOUT_MS) {
        service();
        if (openmv_get_qr_result(&qr)) {
            task_state.openmv_task_get_ok = 1;
            printf("[二维码] 成功，任务码=%s，长度=%u，延迟 %lu ms\r\n",
                   qr.qr_data,
                   (unsigned int)QR_DATA_LEN,
                   (unsigned long)(qr.qr_tick - start));
            return 1;
        }
        HAL_Delay(POLL_MS);
    }
    task_state.openmv_task_get_ok = 0;
    printf("[二维码] 超时（%lu ms）\r\n", (unsigned long)QR_TIMEOUT_MS);
    return 0;
}

// 测试单个Maix任务（通用）
static uint8_t test_maix_task(const char *name, uint8_t cmd, uint8_t target,
                              void (*start_fn)(uint8_t), uint8_t need_invalid_first)
{
    uint32_t start = HAL_GetTick();
    uint16_t unique = 0, dup = 0, ok_cnt = 0, invalid_zero = 0, invalid_nonzero = 0;
    uint8_t saw_valid = 0, saw_stable = 0, saw_invalid_before_ok = 0;
    uint8_t has_last = 0, last_id = 0;

    start_fn(target);
    printf("[%s] 启动，等待至少 %u 个独立帧...\n", name, MIN_FRAMES);

    while ((HAL_GetTick() - start) < MAIX_TEST_TIMEOUT_MS) {
        service();

        maix_vision_t raw;
        if (maix_get_result(&raw)) {
            // 检查命令和目标是否匹配
            if (raw.cmd != cmd || raw.target != target) {
                // 错误帧直接忽略，但算作失败条件（后面会检测）
                // 这里我们简单计数错误，最后判定
                // 为简化，我们直接标记失败，但为了信息完整，我们记录
                // 由于简化，我们只打印一次警告，并最终判失败
                static uint8_t warned = 0;
                if (!warned) {
                    printf("[%s] 警告：收到不匹配命令/目标，测试可能失败\n", name);
                    warned = 1;
                }
                continue;
            }

            // 去重
            if (has_last && raw.frame_id == last_id) {
                dup++;
                continue;
            }
            has_last = 1;
            last_id = raw.frame_id;
            unique++;

            // 统计 ok 和零偏移
            if (raw.ok) {
                ok_cnt++;
                // 如果之前没有ok，则检查是否先出现过invalid
            } else {
                if (raw.dx == 0 && raw.dy == 0) {
                    invalid_zero++;
                    if (ok_cnt == 0) saw_invalid_before_ok = 1;
                } else {
                    invalid_nonzero++;
                }
            }
            // 抽样打印，避免USART6逐帧阻塞影响USART1接收。
            if (unique <= 3 || (unique % 5) == 0) {
                printf("[%s RX] id=%u ok=%u dx=%d dy=%d\r\n",
                       name, raw.frame_id, raw.ok, raw.dx, raw.dy);
            }
        }

        maix_result_t res;
        if (maix_get_filtered_result(&res)) {
            uint8_t became_valid = res.valid && !saw_valid;
            uint8_t became_stable = res.stable && !saw_stable;
            if (res.valid) saw_valid = 1;
            if (res.stable) saw_stable = 1;
            if (became_valid || became_stable) {
                printf("[%s FILTER] samples=%u valid=%u stable=%u dx=%d dy=%d\r\n",
                       name, res.sample_count, res.valid, res.stable,
                       res.dx, res.dy);
            }
        }

        // 提前退出条件：足够独立帧且见到valid和stable，且满足“先无效”条件
        if (unique >= MIN_FRAMES && saw_valid && saw_stable &&
            (!need_invalid_first || saw_invalid_before_ok)) {
            break;
        }
        HAL_Delay(POLL_MS);
    }

    // 打印汇总
    printf("[%s] 汇总：独立帧=%u 重复=%u ok帧=%u 无效零=%u 无效非零=%u 有效=%u 稳定=%u 先无效=%u\n",
           name, unique, dup, ok_cnt, invalid_zero, invalid_nonzero,
           saw_valid, saw_stable, saw_invalid_before_ok);

    // 判定条件
    uint8_t pass = (unique >= MIN_FRAMES && saw_valid && saw_stable &&
                    invalid_nonzero == 0 &&
                    (!need_invalid_first || saw_invalid_before_ok));
    printf("[%s] %s\n", name, pass ? "通过" : "失败");
    return pass;
}

// 测试B3丢失（圆环移除）
static uint8_t test_b3_loss(void)
{
    uint32_t start = HAL_GetTick();
    uint8_t saw_zero_raw = 0, saw_filter_clear = 0, stale_coord = 0;

    printf("[B3丢失] 请移除圆环...\n");
    while ((HAL_GetTick() - start) < LOSS_TIMEOUT_MS) {
        service();

        maix_vision_t raw;
        if (maix_get_result(&raw) && !raw.ok) {
            if (raw.dx == 0 && raw.dy == 0)
                saw_zero_raw = 1;
            else
                stale_coord = 1;
        }

        maix_result_t res;
        if (maix_get_filtered_result(&res) && saw_zero_raw && !res.valid && !res.stable) {
            saw_filter_clear = 1;
        }

        if (saw_zero_raw && saw_filter_clear) break;
        HAL_Delay(POLL_MS);
    }

    uint8_t pass = saw_zero_raw && saw_filter_clear && !stale_coord;
    printf("[B3丢失] %s (零原始=%u 滤波清除=%u 残留偏移=%u)\n",
           pass ? "通过" : "失败", saw_zero_raw, saw_filter_clear, stale_coord);
    return pass;
}

// 测试停止后立即清除（B0）
static uint8_t test_b0_clear(void)
{
    maix_stop();
    uint8_t pass = (maix_vision.new_data == 0 && maix_result.valid == 0 &&
                    maix_result.stable == 0 && maix_result.sample_count == 0);
    printf("[B0清除] %s\n", pass ? "通过" : "失败");
    return pass;
}

// 测试超时复位
static uint8_t test_timeout(void)
{
    uint32_t start = HAL_GetTick();
    maix_start_onup(MAIX_COLOR_GREEN);
    // 等待至少一帧到达
    uint8_t got = 0;
    while ((HAL_GetTick() - start) < 3000) {
        service();
        if (maix_vision.new_data) { got = 1; break; }
        HAL_Delay(POLL_MS);
    }
    if (!got) {
        printf("[超时] 预条件失败：未收到任何帧\n");
        maix_stop();
        return 0;
    }
    // 暂停服务，等待超时
    printf("[超时] 暂停接收 %lu ms...\n", (unsigned long)(MAIX_TIMEOUT_MS + EXTRA_MS));
    cam_rx_flag = 0;
    HAL_Delay(MAIX_TIMEOUT_MS + EXTRA_MS);
    // DMA在暂停期间仍会继续接收Maix连续帧。这里必须先检查超时，
    // 不能先调用service()解析积压帧，否则新帧会刷新tick并掩盖超时。
    maix_result_task();
    maix_result_t res;
    uint8_t got_res = maix_get_filtered_result(&res);

    // 超时结果取出后再停止Maix并丢弃暂停期间的DMA积压数据。
    maix_stop();
    cam_empty_rx();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    uint8_t pass = got_res && res.sample_count == 0 && res.valid_count == 0 &&
                   !res.valid && !res.stable;
    printf("[超时] %s (收到结果=%u samples=%u valid=%u stable=%u)\n",
           pass ? "通过" : "失败", got_res, res.sample_count, res.valid, res.stable);
    return pass;
}

// OpenMV -> F407 单项测试：只发送B1并等待一帧BB/LEN=15
void openmv_test(void)
{
    openmv_end = 0;
    printf("\r\n========== OpenMV -> F407 单项测试 ==========""\r\n");

    // 停止扫码并清空USART1残留
    openmv_send_stop_scan();
    // printf("[OpenMV] 停止扫码并清空USART1残留\r\n");
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    // 开始扫码
    // printf("[OpenMV] 发送B1启动扫码，等待BB/LEN=%u\r\n",
    //        (unsigned int)QR_DATA_LEN);
    openmv_send_start_scan();
    uint8_t pass = wait_qr();

    openmv_send_stop_scan();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();
    led_off();

    printf("[OpenMV] 单项测试：%s\r\n", pass ? "通过" : "失败");
    printf("=============================================\r\n");
    openmv_end = 1;
}

// Maix B2绿色物料 -> F407 单项测试
void maix_b2_test(void)
{
    printf("\r\n========== Maix B2 -> F407 单项测试 =========="
           "\r\n");
    printf("[Maix B2] 请确保画面中有绿色物料\r\n");
    printf("[Maix B2] 发送B0并清空USART1残留\r\n");
    maix_stop();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    printf("[Maix B2] 发送B2/C4，等待至少%u个独立帧\r\n",
           MIN_FRAMES);
    uint8_t pass = test_maix_task(
        "B2", MAIX_CMD_BLOB, MAIX_COLOR_GREEN, maix_start_blob, 0
    );

    maix_stop();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();
    led_off();

    printf("[Maix B2] 单项测试：%s\r\n", pass ? "通过" : "失败");
    printf("=============================================\r\n");
}

// 摄像头综合测试函数
void cam_all_test(void)
{
    printf("\n========== 摄像头综合测试开始 ==========\n");
    printf("请准备：合法二维码、绿色物料、2号圆环、绿色码垛目标\n");

    printf("[步骤1] 停止并清空残留\n");
    stop_and_clear();

    printf("[步骤2] 二维码测试\n");
    openmv_send_start_scan();
    uint8_t qr_pass = wait_qr();
    openmv_send_stop_scan();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    printf("[步骤3] Maix B2 色块识别（绿色）\n");
    uint8_t b2_pass = test_maix_task("B2", MAIX_CMD_BLOB, MAIX_COLOR_GREEN, maix_start_blob, 0);
    maix_stop();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    printf("[步骤4] Maix B3 圆环识别（2号）\n");
    uint8_t b3_pass = test_maix_task("B3", MAIX_CMD_RING, MAIX_RING_2, maix_start_ring, 1);
    uint8_t b3_loss_pass = test_b3_loss();
    maix_stop();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    printf("[步骤5] Maix B4 码垛检测（绿色）\n");
    uint8_t b4_pass = test_maix_task("B4", MAIX_CMD_ONUP, MAIX_COLOR_GREEN, maix_start_onup, 0);

    printf("[步骤6] B0 清除测试\n");
    uint8_t b0_pass = test_b0_clear();
    service_for(STOP_GUARD_MS);
    cam_empty_rx();

    printf("[步骤7] 超时复位测试\n");
    uint8_t timeout_pass = test_timeout();

    led_off();
    printf("\n========== 测试汇总 ==========\n");
    printf("二维码: %s\n", qr_pass ? "通过" : "失败");
    printf("B2色块: %s\n", b2_pass ? "通过" : "失败");
    printf("B3圆环: %s\n", b3_pass ? "通过" : "失败");
    printf("B3丢失: %s\n", b3_loss_pass ? "通过" : "失败");
    printf("B4码垛: %s\n", b4_pass ? "通过" : "失败");
    printf("B0清除: %s\n", b0_pass ? "通过" : "失败");
    printf("超时复位: %s\n", timeout_pass ? "通过" : "失败");
    uint8_t final = qr_pass && b2_pass && b3_pass && b3_loss_pass && b4_pass && b0_pass && timeout_pass;
    printf("最终结果: %s\n", final ? "通过" : "失败");
    printf("=================================\n");
}

// // openmv扫码测试 key1 按下后开始扫码，扫码结果通过无线串口输出
// void openmv_qr_test(void)
// {
//     openmv_send_start_scan();
//     printf("OpenMV QR test start\r\n");
//
//     while (1)
//     {
//         // 本测试是阻塞循环，必须在这里继续处理USART1 DMA中的新字节。
//         if (cam_rx_flag)
//         {
//             cam_rx_flag = 0;
//             cam_rx_handle();
//         }
//         openmv_qr_task();
//
//         if (key_getlongpress() == 1)
//         {
//             openmv_send_stop_scan();
//             // led_off();
//             printf("OpenMV QR test stop\r\n");
//             break;
//         }
//
//         HAL_Delay(10);
//     }
// }

/*
// 之前的openmv测试代码
void openmv_qr_test_legacy(void) {
    // key
    // void key_handle(void)
    // {
    //   uint8_t shortpress = key_getshortpress ();
    //   uint8_t longpress = key_getlongpress ();
    //
    //   if (shortpress != 0 )
    //   {
    //     switch(shortpress )
    //     {
    //       case 1:
    //         led_off();
    //         openmv_send_start_scan();
    //         break ;
    //     }
    //   }
    //   if (longpress != 0 )
    //   {
    //     switch(longpress )
    //     {
    //       // case 1:
    //       //  led_tick ();
    //     }
    //   }
    // }

    //  主函数
    //   while (1)
    //   {
    //     key_handle();
    //     openmv_qr_task();
    //   }
}
*/

// // maixcam测试函数，按键1开始识别物料，按键1长按停止识别，识别结果通过虚拟串口CDC输出
// void maix_blob_test(void)
// {
//     uint8_t started = 0;
//     maix_vision_t vision;
//     maix_result_t result;
//
//     while (1)
//     {
//         if (!started)
//         {
//             led_on();
//             maix_start_blob(MAIX_COLOR_GREEN);
//             printf("Maix blob test start\r\n");
//             started = 1;
//         }
//
//         // 本测试是阻塞循环，不能依赖main()中的摄像头接收处理。
//         if (cam_rx_flag)
//         {
//             cam_rx_flag = 0;
//             cam_rx_handle();
//         }
//         maix_result_task();
//
//         if (maix_get_result(&vision))
//         {
//             printf("Maix result: cmd=%02X target=%02X frame=%u ok=%d dx=%d dy=%d tick=%lu\r\n",
//                    vision.cmd, vision.target, vision.frame_id, vision.ok,
//                    vision.dx, vision.dy, vision.tick);
//         }
//
//         if (maix_get_filtered_result(&result))
//         {
//             printf("Maix filter: frame=%u samples=%u ok=%u valid=%u stable=%u dx=%d dy=%d\r\n",
//                    result.frame_id, result.sample_count, result.valid_count,
//                    result.valid, result.stable, result.dx, result.dy);
//         }
//
//         if (key_getlongpress() == 1)
//         {
//             led_off();
//             maix_stop();
//             printf("Maix blob test stop\r\n");
//             break;
//         }
//
//         HAL_Delay(10);
//     }
// }
