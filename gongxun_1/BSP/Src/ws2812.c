#include "ws2812.h"

// CubeMX 生成的 TIM4_CH2 DMA 句柄（定义在 tim.c）
extern DMA_HandleTypeDef hdma_tim4_ch2;

// ============================================================
// 定时器参数（与 tim.c 里的 MX_TIM4_Init 保持一致）
// ============================================================
#define WS2812_TIMER_CLK     84000000UL  // TIM4 计数时钟 = 84MHz
#define WS2812_TIMER_PERIOD  104         // ARR，周期 = 105 计数 = 1.25us

// 每个 bit 的高电平时间（单位：计数）
//   0 码：高 0.4us  -> 0.4us / (1/84MHz) = 33.6 -> 34
//   1 码：高 0.8us  -> 0.8us / (1/84MHz) = 67.2 -> 68
#define WS2812_T0H           34
#define WS2812_T1H           68

// 数据长度：8 颗 x 24bit (G8 + R8 + B8)
#define WS2812_BITS_PER_LED  24
#define WS2812_BIT_COUNT     (WS2812_LED_COUNT * WS2812_BITS_PER_LED)

// 复位信号：数据线拉低 > 50us 才会锁存，这里等待 80 个周期 = 100us，留足余量
#define WS2812_RESET_PERIOD  80

// ============================================================
// 内部变量
// ============================================================
// 颜色帧缓冲：[index][0]=R, [1]=G, [2]=B
static uint8_t ws2812_rgb[WS2812_LED_COUNT][3];

// PWM 比较值缓冲：每一位一个值（0 码 = WS2812_T0H，1 码 = WS2812_T1H）
static uint16_t ws2812_pwm_buf[WS2812_BIT_COUNT];

// ============================================================
// 内部函数
// ============================================================
// 等待 TIM4 更新事件 n 次（每次 = 1.25us），用于复位延时
static void ws2812_wait_update(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        while (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) == RESET)
        {
        }
    }
}

// 把 RGB 帧缓冲转换成 PWM 比较值缓冲
// WS2812 数据顺序为 G -> R -> B，每个字节高位（MSB）在前
static void ws2812_fill_pwm_buffer(void)
{
    uint16_t *p = ws2812_pwm_buf;

    for (int led = 0; led < WS2812_LED_COUNT; led++)
    {
        const uint8_t grb[3] = {
            ws2812_rgb[led][1],   // G
            ws2812_rgb[led][0],   // R
            ws2812_rgb[led][2]    // B
        };

        for (int color = 0; color < 3; color++)
        {
            for (int bit = 7; bit >= 0; bit--)
            {
                *p++ = (grb[color] & (1u << bit)) ? WS2812_T1H : WS2812_T0H;
            }
        }
    }
}

// ============================================================
// 对外接口
// ============================================================
// 初始化：DMA 已由 CubeMX 配好，这里只需熄灭所有灯珠
void ws2812_init(void)
{
    // hdma_tim4_ch2 已在 MX_TIM4_Init() -> HAL_TIM_Base_MspInit() 里配置完成，
    // 这里无需再初始化 DMA，直接清空颜色缓冲
    ws2812_set_led(0, 255, 255, 255); // 第0个灯坏了，给白色，不用
    ws2812_clear();
    ws2812_show();
}

// 设置单个灯珠颜色
void ws2812_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT)
    {
        return;
    }

    ws2812_rgb[index][0] = r;
    ws2812_rgb[index][1] = g;
    ws2812_rgb[index][2] = b;
}

// 设置所有灯珠为同一颜色
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 1; i < WS2812_LED_COUNT; i++)
    {
        ws2812_set_led(i, r, g, b);
    }
}

// 关闭所有灯珠
void ws2812_clear(void)
{
    ws2812_set_all(0, 0, 0);
}

// 刷新显示：把帧缓冲发送到灯珠
void ws2812_show(void)
{
    // 1. 把 RGB 数据转换成 PWM 比较值
    ws2812_fill_pwm_buffer();

    // 2. 停止 PWM，预装载第一位的比较值并复位计数器，避免第一位被漏发
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ws2812_pwm_buf[0]);
    htim4.Instance->EGR = TIM_EGR_UG;   // 产生更新事件：预装载值生效 + CNT 清零
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    // 3. 启动 DMA，发送剩下的 (BIT_COUNT - 1) 位
    if (HAL_DMA_Start(&hdma_tim4_ch2,
                      (uint32_t)&ws2812_pwm_buf[1],
                      (uint32_t)&htim4.Instance->CCR2,
                      WS2812_BIT_COUNT - 1) != HAL_OK)
    {
        return;
    }
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);

    // 4. 启动 PWM 输出，数据开始发送
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    // 5. 等待 DMA 把所有位发送完（该函数同时会把 DMA 状态复位，供下次使用）
    if (HAL_DMA_PollForTransfer(&hdma_tim4_ch2, HAL_DMA_FULL_TRANSFER, 100) != HAL_OK)
    {
        // 超时或出错：关闭 DMA 请求并拉低数据线，避免灯珠数据错乱
        __HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC2);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        return;
    }

    // 6. 等待最后一个比较值被装载生效（一个更新事件）
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    while (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) == RESET)
    {
    }

    // 7. 关闭 DMA 请求，把数据线拉低，产生复位信号让灯珠锁存颜色
    __HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    ws2812_wait_update(WS2812_RESET_PERIOD);
}

// 单个LED控制
void ws2812_on(uint8_t index,LED_Color color)
{
    switch (color)
    {
        case LED_RED:
            ws2812_set_led(index,255,0,0);
            break;
        case LED_GREEN:
            ws2812_set_led(index,0,255,0);
            break;
        case LED_BLUE:
            ws2812_set_led(index,0,0,255);
            break;
        case LED_YELLOW:
            ws2812_set_led(index,255,255,0);
            break;
        case LED_PINK:
            ws2812_set_led(index,255,0,255);
            break;
        case LED_WHITE:
            ws2812_set_led(index,255,255,255);
            break;
        case LED_OFF:
            ws2812_set_led(index,0,0,0);
            break;
        default:
            ws2812_set_led(index,0,0,0);
            break;
    }
    // ws2812_show();
}

// ws2812统一启动
void ws2812_all_on(LED_Color color)
{
    switch (color)
    {
        case LED_RED:
            ws2812_set_all(255,0,0);
            break;
        case LED_GREEN:
            ws2812_set_all(0,255,0);
            break;
        case LED_BLUE:
            ws2812_set_all(0,0,255);
            break;
        case LED_YELLOW:
            ws2812_set_all(255,255,0);
            break;
        case LED_PINK:
            ws2812_set_all(255,0,255);
            break;
        case LED_WHITE:
            ws2812_set_all(255,255,255);
            break;
        case LED_OFF:
            ws2812_set_all(0,0,0);
            break;
        default:
            ws2812_set_all(0,0,0);
            break;
    }
    ws2812_show();
}

// ws2812全部熄灭
void ws2812_off(void)
{
    ws2812_clear();
    ws2812_show();
}