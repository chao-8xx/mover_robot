#ifndef GONGXUN_1_WS2812_H
#define GONGXUN_1_WS2812_H

typedef enum {
    LED_RED    = 0,
    LED_GREEN  = 1,
    LED_BLUE   = 2,
    LED_YELLOW = 3,
    LED_PINK   = 4,
    LED_WHITE  = 5,
    LED_OFF    = 6
} LED_Color;

// 灯珠数量（8 颗）
#define WS2812_LED_COUNT 8

// ---- 函数声明 ---- //
// 初始化 WS2812（熄灭所有灯珠；DMA 由 CubeMX 在 MX_TIM4_Init 里配置）
// 注意：必须在 CubeMX 的 MX_TIM4_Init() 之后调用
void ws2812_init(void);

// 设置第 index 颗灯珠颜色
//   index : 0 ~ WS2812_LED_COUNT-1
//   r/g/b : 0 ~ 255
// 只更新缓冲区，不会立刻点亮，需要调用 ws2812_show() 才刷新到灯珠
void ws2812_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// 把所有灯珠设置成同一个颜色（只更新缓冲区，需要 ws2812_show() 刷新）
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);

// 关闭所有灯珠（全部设为黑色，需要 ws2812_show() 刷新）
void ws2812_clear(void);

// 把缓冲区数据发送到灯珠（真正刷新显示，会阻塞约 300us）
void ws2812_show(void);

// 单个LED控制
void ws2812_on(uint8_t index,LED_Color color);

// ws2812统一启动
void ws2812_all_on(LED_Color color);

// ws2812全部熄灭
void ws2812_off(void);

#endif //GONGXUN_1_WS2812_H
