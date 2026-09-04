#include "remind.h"

#define FLASH_TIME   22

uint16_t work_time = 0;         // 工作时间
uint8_t flash_times = 0;        // 闪光次数
bool work_state = false;        // 工作状态

Remind_State remindState;       // 提示装置状态
LED_Color led_color = LED_OFF;  // ws2812颜色状态机

// 提示装置初始化  包含ws2812、板载LED、板载蜂鸣器
void remind_init(void)
{
    ws2812_init();
    led_buzzer_init();
}

// 提示装置控制（非阻塞）
void remind_control(void)
{
    switch (remindState)
    {
        case RED_REMIND:
            led_color = LED_RED;
            break;
        case GREEN_REMIND:
            led_color = LED_GREEN;
            break;
        case BLUE_REMIND:
            led_color = LED_BLUE;
            break;
        case YELLOW_REMIND:
            led_color = LED_YELLOW;
            break;
        case PINK_REMIND:
            led_color = LED_PINK;
            break;
        case WHITE_REMIND:
            led_color = LED_WHITE;
            break;
        case NO_REMIND:
            led_color = LED_OFF;
            break;
        default:
            break;
    }

    work_time ++;
    if (led_color == LED_OFF) {     // 无提醒状态 红蓝闪烁
        if (work_time >= FLASH_TIME) {
            work_time = 0;
            work_state = !work_state;   // 每隔一段时间翻转一次状态
        }
        if (work_state) {
            ws2812_on(1,LED_BLUE);
            ws2812_on(2,LED_BLUE);
            ws2812_on(3,LED_BLUE);
            ws2812_on(4,LED_BLUE);
            ws2812_on(5,LED_RED);
            ws2812_on(6,LED_RED);
            ws2812_on(7,LED_RED);
            ws2812_show();
        }
        else {
            ws2812_on(7,LED_BLUE);
            ws2812_on(6,LED_BLUE);
            ws2812_on(5,LED_BLUE);
            ws2812_on(4,LED_RED);
            ws2812_on(3,LED_RED);
            ws2812_on(2,LED_RED);
            ws2812_on(1,LED_RED);
            ws2812_show();
        }
    }
    else {
        if (work_time >= FLASH_TIME / 2 + 1) {
            work_time = 0;
            work_state = !work_state;   // 每隔一段时间翻转一次状态
            if (work_state) {
                flash_times ++;
            }
        }
        if (work_state) {
            ws2812_all_on(led_color);
            buzzer_on();
        }
        else {
            ws2812_off();
            buzzer_off();
            if (flash_times > 2) {
                flash_times = 0;
                buzzer_off();
                remindState = NO_REMIND;
            }
        }
    }
}
