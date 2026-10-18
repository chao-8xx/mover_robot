#ifndef GONGXUN_1_LED_BUZZER_H
#define GONGXUN_1_LED_BUZZER_H

// 拓展板板载LED与蜂鸣器初始化
void led_buzzer_init(void);

// LED亮灯
void led_on(void);

// LED灭灯
void led_off(void);

// LED翻转
void led_toggle(void);

// 开启补光板
void led_board_on(void);

// 关闭补光板
void led_board_off(void);

// 蜂鸣器打开
void buzzer_on(void);

// 蜂鸣器关闭
void buzzer_off(void);

#endif //GONGXUN_1_LED_BUZZER_H
