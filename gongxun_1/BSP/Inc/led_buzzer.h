#ifndef GONGXUN_1_LED_BUZZER_H
#define GONGXUN_1_LED_BUZZER_H

// 拓展板板载LED与蜂鸣器初始化
void light_buzzer_init(void);

// 补光板亮灯
void light_on(void);

// 补光板灭灯
void light_off(void);

// 补光板翻转
void light_toggle(void);

// 开启补光板
void led_board_on(void);

// 关闭补光板
void led_board_off(void);

// 蜂鸣器打开
void buzzer_on(void);

// 蜂鸣器关闭
void buzzer_off(void);

#endif //GONGXUN_1_LED_BUZZER_H
