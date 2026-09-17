//
// Created by mechreuo on 2026/6/2.
//

#ifndef GONGXUN_1_KEY_H
#define GONGXUN_1_KEY_H

#define KEY_COUNT 1     // 按键数量

uint8_t key_getshortpress(void);
uint8_t key_getlongpress(void);
uint8_t key_getstate(uint8_t index);

void key_tick(void);
void key_handle(void);


// void led_init(void);
// void led_on(void);
// void led_off(void);
// void led_toggle(void);

#endif //GONGXUN_1_KEY_H
