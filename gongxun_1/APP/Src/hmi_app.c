//
// Created by mechreuo on 2026/8/18.
//

#include "../Inc/hmi_app.h"


// 触摸串口屏事件
void hmi_touch(void) {
    // led_toggle();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}
