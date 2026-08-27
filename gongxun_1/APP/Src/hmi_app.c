#include "hmi_app.h"


// 触摸串口屏事件
void hmi_touch(void) {
    // led_toggle();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}
