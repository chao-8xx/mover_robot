#include "hmi_app.h"

// 串口屏初始化
void my_hmi_init(void)
{
    hmi_init(&huart5);
}

// 触摸串口屏事件
void hmi_touch(void) {
    // led_toggle();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}

// 触摸串口屏发车事件
void hmi_start(void)
{
    car_run = true;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
}

// 触摸串口屏停车事件
void hmi_stop(void)
{
    car_run = false;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
}

// 触摸串口屏事件总处理
// 在hmi.c的hmi_rx_callback回调函数内调用
void hmi_solve(UART_HandleTypeDef *huart, uint16_t Size)
{
    hmi_touch_t touch;
    // 循环调用 hmi_GetTouch
    for (uint16_t i = 0; i < Size; i++) {
        touch = hmi_GetTouch(hmi_rxbuffer[i]);

        if (touch.is_valid && touch.event_type == 0x00) {   // 事件有效且按键弹起
            if ((touch.page_id == 0x01 && touch.obj_id == 0x06) ||
                (touch.page_id == 0x03 && touch.obj_id == 0x02) ||
                (touch.page_id == 0x04 && touch.obj_id == 0x02)) {
                hmi_stop(); // 停车事件触发
            }
            else if (touch.page_id == 0x01 && touch.obj_id == 0x04) {
                hmi_start();
                if (car_reset) {
                    car_reset = false;  // 置未复位标志，发车前复位一次
                }
            }
        }
    }
}
