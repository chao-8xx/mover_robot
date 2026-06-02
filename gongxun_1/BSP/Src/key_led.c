//
// Created by mechreuo on 2026/6/2.
//

#include "headfile.h"


/////////////////////////////////////////// KEY 按键封装函数 /////////////////////////////////////////////

int key_scan(void){
    if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))
        return 1;
    // else if(!HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin))
    //     return 2;
    // else if(!HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin))
    //     return 3;
    // else if(!HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin))
    //     return 4;
    else
        return 0;
}


///////////////////////////////////////////// LED 封装函数 /////////////////////////////////////////////
// PC0 亮灯
void led_on(void) {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
}

// PC0 灭灯
void led_off(void) {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

// PC0 反转
void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
}