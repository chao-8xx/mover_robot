#include "headfile.h"

#define LED_BUZZER_PORT   GPIOE
#define LED_PIN           GPIO_PIN_4
#define BUZZER_PIN        GPIO_PIN_2

// 拓展板板载LED与蜂鸣器初始化
void light_buzzer_init(void)
{
    light_off();
    buzzer_off();
}

// 补光板亮灯
void light_on(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,LED_PIN,GPIO_PIN_SET);
}

// 补光板灭灯
void light_off(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,LED_PIN,GPIO_PIN_RESET);
}

// 补光板翻转
void light_toggle(void)
{
    HAL_GPIO_TogglePin(LED_BUZZER_PORT,LED_PIN);
}

// 开启补光板
void led_board_on(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

// 关闭补光板
void led_board_off(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}



// 蜂鸣器打开
void buzzer_on(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,BUZZER_PIN,GPIO_PIN_SET);
}

// 蜂鸣器关闭
void buzzer_off(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET);
}
