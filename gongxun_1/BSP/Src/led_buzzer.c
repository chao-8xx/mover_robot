#include "led_buzzer.h"

#define LED_BUZZER_PORT   GPIOC
#define LED_PIN           GPIO_PIN_0
#define BUZZER_PIN        GPIO_PIN_2

// 拓展板板载LED与蜂鸣器初始化
void led_buzzer_init(void)
{
    led_off();
    buzzer_off();
}

// LED亮灯
void led_on(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,LED_PIN,GPIO_PIN_RESET);
}

// LED灭灯
void led_off(void)
{
    HAL_GPIO_WritePin(LED_BUZZER_PORT,LED_PIN,GPIO_PIN_SET);
}

// LED翻转
void led_toggle(void)
{
    led_on();
    HAL_Delay(500);
    led_off();
    HAL_Delay(500);
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
