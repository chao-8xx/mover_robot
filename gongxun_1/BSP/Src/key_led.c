//
// Created by mechreuo on 2026/6/2.
//

#include "headfile.h"


/////////////////////////////////////////// KEY 按键封装函数 /////////////////////////////////////////////

uint8_t key_shortpress ;
uint8_t key_longpress ;

// 获取短按
uint8_t key_getshortpress(void)
{
	uint8_t temp ;
	if (key_shortpress)
	{
		temp = key_shortpress;
		key_shortpress = 0;
		return temp ;
	}
	return 0;
}

// 获取长按
uint8_t key_getlongpress(void)
{
	uint8_t temp ;
	if (key_longpress)
	{
		temp = key_longpress;
		key_longpress = 0;
		return temp;
	}
	return 0;
}

// 获取按键状态
uint8_t key_getstate(uint8_t index)
{
	switch (index)
	{
		case 0:			// KEY_NEXT
			return (HAL_GPIO_ReadPin (GPIOC ,GPIO_PIN_1 ) == GPIO_PIN_RESET) ? 1 : 0;
		// case 1:			// KET_LAST
		// 	return (HAL_GPIO_ReadPin (GPIOB ,GPIO_PIN_14 ) == GPIO_PIN_RESET) ? 2 : 0;
		// case 2:			// KEY_ENTER
		// 	return (HAL_GPIO_ReadPin (GPIOB ,GPIO_PIN_10 ) == GPIO_PIN_SET) ? 3 : 0;
		// case 3:			// KEY_BACK
		// 	return (HAL_GPIO_ReadPin (GPIOB ,GPIO_PIN_0  ) == GPIO_PIN_SET) ? 4 : 0;
		default :
			return 0;
	}
}

// 按键扫描函数，定时调用 现在在5ms中断中调用
void key_tick(void)
{
	static uint8_t count ;
	static uint8_t key_state[KEY_COUNT] = {0};
	static uint32_t press_time[KEY_COUNT] = {0};
	static uint8_t long_press_flag [KEY_COUNT] = {0};

	count ++;

	if(count >= 20)
	{
		count = 0;

		for (int i = 0; i < KEY_COUNT ; i++)
		{
			uint8_t keyvalue = key_getstate (i);

			if (keyvalue == i + 1)
			{
				if (key_state [i] == 0)
				{
					key_state [i] = 1;
					press_time [i] = 0;
					long_press_flag [i] = 0;
				}
				else
				{
					press_time [i] ++;
					if(press_time [i] >= 10 && !long_press_flag [i])
					{
						long_press_flag [i] = 1;
						key_longpress = i + 1;
					}
				}
			}
			else
			{
				if (key_state [i] == 1)
				{
					if (press_time [i] < 10)	// 1000ms
					{
						key_shortpress = i + 1;
					}
					key_state [i] = 0;
					press_time [i] = 0;
				}
			}
		}
	}
}

// 按键处理函数，在主循环中调用
void key_handle(void)
{
	uint8_t shortpress = key_getshortpress ();
	uint8_t longpress = key_getlongpress ();

	if (shortpress != 0 )
	{
		switch(shortpress )
		{
			case 1:
				led_on();
				// lidar_test();
				maix_blob_test();
				// openmv_qr_test();
				break ;

			case 2:
				maix_blob_test();
				break ;

				// OLED_Clear();
				// return flag;
				//
				// break ;

			// case 3:
			// 	led_getstate();	break ;
			// case 4:
			// 	change_state();	break ;
		}
	}
	if (longpress != 0 )
	{
		switch(longpress )
		{
			case 1:
				led_off();
				// lidar_stop();
		}
	}
}


///////////////////////////////////////////// LED 封装函数 /////////////////////////////////////////////
// led 初始化
void led_init(void) {
	led_off();
}

// PC0 亮灯
void led_on(void) {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
}

// PC0 灭灯
void led_off(void) {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

// PC0 翻转
void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
}
