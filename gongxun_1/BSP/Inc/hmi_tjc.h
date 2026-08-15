/*=============================================================================
 * 文件名: hmi_tjc.h
 * 描  述: 淘晶驰串口屏通用驱动
 * 架构层: 第2层 (驱动层)
 * 设计思想: 纯软件协议层。绝对不包含任何STM32特定的头文件。只依赖 bsp_uart.h。
 *=============================================================================*/
#ifndef __HMI_TJC_H
#define __HMI_TJC_H

#include "headfile.h"

/* 淘晶驰串口屏指令结束符: 连续3个 0xFF */
#define TJC_END_CMD_LEN   3
/* 屏幕触摸事件结构体 */
typedef struct {
    uint8_t  page_id;     /* 触摸发生在哪一页 */
    uint8_t  widget_id;   /* 被点击的控件编号 */
    uint8_t  event_type;  /* 事件类型: 0x01=按下, 0x00=弹起 */
    bool     is_valid;    /* 是否成功解析出一个有效事件 */
} HMI_TouchEvent_t;

/**
 * @brief  从屏幕串口缓冲区中解析触摸事件
 * @retval 解析出的触摸事件结构体
 *
 * 【协议格式】淘晶驰屏幕"发送键值"时, 固定发送7字节:
 *   0x65 [页码] [控件ID] [事件] 0xFF 0xFF 0xFF
 */
HMI_TouchEvent_t HMI_GetTouchEvent(void);
/*--- API 声明 ---*/
/* 格式化发送指令到底层 (核心函数) */
void HMI_SendCmd(const char *format, ...);

/* 适老化UI专属控制接口封装 (二次封装方便App层调用) */
void HMI_SetText(const char *objName, const char *text);  /* 设置文本控件内容 */
void HMI_SetValue(const char *objName, int32_t val);      /* 设置数字控件内容 */
// void HMI_ChangePage(const char *pageName);                /* 切换页面 */

#endif /* __HMI_TJC_H */
