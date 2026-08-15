/*=============================================================================
 * 文件名: hmi_tjc.c
 * 描  述: 淘晶驰串口屏通用驱动实现
 *=============================================================================*/
#include "headfile.h"

/* 屏幕指令缓存区大小：根据最长可能发送的字符串设定 */
#define HMI_TX_BUF_SIZE  128  

/* 指令结束符序列 */
static const uint8_t s_tjc_end_cmd[TJC_END_CMD_LEN] = {0xFF, 0xFF, 0xFF};

/**
 * @brief  向串口屏发送带格式化的指令 (核心协议封装)
 * @param  format: 格式化字符串 (类似 printf)
 * @param  ...   : 可变参数
 * 
 * @note   【设计思想】为什么用可变参数？
 *         屏幕指令如 "t0.txt=\"220V\""。用 sprintf 先格式化，再追加 0xFF 0xFF 0xFF 
 *         是最高效、最通用的方法。这一步封装好，上层调用就极其舒服。
 */
void HMI_SendCmd(const char *format, ...)
{
    char tx_buf[HMI_TX_BUF_SIZE];
    uint16_t len;
    va_list args; /* 定义可变参数列表 */
    
    /* 1. 将上层传入的格式化字符串，解析到 tx_buf 中 */
    va_start(args, format);
    vsnprintf(tx_buf, HMI_TX_BUF_SIZE, format, args);
    va_end(args);
    
    /* 2. 获取有效字符串长度 */
    len = strlen(tx_buf);
    
    /* 防御性编程：如果字符串太长，截断它，为结束符留出空间 */
    if(len > HMI_TX_BUF_SIZE - TJC_END_CMD_LEN) {
        len = HMI_TX_BUF_SIZE - TJC_END_CMD_LEN; 
    }
    
    /* 3. 在字符串尾部追加淘晶驰专属的3个0xFF结束符 */
    tx_buf[len]   = s_tjc_end_cmd[0];
    tx_buf[len+1] = s_tjc_end_cmd[1];
    tx_buf[len+2] = s_tjc_end_cmd[2];
    
    /* 4. 调用无硬件依赖的底层接口发送出去 (总长度 = 字符长度 + 3个结束符) */
    // BSP_UART_SendArray(COM_HMI_SCREEN, (uint8_t *)tx_buf, len + 3);
    HAL_UART_Transmit(&huart6,(uint8_t *)tx_buf,len + 3,100);
}

/**
 * @brief  设置文本控件的内容 (适老化UI常用)
 * @param  objName: 控件名称，例如 "t0" (电压显示文本框)
 * @param  text:    要显示的文本内容，例如 "220.5"
 */
void HMI_SetText(const char *objName, const char *text)
{
    /* 淘晶驰修改文本格式: 控件名.txt="内容" */
    HMI_SendCmd("%s.txt=\"%s\"", objName, text);
}

/**
 * @brief  设置数值控件的内容
 * @param  objName: 控件名称，例如 "n0"
 * @param  val:     整数值
 */
void HMI_SetValue(const char *objName, int32_t val)
{
    /* 淘晶驰修改数值格式: 控件名.val=数值 */
    HMI_SendCmd("%s.val=%d", objName, val);
}

/**
 * @brief  切换屏幕页面 (方便老人大按键跳转)
 * @param  pageName: 页面名称，例如 "page1"
 */
void HMI_ChangePage(const char *pageName)
{
    /* 淘晶驰切页格式: page 页面名 */
    HMI_SendCmd("page %s", pageName);
}
//
// /**
//  * @brief  非阻塞状态机解析屏幕回传的触摸事件
//  * @note   和 SU03T_GetCommand 同样的设计模式: 状态机逐字节匹配
//  */
// HMI_TouchEvent_t HMI_GetTouchEvent(void)
// {
//     static uint8_t state = 0;
//     static uint8_t rx_cache[4];  /* 缓存: 页码、控件ID、事件类型 */
//     static uint8_t ff_count = 0; /* 统计连续收到几个 0xFF */
//     uint8_t byte;
//
//     HMI_TouchEvent_t evt = {0, 0, 0, false}; /* 默认返回无效事件 */
//
//     while (BSP_UART_ReadByte(COM_HMI_SCREEN, &byte))
//     {
//         switch (state)
//         {
//             case 0: /* 等待帧头 0x65 */
//                 if (byte == 0x65) {
//                     state = 1;
//                 }
//                 break;
//
//             case 1: /* 接收页码 */
//                 rx_cache[0] = byte;
//                 state = 2;
//                 break;
//
//             case 2: /* 接收控件ID */
//                 rx_cache[1] = byte;
//                 state = 3;
//                 break;
//
//             case 3: /* 接收事件类型 */
//                 rx_cache[2] = byte;
//                 ff_count = 0;
//                 state = 4;
//                 break;
//
//             case 4: /* 等待3个连续的 0xFF 作为帧尾 */
//                 if (byte == 0xFF) {
//                     ff_count++;
//                     if (ff_count >= 3) {
//                         /* 完整一帧解析成功！ */
//                         evt.page_id    = rx_cache[0];
//                         evt.widget_id  = rx_cache[1];
//                         evt.event_type = rx_cache[2];
//                         evt.is_valid   = true;
//                         state = 0; /* 复位状态机 */
//                         return evt;
//                     }
//                 } else {
//                     state = 0; /* 帧尾不对, 丢弃重来 */
//                 }
//                 break;
//         }
//     }
//     return evt; /* 没有完整事件 */
// }