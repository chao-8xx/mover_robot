#include "headfile.h"

/**********************************************************
*** Emm_V5.0 闭环步进电机驱动器
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN专栏：https://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
**********************************************************/

// 一帧命令：data 存放命令字节，len 存放有效长度
// DMA 发送期间会一直读这个缓冲，所以必须保存在全局区，不能放在函数栈上
typedef struct
{
  uint8_t data[EMM_V5_TX_FRAME_MAX];
  uint8_t len;
} Emm_V5_TxFrame_t;

// 环形队列：head 指向“正在发送/下一帧要发送”的位置，tail 指向“下一个空位”
// count 表示队列里还有几帧没发完（包括正在 DMA 发送的那一帧）
static Emm_V5_TxFrame_t emm_v5_tx_queue[EMM_V5_TX_QUEUE_SIZE];
static volatile uint8_t emm_v5_tx_head = 0;
static volatile uint8_t emm_v5_tx_tail = 0;
static volatile uint8_t emm_v5_tx_count = 0;
// 串口DMA接收数据
#define EMM_V5_RX_BUF_SIZE   64
static uint8_t  emm_v5_rx_buf[EMM_V5_RX_BUF_SIZE];
static volatile int32_t emm_v5_pos[7];          // 按地址 1~6 存最新位置
static volatile uint8_t emm_v5_pos_valid[7];
static volatile int32_t emm_v5_sample_pos[7];  // 完整的带符号位置  65536单位/圈
static volatile uint32_t emm_v5_sample_tick[7];
static volatile uint8_t emm_v5_sample_valid[7];
// static uint8_t emm_v5_rx_frame[8];             // 保留跨两次DMA回调的半帧
// static uint32_t emm_v5_rx_tick = 0;
static uint8_t emm_v5_rx_count = 0;
static uint8_t  emm_v5_rx_ring[128];   // 累积收到的字节流
static uint16_t emm_v5_rx_ring_len = 0;

/**
  * @brief    串口2发送命令帧（非阻塞）
  * @note     命令先入队，由 DMA + USART2 中断逐帧发送；
  *           主循环和定时器中断中均可安全调用，不会忙等。
  */
static uint8_t Emm_V5_Send(const uint8_t *cmd, uint8_t len)
{
  uint8_t next_tail;
  uint32_t primask;

  // 非法长度直接丢弃，不参与入队
  if (len == 0 || len > EMM_V5_TX_FRAME_MAX)
  {
    return 0;
  }

  // 保存当前中断屏蔽状态，然后关中断，防止主循环和中断同时改队列导致错乱
  primask = __get_PRIMASK();
  __disable_irq();

  // 队列满了就不阻塞，直接丢弃这帧新命令
  if (emm_v5_tx_count >= EMM_V5_TX_QUEUE_SIZE)
  {
    __set_PRIMASK(primask);
    return 0;
  }

  // 把命令帧复制到队尾的空闲位置，DMA 后续会从这个全局缓冲读数据
  memcpy(emm_v5_tx_queue[emm_v5_tx_tail].data, cmd, len);
  emm_v5_tx_queue[emm_v5_tx_tail].len = len;

  // tail 向后移动一格（超过数组末尾就绕回 0，形成环形）
  next_tail = (uint8_t)((emm_v5_tx_tail + 1) % EMM_V5_TX_QUEUE_SIZE);
  emm_v5_tx_tail = next_tail;
  emm_v5_tx_count++;

  // count == 1 表示之前队列是空的，说明 DMA 当前空闲，需要主动启动第一帧
  if (emm_v5_tx_count == 1)
  {
    if (HAL_UART_Transmit_DMA(&huart2,
                              emm_v5_tx_queue[emm_v5_tx_head].data,
                              emm_v5_tx_queue[emm_v5_tx_head].len) != HAL_OK)
    {
      // 启动失败时清空队列，避免后续命令一直堆积
      emm_v5_tx_head = 0;
      emm_v5_tx_tail = 0;
      emm_v5_tx_count = 0;
      __set_PRIMASK(primask);
      return 0;
    }
  }

  // 恢复之前的中断屏蔽状态（之前没关就不开，之前关了仍保持关闭）
  __set_PRIMASK(primask);
  return 1;
}

/**
  * @brief    串口2发送速度/停止命令帧（非阻塞，同电机只保留最新一帧）
  * @note     扫描队列中还没开始发送的旧速度/停止命令并原地覆盖，
  *           避免连续控制时同电机命令堆积，也让 stop 能顶掉旧的运动命令
  */
static void Emm_V5_Send_Coalesce(const uint8_t *cmd, uint8_t len, uint8_t addr)
{
  uint8_t i;
  uint8_t idx;
  uint8_t scan_count;
  uint32_t primask;

  if (len == 0 || len > EMM_V5_TX_FRAME_MAX)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  // 跳过正在 DMA 发送的 head 帧，只扫描后面“等待发送”的帧
  scan_count = (emm_v5_tx_count > 0) ? (uint8_t)(emm_v5_tx_count - 1) : 0;
  idx = (uint8_t)((emm_v5_tx_head + 1) % EMM_V5_TX_QUEUE_SIZE);

  for (i = 0; i < scan_count; i++)
  {
    // 同电机且是旧的速度(0xF6)/停止(0xFE)命令，用最新命令覆盖它
    if (emm_v5_tx_queue[idx].data[0] == addr &&
        (emm_v5_tx_queue[idx].data[1] == 0xF6 ||
         emm_v5_tx_queue[idx].data[1] == 0xFE))
    {
      memcpy(emm_v5_tx_queue[idx].data, cmd, len);
      emm_v5_tx_queue[idx].len = len;
      __set_PRIMASK(primask);
      return;
    }
    idx = (uint8_t)((idx + 1) % EMM_V5_TX_QUEUE_SIZE);
  }

  // 没有可替换的旧命令，走普通入队逻辑
  if (emm_v5_tx_count >= EMM_V5_TX_QUEUE_SIZE)
  {
    __set_PRIMASK(primask);
    return;
  }

  memcpy(emm_v5_tx_queue[emm_v5_tx_tail].data, cmd, len);
  emm_v5_tx_queue[emm_v5_tx_tail].len = len;
  emm_v5_tx_tail = (uint8_t)((emm_v5_tx_tail + 1) % EMM_V5_TX_QUEUE_SIZE);
  emm_v5_tx_count++;

  if (emm_v5_tx_count == 1)
  {
    if (HAL_UART_Transmit_DMA(&huart2,
                              emm_v5_tx_queue[emm_v5_tx_head].data,
                              emm_v5_tx_queue[emm_v5_tx_head].len) != HAL_OK)
    {
      emm_v5_tx_head = 0;
      emm_v5_tx_tail = 0;
      emm_v5_tx_count = 0;
    }
  }

  __set_PRIMASK(primask);
}

/**
  * @brief    串口2发送完成回调，继续发送队列中的下一帧
  * @note     这一帧 DMA 数据搬完、最后一个字节也发出去后，
  *           HAL 会在 USART2 中断里调用本函数，不是在主循环轮询
  */
void Emm_V5_tx_callback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  // 只处理串口2，其它串口的发送完成回调不归这里管
  if (huart->Instance != USART2)
  {
    return;
  }

  // 同样关中断保护队列，防止和主循环/定时器里的入队操作冲突
  primask = __get_PRIMASK();
  __disable_irq();

  // count > 0 说明确实有帧刚发完，正常出队
  if (emm_v5_tx_count > 0)
  {
    // head 指向的那一帧已经发完，把 head 移到下一帧
    emm_v5_tx_head = (uint8_t)((emm_v5_tx_head + 1) % EMM_V5_TX_QUEUE_SIZE);
    emm_v5_tx_count--;

    // 出队后如果还有等待的帧，立刻启动下一帧 DMA 发送
    if (emm_v5_tx_count > 0)
    {
      if (HAL_UART_Transmit_DMA(&huart2,
                                emm_v5_tx_queue[emm_v5_tx_head].data,
                                emm_v5_tx_queue[emm_v5_tx_head].len) != HAL_OK)
      {
        // 启动失败就清空队列，避免后续命令一直堆积
        emm_v5_tx_head = 0;
        emm_v5_tx_tail = 0;
        emm_v5_tx_count = 0;
      }
    }
  }

  // 恢复中断屏蔽状态
  __set_PRIMASK(primask);
}

/**
  * @brief    复位串口2发送队列
  * @note     串口错误回调中调用，避免队列状态卡死
  */
void Emm_V5_Tx_Reset(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  emm_v5_tx_head = 0;
  emm_v5_tx_tail = 0;
  emm_v5_tx_count = 0;

  __set_PRIMASK(primask);
}

// 返回发送队列里还没发完的帧数（0 = 空闲）
uint8_t Emm_V5_Tx_Count(void)
{
  return emm_v5_tx_count;
}

// 电机串口接收DMA开启
void Emm_V5_Rx_Start(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, emm_v5_rx_buf, EMM_V5_RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);   // 关半满中断
}

// 电机串口接收回调
void Emm_V5_rx_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == NULL || huart->Instance != USART2 || Size > EMM_V5_RX_BUF_SIZE)
    return;

  // 累积本次收到的字节，留出余量防止越界
  for (uint16_t i = 0; i < Size && emm_v5_rx_ring_len < sizeof(emm_v5_rx_ring) - 8; i++) {
    emm_v5_rx_ring[emm_v5_rx_ring_len++] = emm_v5_rx_buf[i];
  }

  // 按 0x36 ... 0x6B 帧头帧尾扫描，ack 和噪声自动跳过
  uint16_t j = 0;
  while (j + 7 < emm_v5_rx_ring_len) {
    uint8_t addr = emm_v5_rx_ring[j];
    if (addr >= 1 && addr <= 6 &&
        emm_v5_rx_ring[j + 1] == 0x36 &&
        emm_v5_rx_ring[j + 7] == 0x6B) {

      uint32_t raw = ((uint32_t)emm_v5_rx_ring[j + 3] << 24) |
                     ((uint32_t)emm_v5_rx_ring[j + 4] << 16) |
                     ((uint32_t)emm_v5_rx_ring[j + 5] << 8)  |
                      (uint32_t)emm_v5_rx_ring[j + 6];

      uint32_t legacy = raw >> 8;
      if (legacy & 0x800000UL)
        legacy |= 0xFF000000UL;

      emm_v5_pos[addr] = (int32_t)legacy;
      emm_v5_pos_valid[addr] = 1;

      emm_v5_sample_pos[addr] = (int32_t)(emm_v5_rx_ring[j + 2] ? -(int64_t)raw : (int64_t)raw);
      emm_v5_sample_tick[addr] = HAL_GetTick();
      emm_v5_sample_valid[addr] = 1;

      j += 8;   // 消费掉这一整帧
        } else {
          j++;      // 不是帧头，跳一个字节
        }
  }

  // 清掉已消费部分，保留末尾不足 8 字节的半帧
  if (j > 0) {
    memmove(emm_v5_rx_ring, emm_v5_rx_ring + j, emm_v5_rx_ring_len - j);
    emm_v5_rx_ring_len -= j;
  }
}

// 上层拿位置（取走后清 valid）
uint8_t Emm_V5_Get_Pos(uint8_t addr, int32_t *pos)
{
  if (addr < 1 || addr > 6 || pos == NULL) return 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!emm_v5_pos_valid[addr]) {
    __set_PRIMASK(primask);
    return 0;
  }
  *pos = emm_v5_pos[addr];
  emm_v5_pos_valid[addr] = 0;
  __set_PRIMASK(primask);
  return 1;
}

// 原子取走位置和接收时间  避免读到新位置配旧时间
uint8_t Emm_V5_Get_Pos_Sample(uint8_t addr, int32_t *pos, uint32_t *tick)
{
  if (addr < 1 || addr > 6 || pos == NULL || tick == NULL) return 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!emm_v5_sample_valid[addr]) {
    __set_PRIMASK(primask);
    return 0;
  }
  *pos = emm_v5_sample_pos[addr];
  *tick = emm_v5_sample_tick[addr];
  emm_v5_sample_valid[addr] = 0;
  __set_PRIMASK(primask);
  return 1;
}

// 串口错误、初始化时调用  正常重开DMA时不能清除半帧
void Emm_V5_Rx_Reset(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  emm_v5_rx_count = 0;
  for (uint8_t i = 1; i <= 6; i++) {
    emm_v5_pos_valid[i] = 0;
    emm_v5_sample_valid[i] = 0;
  }
  __set_PRIMASK(primask);
}

/**
  * @brief    将当前位置清零
  * @param    addr  电机地址
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 命令码
  cmd[2] =  0x6D;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  电机地址
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 命令码
  cmd[2] =  0x52;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  电机地址
  * @param    s     系统参数枚举
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 命令码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  Emm_V5_Send(cmd, i);
}

/**
  * @brief    在线修改控制/闭环控制模式
  * @param    addr      电机地址
  * @param    svF       是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode 控制模式：0关闭所有使能输出，1开环模式，2闭环模式，
  *                     3 En端口负极为锁轴功能输出，Dir端口负极为锁轴方向高电平输出
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 命令码
  cmd[2] =  0x69;                       // 命令码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  ctrl_mode;                  // 控制模式
  cmd[5] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  电机地址
  * @param    state 使能状态，true为使能电机，false为关闭电机
  * @param    snF   同步运动标志，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 命令码
  cmd[2] =  0xAB;                       // 命令码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 同步运动标志
  cmd[5] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 6);
}

/**
  * @brief    清零编码器
  * @param    addr 电机地址
  * @param    0x0A  命令码
  * @param    0x6D  命令状态
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Clear_Encoder(uint8_t addr)
{
  uint8_t cmd[4] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 命令码
  cmd[2] =  0x6D;                       // 命令状态
  cmd[3] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 4);
}
/**
  * @brief    获取编码器
  * @param    addr 电机地址
  * @param    0x36  命令码
  * @retval   地址 + 命令码 + 校验字节
  */
void Emm_V5_Get_Encoder(uint8_t addr)
{
  uint8_t cmd[3] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x36;                       // 命令码
  cmd[2] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 3);
}

/**
  * @brief    速度模式控制
  * @param    addr 电机地址
  * @param    dir  方向，0为CW，其他值为CCW
  * @param    vel  速度，范围0 - 5000RPM
  * @param    acc  加速度，范围0 - 255，0为直接启动
  * @param    snF  同步运动标志，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 命令码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位
  cmd[5] =  acc;                        // 加速度
  cmd[6] =  snF;                        // 同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  Emm_V5_Send_Coalesce(cmd, 8, addr);
}

/**
  * @brief    位置模式控制
  * @param    addr 电机地址
  * @param    dir  方向，0为CW，其他值为CCW
  * @param    vel  速度(RPM)，范围0 - 5000RPM
  * @param    acc  加速度，范围0 - 255，0为直接启动
  * @param    clk  脉冲数，范围0 - (2^32 - 1)
  * @param    raF  绝对/相对标志，false为相对运动，true为绝对值运动
  * @param    snF  同步运动标志，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
uint8_t Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint16_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 命令码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位
  cmd[5]  =  acc;                       // 加速度
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 绝对/相对标志
  cmd[11] =  snF;                       // 同步运动标志
  cmd[12] =  0x6B;                      // 校验字节

  return Emm_V5_Send(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式通用）
  * @param    addr 电机地址
  * @param    snF  同步运动标志，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 命令码
  cmd[2] =  0x98;                       // 命令码
  cmd[3] =  snF;                        // 同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  Emm_V5_Send_Coalesce(cmd, 5, addr);
}

/**
  * @brief    触发同步启动运动
  * @param    addr 电机地址
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 命令码
  cmd[2] =  0x66;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 4);
}

/**
  * @brief    设置当前圈数零位位置
  * @param    addr 电机地址
  * @param    svF  是否存储标志，false为不存储，true为存储
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 命令码
  cmd[2] =  0x88;                       // 命令码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr   电机地址
  * @param    svF    是否存储标志，false为不存储，true为存储
  * @param    o_mode 回零模式，0为圆圈接近零点，1为圆圈绝对值零点，
  *                  2为圆圈限位碰撞零点，3为圆圈限位回零点
  * @param    o_dir  回零方向，0为CW，其他值为CCW
  * @param    o_vel  回零速度，单位RPM
  * @param    o_tm   回零超时时间，单位毫秒
  * @param    sl_vel 限位碰撞回零转速，单位RPM
  * @param    sl_ma  限位碰撞回零电流，单位Ma
  * @param    sl_ms  限位碰撞回零时间，单位Ms
  * @param    potF   上电自动回零，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 命令码
  cmd[2] =  0xAE;                       // 命令码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  o_mode;                     // 回零模式
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 限位碰撞回零转速(RPM)高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 限位碰撞回零转速(RPM)低8位
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 限位碰撞回零电流(Ma)高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 限位碰撞回零电流(Ma)低8位
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 限位碰撞回零时间(Ms)高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 限位碰撞回零时间(Ms)低8位
  cmd[18] =  potF;                      // 上电自动回零
  cmd[19] =  0x6B;                      // 校验字节

  Emm_V5_Send(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   电机地址
  * @param    o_mode 回零模式，0为圆圈接近零点，1为圆圈绝对值零点，
  *                  2为圆圈限位碰撞零点，3为圆圈限位回零点
  * @param    snF    同步运动标志，false为不启用，true为启用
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 命令码
  cmd[2] =  o_mode;                     // 回零模式
  cmd[3] =  snF;                        // 同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr 电机地址
  * @retval   地址 + 命令码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 命令码
  cmd[2] =  0x48;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  Emm_V5_Send(cmd, 4);
}
