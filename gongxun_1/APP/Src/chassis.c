#include "headfile.h"

// ==================== 机械参数（按实际底盘修改） ====================
// 轮径(cm) 和电机转一圈对应的脉冲数，决定“距离 -> 脉冲”的换算
#define CHASSIS_WHEEL_DIAM_CM        7.9f       // 麦轮直径
#define CHASSIS_MOTOR_PULSE_PER_REV  3200       // 多少脉冲一圈
#define CHASSIS_CM_PER_PULSE         ((3.14159265f * CHASSIS_WHEEL_DIAM_CM) / CHASSIS_MOTOR_PULSE_PER_REV)  // cm/脉冲
#define CHASSIS_CM_PER_FEEDBACK      ((3.14159265f * CHASSIS_WHEEL_DIAM_CM) / 65536.0f)  // 位置反馈与控制脉冲的单位不同
#define CHASSIS_YAW_SETTLE     50        // 50 个 10ms = 稳定 500ms 后才算转到位
#define CHASSIS_MOVE_MARGIN_MS 500       // 位置移动结束的额外余量ms
#define CHASSIS_ARRIVE_TOL_CM  0.5f      // 底盘到位容差 cm（编码器反馈提前判到位）

// 偏航 PID：输出是电机速度(RPM)，正负方向按实际转向调整
#define CHASSIS_YAW_KP         3.8f
#define CHASSIS_YAW_KD         0.2f
#define CHASSIS_YAW_MAX_I      50.0f
#define CHASSIS_YAW_MAX_OUT    120

// 转向电机死区速度
#define ROTATE_DEAD_SPEED  0
// 横移补偿：指令距离 / 实测距离，先标定再填
#define CHASSIS_STRAFE_K  1.10f     // 原1.21f

// 底盘四台电机速度（保留，便于查看）
int16_t vel[4] = {0};

// 全局位姿使用变量
static uint8_t odom_inited = 0;         // 是否已经完成第一次采样(第一次只记录基准，不算位移)
static float   odom_last_dist[4] = {0}; // 上一次采样时四个轮子的累计位移(cm)，用来算增量
float cur_wheel_dist[4] = {0.0f};       // 当前底盘四个电机的编码器状态 cm
static uint32_t odom_wheel_tick[4];    // 四轮各自完整反馈的接收时间
static uint8_t odom_new_mask = 0;      // 本轮已收到新反馈的轮子
static volatile uint32_t odom_epoch = 0; // 防止中断复位后主循环提交旧结果
static uint32_t odom_reset_tick = 0;
static uint32_t odom_pose_tick = 0;
static uint32_t odom_yaw_epoch = 0;
static uint8_t odom_origin_valid = 0;  // 已确认实际起点  暂停后必须重新设置
static float odom_yaw_offset = 0.0f;
static float odom_last_yaw = 0.0f;
#define ODOM_POLL_MS            20    // 每20ms读一个电机  一轮约80ms
#define ODOM_SAMPLE_TIMEOUT_MS  150   // 四轮采样最大年龄
#define ODOM_SAMPLE_SPAN_MS      80   // 四轮反馈最大时间差
#define ODOM_RESET_WAIT_MS      100   // 编码器清零后先丢弃在途旧反馈

// 底盘运动状态
Chassis_Data my_chassis;

static volatile Chassis_Mode chassis_mode = CHASSIS_IDLE;
static volatile float chassis_target_yaw = 0.0f;
static volatile uint8_t chassis_settle = 0;     // 角度稳定帧数
static volatile uint32_t chassis_deadline = 0;
static uint8_t chassis_inited = 0;

// 分时发送状态：一次移动 = 4 条电机命令
static int32_t chassis_wheel_cmd[4];     // 位置模式=脉冲，速度模式=速度
static uint8_t chassis_send_mode = 0;    // 0=速度模式，1=位置模式
static int16_t chassis_move_speed = 0;   // 位置模式使用
static uint8_t chassis_send_index = 0;   // 下一个要发送的电机下标
static uint8_t chassis_sending = 0;      // 1 = 还在逐条发送命令
static float chassis_start_dist[4] = {0};   // 每轮起点累计位移 cm
static float chassis_target_dist[4] = {0};  // 每轮目标增量 cm

// ======== 初始化 ======== //
// float 绝对值
static float my_absf(float data)
{
    return data < 0.0f ? -data : data;
}
// 底盘初始化：配置偏航 PID
void chassis_init(void)
{
    Positional_PID_Init(&yaw_pid,
                        CHASSIS_YAW_KP,
                        0.0f,
                        CHASSIS_YAW_KD,
                        CHASSIS_YAW_MAX_I,
                        -CHASSIS_YAW_MAX_OUT,
                        CHASSIS_YAW_MAX_OUT);
    PID_Reset(&yaw_pid);
    chassis_mode = CHASSIS_IDLE;
    chassis_inited = 1;

    chassis_odom_hold();    // 未发车 关闭全局位姿解算
}

// ======== 底盘麦轮解算 ======== //
// 清空底盘（停车）
static void chassis_clear(void)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = 0;
}
// 麦克纳姆轮运动学解算
// wheel[i] 的正负通过 motor_set_speed 转换成方向，绝对值是速度
static void chassis_solve(void)
{
    int wheel[4];   // 四个轮子的速度（正负表示方向）

    // 起点轮子基准没建立就动了  即使之后停车也不能把当前位置当起点
    if (!odom_inited && (my_chassis.x || my_chassis.y || my_chassis.omega))
        odom_origin_valid = 0;

    wheel[0] = my_chassis.y + my_chassis.x - my_chassis.omega;
    wheel[1] = -my_chassis.y + my_chassis.x - my_chassis.omega;
    wheel[2] = my_chassis.y - my_chassis.x - my_chassis.omega;
    wheel[3] = -my_chassis.y - my_chassis.x - my_chassis.omega;

    for (int i = 0; i < 4; i++) {
        vel[i] = (int16_t)wheel[i];
        if (wheel[i] > 0) {
            chassis_wheel_cmd[i] = wheel[i] + ROTATE_DEAD_SPEED;
        }
        else if (wheel[i] < 0) {
            chassis_wheel_cmd[i] = wheel[i] - ROTATE_DEAD_SPEED;
        }
        else {
            chassis_wheel_cmd[i] = wheel[i];
        }
    }
    chassis_send_mode = 0;     // 速度模式
    // 只有当前没在发送时才重置序列 让car_move_send_tick把 4 个轮子依次发完
    if (!chassis_sending) {
        chassis_send_index = 0;
        chassis_sending = 1;
    }
}
// 停车
void car_stop(void)
{
    chassis_clear();
    chassis_solve();
    chassis_send_index = 0; // 强制从 0 号轮重新发送四轮停止命令，避免旧速度命令残留
    chassis_sending = 1;
    chassis_mode = CHASSIS_IDLE;
}

// ======== 底盘移动旋转控制 ======== //
// 速度模式：小车平移和旋转
void car_speed_move (int16_t x_speed, int16_t y_speed, int16_t omega_speed)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;
    }

    my_chassis.x = x_speed;
    my_chassis.y = y_speed;
    my_chassis.omega = omega_speed;
    chassis_solve();
}

// 小车旋转 只旋转不位移
static void chassis_rotate(int16_t omega)
{
    my_chassis.x = 0;
    my_chassis.y = 0;
    my_chassis.omega = omega;
}

// 原地旋转
void car_rotate(int16_t omega)
{
    chassis_rotate(omega);
    chassis_solve();
}

// 一次调用：设置目标角度（复位角度版）
// 非阻塞：只设置目标，实际由 car_rotate_ctrl() 每 10ms 驱动闭环
void car_rotate_angle(float target_angle)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙（旋转/移动中）时不接受新目标，避免互相覆盖
    }

    if (!chassis_inited) {
        chassis_init();
    }

    chassis_target_yaw = target_angle;
    chassis_settle = 0;
    PID_Reset(&yaw_pid);
    chassis_mode = CHASSIS_ROTATE;
}
// 10ms 周期调用（放 TIM2 中断或主循环）：
// 用偏航 PID 闭环转到目标角度，到位后调用 hwt101_yaw_reset() 复位偏航角，用于转完后复位
void car_rotate_ctrl(void)
{
    // 未初始化则初始化
    if (!chassis_inited) {
        chassis_init();
    }

    // 不处于旋转状态，不执行
    if (chassis_mode != CHASSIS_ROTATE) {
        return;
    }

    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    float err = chassis_target_yaw - now_yaw;   // 不归一化，直接比较

    if (err > YAW_THRESH || err < -YAW_THRESH) {
        chassis_settle = 0;
        Positional_PID_Cal(&yaw_pid, chassis_target_yaw, now_yaw);
        car_rotate((int16_t)yaw_pid.output);
    }
    else {
        // 误差小：先停住，连续稳定 50 帧（500ms）才算到位
        car_rotate(0);
        chassis_settle++;
        if (chassis_settle >= CHASSIS_YAW_SETTLE) {
            car_stop();
            hwt101_yaw_reset();         // 到位后把偏航角复位为 0
            chassis_target_yaw = 0.0f;  // 底盘目标角度也复位
        }
    }
}

// 小车旋转目标角度（不复位角度版）
void car_rotate_yaw(float target_yaw)
{
    // 未初始化则初始化
    if (!chassis_inited) {
        chassis_init();
    }

    // 陀螺仪没数据就不执行
    if (!hwt101_data_valid()) {
        return;
    }

    // 底盘状态为旋转
    chassis_mode = CHASSIS_ROTATE;

    float err = target_yaw - my_yaw;   // 不归一化，直接比较

    if (err > YAW_THRESH || err < -YAW_THRESH) {
        chassis_settle = 0;     // 等待时间清零
        Positional_PID_Cal(&yaw_pid, target_yaw, my_yaw);
        car_rotate((int16_t)(yaw_pid.output));
    }
    else {
        // 误差小：先停住，连续稳定 100 帧（1000ms）才算到位
        car_rotate(0);   // 只发 0 速度，保持 CHASSIS_ROTATE，直到稳定计数满
        chassis_settle++;
        if (chassis_settle >= CHASSIS_YAW_SETTLE) {
            car_stop();
            chassis_mode = CHASSIS_IDLE;    // 底盘空闲
        }
    }
}

// 位置模式：沿 (x_cm, y_cm) 走一段相对距离
static void car_move_xy(int16_t speed, float x_cm, float y_cm)
{
    int32_t px, py;
    int32_t w[4];
    float max_cm, speed_cm_s;
    static float acc = (float)STEP_MOTOR_CHASSIS_ACC;
    uint32_t t_ms = 0;

    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }

    if (!chassis_inited) {
        chassis_init();
    }

    // 距离 -> 脉冲数（四舍五入）
    px = (int32_t)((x_cm * CHASSIS_STRAFE_K) / CHASSIS_CM_PER_PULSE + (x_cm >= 0.0f ? 0.5f : -0.5f));
    py = (int32_t)(y_cm / CHASSIS_CM_PER_PULSE + (y_cm >= 0.0f ? 0.5f : -0.5f));
    if (!odom_inited && (px || py))
        odom_origin_valid = 0;

    // 麦克纳姆轮运动学
    w[0] =  py + px;
    w[1] = -py + px;
    w[2] =  py - px;
    w[3] = -py - px;

    // 不立即发送：先存下 4 个轮子的脉冲数，
    // 由 car_move_send_tick() 在 1ms 里分时逐条发送，避免驱动器丢帧
    for (int i = 0; i < 4; i++) {
        if (w[i] > 2147483647) {
            w[i] = 2147483647;
        }
        if (w[i] < -2147483647) {
            w[i] = -2147483647;
        }
        chassis_wheel_cmd[i] = w[i];
    }

    // 记录每轮起点和目标增量，供编码器反馈提前判到位
    for (int i = 0; i < 4; i++) {
        chassis_start_dist[i] = cur_wheel_dist[i];
        chassis_target_dist[i] = (float)chassis_wheel_cmd[i] * CHASSIS_CM_PER_PULSE;
    }
    chassis_send_mode = 1;     // 位置模式
    chassis_move_speed = speed;
    chassis_send_index = 0;
    chassis_sending = 1;

    // 按最长轴估算移动耗时，作为 busy 结束时间
    max_cm = my_absf(x_cm) > my_absf(y_cm) ? my_absf(x_cm) : my_absf(y_cm);
    speed_cm_s = 3.14159265f * CHASSIS_WHEEL_DIAM_CM * my_absf((float)speed) / 60.0f;
    if (speed_cm_s < 0.1f) {
        speed_cm_s = 0.1f;
    }

    if (acc < 1.0f) {
        t_ms = (uint32_t)(max_cm / speed_cm_s * 1000.0f);
    }
    else {
        float t_ramp_s = my_absf((float)speed)
                     * (256.0f - (float)STEP_MOTOR_CHASSIS_ACC) * 50e-6f; // ZDT官方资料，曲线加减速时间计算公式
        t_ms = (uint32_t)(((max_cm / speed_cm_s) + t_ramp_s) * 1000.0f);  // 梯形公式
    }

    chassis_deadline = HAL_GetTick()
                       + (uint32_t)(t_ms)
                       + CHASSIS_MOVE_MARGIN_MS;
    chassis_mode = CHASSIS_MOVE;
}

// 小车沿 X/Y 轴走固定距离（cm）
void car_run_distance(int16_t speed, float x_cm, float y_cm)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }
    car_move_xy(speed, x_cm, y_cm);
}

// 底盘四个电机回到编码器零点位置
void car_back_zero(void)
{
    if (chassis_mode != CHASSIS_IDLE) {
        return;   // 底盘忙时不接受新移动指令
    }

    if (!odom_inited)
        odom_origin_valid = 0;
    chassis_send_mode = 2;     // 回零模式
    chassis_send_index = 0;
    chassis_sending = 1;
}

// ======== 底盘状态 ======== //
// 是否还在执行旋转/移动
uint8_t car_is_busy(void)
{
    return chassis_mode != CHASSIS_IDLE;
}
// 获取底盘状态
Chassis_Mode car_get_state(void)
{
    return chassis_mode;
}

// ======== 全局位姿解析 ======== //
// 暂停底盘航迹推算（跟随模式下调用）
void chassis_odom_hold(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    chassis_odom_reset();
    odom_origin_valid = 0;     // 暂停期间的位置未知  不能延时后沿用旧坐标
    task_state.odom_on = 0;
    __set_PRIMASK(primask);
}
// 车辆静止在已知位置时调用  坐标使用现有cm单位  不会下发移动命令
uint8_t chassis_odom_origin_set(float x_cm, float y_cm)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    chassis_odom_hold();
    if (car_is_busy() || my_chassis.x || my_chassis.y || my_chassis.omega ||
        !isfinite(x_cm) || !isfinite(y_cm) || x_cm < 0.0f || y_cm < 0.0f ||
        x_cm >= MAP_WIDTH / 10.0f || y_cm >= MAP_HEIGHT / 10.0f) {
        __set_PRIMASK(primask);
        return 0;
    }
    task_state.x_cm = x_cm;
    task_state.y_cm = y_cm;
    odom_yaw_epoch = hwt101.yaw_epoch;
    odom_yaw_offset = map_pose_yaw_offset_get();
    odom_origin_valid = 1;
    task_state.odom_on = 1;
    __set_PRIMASK(primask);
    return 1;
}
// 主循环调用：轮流给四个电机发读位置请求  静止、旋转时也要采样
void chassis_odom_poll(void)
{
    static uint8_t idx = 0;
    static uint32_t last_tick = 0;

    // 位姿解算被暂停时不发读请求，避免干扰升降/伸缩的反馈
    if (!task_state.odom_on) {
        return;
    }

    // 等当前发送完成再读  保留底盘原有分时发送  不向串口队列连续塞请求
    if (Emm_V5_Tx_Count() != 0) {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - last_tick) < ODOM_POLL_MS ||
        (uint32_t)(HAL_GetTick() - odom_reset_tick) < ODOM_RESET_WAIT_MS) return;
    last_tick = HAL_GetTick();

    Emm_V5_Read_Sys_Params(idx + 1, S_CPOS);   // 地址 1~4
    idx = (idx + 1) & 0x03;
}
// 主循环调用：取完整反馈及其接收时间  四轮独立记录是否收到新数据
void chassis_odom_fetch(void)
{
    uint32_t epoch = odom_epoch;
    for (int i = 0; i < 4; i++) {
        int32_t pos;
        uint32_t tick;
        if (Emm_V5_Get_Pos_Sample(i + 1, &pos, &tick)) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (epoch == odom_epoch &&
                (uint32_t)(tick - odom_reset_tick) >= ODOM_RESET_WAIT_MS &&
                (uint32_t)(tick - odom_reset_tick) <= (uint32_t)(HAL_GetTick() - odom_reset_tick) &&
                (uint32_t)(HAL_GetTick() - tick) <= ODOM_SAMPLE_TIMEOUT_MS) {
                cur_wheel_dist[i] = (float)pos * CHASSIS_CM_PER_FEEDBACK;
                odom_wheel_tick[i] = tick;
                odom_new_mask |= (uint8_t)(1U << i);
            }
            __set_PRIMASK(primask);
        }
    }
}
// 主循环调用：四轮都有新反馈才积分  不用重复数据刷新地图位姿时间
void chassis_odom_update(void)
{
    float dist[4], d[4], x_cm, y_cm, yaw, last_yaw;
    uint32_t now = HAL_GetTick();
    uint32_t oldest_age = 0, newest_age = UINT32_MAX;
    uint32_t epoch, pose_tick;
    uint8_t inited;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 归零、暂停、反馈中断后不再认为旧坐标可靠  必须重新确认已知起点
    if (!task_state.odom_on || !hwt101_yaw_valid() || hwt101.yaw_epoch != odom_yaw_epoch ||
        map_pose_yaw_offset_get() != odom_yaw_offset ||
        (odom_inited && (uint32_t)(now - odom_pose_tick) > MAP_POSE_TIMEOUT_MS))
        odom_origin_valid = 0;

    if (!odom_origin_valid) {
        map_pose_update(0.0f, 0.0f, 0.0f, now, 0);
        __set_PRIMASK(primask);
        return;
    }

    if (odom_new_mask != 0x0F) {
        if (!odom_inited)
            map_pose_update(0.0f, 0.0f, 0.0f, now, 0);
        __set_PRIMASK(primask);
        return;
    }
    for (int i = 0; i < 4; i++) {
        uint32_t age = now - odom_wheel_tick[i];
        if (age > oldest_age) oldest_age = age;
        if (age < newest_age) newest_age = age;
        dist[i] = cur_wheel_dist[i];
        d[i] = dist[i] - odom_last_dist[i];
    }
    odom_new_mask = 0;
    if (oldest_age > ODOM_SAMPLE_TIMEOUT_MS || oldest_age - newest_age > ODOM_SAMPLE_SPAN_MS) {
        __set_PRIMASK(primask);
        return;
    }
    // 位姿时间取四轮和角度中最旧的测量时间  不用主循环时间冒充测量时间
    if ((uint32_t)(now - hwt101.angle_tick) > oldest_age)
        oldest_age = now - hwt101.angle_tick;
    pose_tick = now - oldest_age;
    epoch = odom_epoch;
    inited = odom_inited;
    x_cm = task_state.x_cm;
    y_cm = task_state.y_cm;
    yaw = my_yaw;
    last_yaw = odom_last_yaw;
    if (!inited && (car_is_busy() || my_chassis.x || my_chassis.y || my_chassis.omega)) {
        odom_origin_valid = 0;  // 没记录到起点的轮子基准就开始移动  不能补猜这段位移
        map_pose_update(0.0f, 0.0f, 0.0f, now, 0);
        __set_PRIMASK(primask);
        return;
    }
    __set_PRIMASK(primask);

    // 正运动学：轮子增量 → 车体位移
    // 逆运动学是 w0=y+x-ω, w1=-y+x-ω, w2=y-x-ω, w3=-y-x-ω
    float car_dy = ( d[0] - d[1] + d[2] - d[3]) / 4.0f;
    float car_dx = ((d[0] + d[1] - d[2] - d[3]) / 4.0f) / CHASSIS_STRAFE_K;

    // 与车头朝向约定一致：车头 (1,0)=东，my_yaw=0 朝东、+90 朝北、-90 朝南
    // 区间中间朝向近似旋转期间的位移方向  第一次采样只记录轮子基准
    if (inited) {
        float rad = ((yaw + last_yaw) * 0.5f + odom_yaw_offset) * 3.14159265f / 180.0f;
        x_cm += car_dy * cosf(rad) + car_dx * sinf(rad);
        y_cm += car_dy * sinf(rad) - car_dx * cosf(rad);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (epoch == odom_epoch && task_state.odom_on && odom_origin_valid &&
        hwt101_yaw_valid() && hwt101.yaw_epoch == odom_yaw_epoch) {
        for (int i = 0; i < 4; i++) odom_last_dist[i] = dist[i];
        odom_last_yaw = yaw;
        odom_inited = 1;
        odom_pose_tick = pose_tick;
        task_state.x_cm = x_cm;
        task_state.y_cm = y_cm;
        map_pose_update(x_cm * 10.0f, y_cm * 10.0f, yaw, pose_tick, 1);
    }
    else {
        map_pose_update(0.0f, 0.0f, 0.0f, now, 0);
    }
    __set_PRIMASK(primask);
}
// 静止清零编码器后调用：重建轮子基准  保留已确认的全局X/Y
void chassis_odom_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < 4; i++) {
        cur_wheel_dist[i] = 0.0f;
    }
    odom_new_mask = 0;
    odom_inited = 0;
    odom_reset_tick = HAL_GetTick();
    odom_epoch++;
    __set_PRIMASK(primask);
}
// 编码器反馈提前判到位：四轮都到目标增量才提前清 busy，deadline 仍作兜底
void chassis_odom_arrive_check(void)
{
    if (!task_state.odom_on) return;             // odom 停着没有新鲜反馈，不做提前判
    if (chassis_mode != CHASSIS_MOVE || chassis_sending) return;

    for (int i = 0; i < 4; i++) {
        float actual = cur_wheel_dist[i] - chassis_start_dist[i];
        if (my_absf(actual - chassis_target_dist[i]) > CHASSIS_ARRIVE_TOL_CM) {
            return;
        }
    }
    chassis_mode = CHASSIS_IDLE;
}
// 底盘全局位姿实时解算tick
void chassis_odom_tick(void)
{
    // 航迹推算
    chassis_odom_poll();     // 每20ms读一个电机  静止时也轮询
    chassis_odom_fetch();    // 取回已到达的回复，更新 cur_wheel_dist[]
    chassis_odom_arrive_check();  // 编码器反馈提前判到位（deadline 仍兜底）
    chassis_odom_update();   // 四轮新数据齐全后积分并发布实际位姿
}

// ======== 非阻塞计时发送 ======== //
// 1ms 周期调用：每个周期只发一条电机命令，保证串口上有足够帧间隔
void car_move_send_tick(void)
{
    if (!chassis_sending) {
        return;
    }

    // 上一帧还没发完就等下一拍，保证串口上有真正的帧间隔
    if (Emm_V5_Tx_Count() != 0) {
        return;
    }

    static uint16_t move_time_gap = 0;

    if (chassis_send_index < 4) {
        if (chassis_send_mode == 1) {
            // 位置模式下每条指令隔3ms
            move_time_gap ++;
            if (move_time_gap >= 3) {
                move_time_gap = 0;
                motor_set_speedPos((Motor_Num)chassis_send_index,
                               chassis_move_speed,
                               chassis_wheel_cmd[chassis_send_index]);
                chassis_send_index++;
            }
        }
        else if (chassis_send_mode == 0) {
            // 速度模式下每条指令隔3ms
            move_time_gap ++;
            if (move_time_gap >= 3) {
                move_time_gap = 0;
                motor_set_speed((Motor_Num)chassis_send_index,
                            (int16_t)chassis_wheel_cmd[chassis_send_index]);
                chassis_send_index++;
            }
        }
        else if (chassis_send_mode == 2) {
            // 回零模式下每条指令隔4ms
            move_time_gap ++;
            if (move_time_gap >= 4) {
                move_time_gap = 0;
                motor_set_speed_Pos((Motor_Num)chassis_send_index,
                            (int16_t)40,(int32_t)0);
                chassis_send_index++;
            }
        }
    }
    else {
        chassis_sending = 0;     // 4 条都入队了，剩下的交给 car_run_tick 计时
    }
}

// 1ms 周期调用（可放在 1ms 定时器里）
// 只负责移动的非阻塞计时，到时间后清除busy 不做旋转控制
void car_run_tick(void)
{
    if (!chassis_inited) {
        chassis_init();
    }

    if (chassis_mode == CHASSIS_MOVE) {
        // 位置模式由驱动器自己走完，这里只负责结束 busy 状态
        if ((int32_t)(HAL_GetTick() - chassis_deadline) >= 0) {
            chassis_mode = CHASSIS_IDLE;
        }
    }

    // 1ms周期负责发送指令
    car_move_send_tick();

    // 反馈轮询和位姿发布由主循环的chassis_odom_tick处理
}


