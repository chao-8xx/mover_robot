# 运动控制与抓放机构

功能基线：0dcaf8b。参数为 2026-11-12 核对的源码值；本次未重新标定机械或验证实车精度。

## 1. 先分清单位与坐标

| 数据 | 单位和含义 | 所在位置 |
| --- | --- | --- |
| car_run_distance(speed, x_cm, y_cm) | speed 为驱动器速度指令 RPM；距离 cm；车体 +X 向右、+Y 向前 | [chassis.c](../gongxun_1/APP/Src/chassis.c) |
| car_speed_move(x, y, omega) | 参与四轮合成的速度分量，与电机 RPM 指令同尺度；omega 不是已换算的 rad/s | chassis.c |
| my_yaw / 旋转目标 | 角度 °，连续角可跨多圈；具体零点由复位与上层朝向决定 | [hwt101.c](../gongxun_1/BSP/Src/hwt101.c)、control.c |
| cur_x / cur_y | 比赛栅格索引 0～4，不是实测厘米位置 | [task.h](../gongxun_1/Control/Inc/task.h) |
| x_cm / y_cm | 航迹推算维护的全局位置 cm | task.h、chassis.c |
| map_pose_t | 全局 X/Y 为 mm；yaw 为地图变换角 °，归一化到 [-180,180) | [map_pose.h](../gongxun_1/APP/Inc/map_pose.h) |
| follow_x / follow_y | 相机参考点对应的图像偏差，原图像素坐标尺度；不是 mm | control.c、camera.c |
| 电机位置命令 / 完整位置反馈 | 控制命令使用 3200 脉冲/圈；完整反馈为 65536 单位/圈 | chassis.c、Emm_V5.c |
| 舵机 / 机构目标 | 舵机角度 °；升降和伸缩目标 cm | [catch.h](../gongxun_1/APP/Inc/catch.h) |

比赛方向换算按车头向量和 track_dir_to_yaw() 工作。地图位姿接口的注释约定北为零，chassis_odom_update() 当前积分公式则按东为零解释 my_yaw。两者涉及安装/参考角关系，接入新导航前应结合 map_pose_yaw_offset_set()、map_tf2d.c 和实测方向统一；不能只修改一处注释就认为坐标已一致。

## 2. 四轮运动解算

电机编号从前到后、从左到右为 1/2/3/4，对应驱动器地址 1/2/3/4。当前速度合成公式：

~~~text
w0 =  y + x - omega
w1 = -y + x - omega
w2 =  y - x - omega
w3 = -y - x - omega
~~~

正负号包含当前安装方向。它是项目采用的指令映射，不是带轮半径、车体几何尺寸和 SI 单位的完整速度模型。换驱动器方向、轮子位置或接线后，先逐轮验证，再验证平移和旋转。

固定距离运动先把 x_cm、y_cm 转成轮脉冲，使用相同的平移部分组合。横移额外乘 CHASSIS_STRAFE_K；对角位移由各轮合成，不能简单当成四轮脉冲相同。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| CHASSIS_WHEEL_DIAM_CM | 7.9 | 控制距离/轮周长换算 |
| CHASSIS_MOTOR_PULSE_PER_REV | 3200 | 控制脉冲每圈 |
| CHASSIS_STRAFE_K | 1.10 | 横移补偿，正向命令乘此系数、里程计横向解算除以此系数 |
| CHASSIS_MOVE_MARGIN_MS | 300 | 估算动作结束时增加的余量 |
| CHASSIS_ARRIVE_TOL_CM | 0.5 | 反馈提前结束 busy 的轮位移容差 |
| STEP_MOTOR_CHASSIS_ACC | 150 | 传给驱动器的加速度档位，不是 m/s² |

距离命令使用车体系；想沿场地 X/Y 移动，要先根据车头方向换算。

## 3. 四轮同步启动与 busy

car_move_xy() 只保存四轮目标，并设置 CHASSIS_MOVE。TIM6 的 car_run_tick() 调用 car_move_send_tick()，在上一帧发送队列清空后继续分时装填：

1. 每个轮子用 motor_set_speedPos_sync() 装填相对位置目标，暂不启动。
2. 四轮装填后再发送 Emm_V5_Synchronous_motion(0) 广播触发。
3. 发送队列由 USART2 DMA 和发送完成回调推进。

发送间隔计数在队列可继续时累加，因此“3 ms”是名义节拍，整体还受 UART 占用和其他电机命令影响。当前同步装填函数没有向底盘上层返回每轮入队结果，广播触发也不能作为四轮已接受目标的确认。

这里的“多机协同”表示多电机同步启动，不是多机器人通信或协同导航。

CHASSIS_MOVE 的 busy 主要由估算 deadline 清除。chassis_odom_arrive_check() 另有四轮反馈提前到位判定，但依赖 chassis_odom_tick() 周期执行，当前正式入口未调用。busy=false 表示软件等待结束，不能单独作为实际位置测量或驱动器到位证明。

## 4. 航向与视觉 PID

car_rotate_angle() 设置目标，需持续调用 car_rotate_ctrl() 完成闭环，到位后请求 IMU 归零。比赛状态机主要使用 car_rotate_yaw()，由 rotate_state 分支持续调用，保持连续角参考。两个接口的角度复位行为不同，不能互换后沿用旧目标。

当前航向参数：Kp=3.7、Ki=0.00001、Kd=1.8，积分限幅 3、输出限幅 ±120 RPM，YAW_THRESH=0.3°，稳定计数 40 次。按 10 ms 调用为名义 400 ms；若调用节奏改变，稳定时间也改变。

位置式 PID 的实际形式为：

~~~text
error = target - feedback
integral += Ki * error
output = Kp * error + integral + Kd * (error - last_error)
~~~

实现没有显式乘/除 dt，Ki/Kd 与调用周期绑定。积分限幅独立于总输出限幅；当前输出按 ±maxOutput 对称限制，传入的 minOutput 没有参与位置式计算。yaw_pid 同时供转向和视觉跟踪使用，切换前后需要核查 PID_Reset()/PID_Clear() 的位置。

follow_target() 同时计算航向、图像 X、图像 Y 三个 PID，再发送：

~~~text
车体 x = -图像 y PID 输出
车体 y = -图像 x PID 输出
旋转分量 = 航向 PID 输出
~~~

这个轴交换和符号来自当前安装，不表示像素距离已经转换为物理距离。follow_stretch_target() 使用 follow_pid 控制齿轮齿条，误差绝对值小于 2 时输出为零；读到过有效伸缩位置后才应用行程方向限制。

视觉跟踪会调用 chassis_odom_hold() 撤销旧的全局位姿可信状态。恢复航迹推算需要在已知实际位置重新建立起点和轮子基准。

## 5. 抓放机构

catch.c 管理升降、伸缩、夹爪、云台、转盘，catch.h 提供现场机械位置。Motor_5 对应升降丝杆，Motor_6 对应齿轮齿条；文件中同时保留其他伸缩接口，查调用链再判断实际使用者。

| 机构 | 主要接口 | 完成判断 |
| --- | --- | --- |
| 升降 | catch_lift()、catch_lift_to_height() | 区分相对计时模式和绝对反馈模式；还含超时/重试处理 |
| 齿轮齿条伸缩 | catch_stretch_send()、catch_stretch_distance() | 速度跟随或绝对位置反馈、容差、超时/重试 |
| 保留伸缩丝杆 | catch_stretch_dist()、catch_stretch_to_pos() | 按实际调用及其模式判断，不能与齿轮齿条参数混用 |
| 夹爪/云台/转盘舵机 | claw_ctrl()、holder_rotate()、turntable_rotate() | servo_wait_ms() 软件等待，没有新增舵机到位反馈 |

升降控制命令换算为 4000 脉冲/cm，旧反馈接口另有 LIFT_FEEDBACK_PER_CM=320 的现场标定。伸缩也有独立零点和换算。不能把四轮完整反馈的 65536 单位/圈直接搬进机构旧接口。

动作主流程一般为“下发动作 → 设置等待状态 → 周期服务推进 → busy 结束后进入下一步”。等待结束也可能来自超时，因此不能把 busy 清零统一称为到位成功。复位和急停时，要分别检查驱动命令、待发队列、机构 busy/速度模式及各子状态机。

## 6. 建议调参顺序

1. 确认电机地址、方向、控制脉冲和反馈单位；分别验证前进、横移、旋转。
2. 用已知物理距离标定轮径/横移补偿，记录指令、实测和重复误差。
3. 固定控制周期，调航向 PID，再检查不同方向和带机构动作时的表现。
4. 固定相机、镜头和参考点，验证图像偏差对应车体运动的符号，再调视觉位置环。
5. 分别标定机构零点、目标高度和行程，最后检查抓放状态机的动作间隙。

每次只改变一组有明确因果关系的参数，保留修改前后数值和实测。代码中的阈值、容差、等待时长属于配置，不能直接当作实测精度、响应时间或成功率。
