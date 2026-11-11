# 维护、调参与问题排查

本手册对应 0dcaf8b 的现有行为。2026-11-12 的整理仅增加文档和注释，没有修复下列逻辑问题或改变参数。

## 1. 修改前先记录

保存当前提交号和工作区差异；记录主控实际下载版本、Maix 应用与模块标记、设备 JSON、机械零点、测试场景及现象。源码、生成的固件、包内源码和实际设备文件分别核对，不把电脑目录中的最新文件自动当作设备版本。

一次修改完成后说明“改了什么、为什么、如何验证、仍有哪些限制”。现场参数尽量写明单位和测量来源，避免只有数字没有物理含义。

## 2. 常用修改入口

| 想修改的内容 | 入口 | 同步检查 |
| --- | --- | --- |
| 控制周期/串口外设 | Core/Src/tim.c、usart.c、工程 ioc | 定时计数、PID 调用周期、DMA 回调、引脚与设备配置 |
| 轮径/横移补偿/航向 PID | APP/Src/chassis.c | 距离与反馈单位、四轮方向、实测距离和转向重复误差 |
| 行驶速度/各段距离/视觉目标偏差 | Control/Src/control.c 文件顶部 | 车头朝向、局部/场地坐标、整段状态机触发条件 |
| 视觉位置 PID/机构反馈阈值 | APP/Src/catch.c | x/y/follow 的轴和符号、调用周期、绝对/相对模式 |
| 舵机角度/抓放高度/伸缩目标 | APP/Inc/catch.h | 机械零点、行程、云台与转盘动作间隙 |
| 步进地址/加速度 | BSP/Src/step_motor.c、BSP/Inc/step_motor.h | 电机编号到驱动器地址、发送队列、同步启动 |
| 视觉窗口/主控断流阈值 | BSP/Inc/camera.h、BSP/Src/camera.c | OK/valid/stable 含义、唯一新帧、控制层是否消费失效 |
| LAB/ROI/面积/参考点/覆盖阈值 | vision/maix_now/vision_config.py 与设备 JSON | JSON 优先级、原图和处理图尺度、配置迁移 |
| 比赛路径/基础障碍 | APP/Src/bfs.c、Control/Src/control.c | 5×5 栅格、转弯代价、车头方向、现场距离标定 |
| 道路导航与位姿 | APP 的 map_pose/map_nav/map_update 等 | 正式入口是否调用、角度参考、底盘独占、道路许可 |

CubeMX 重新生成时重点审查 USER CODE 区、DMA 模式、NVIC 优先级、时钟和串口映射。不要用一次重新生成顺带覆盖用户调参。

## 3. 故障排查顺序

| 现象 | 先看什么 | 用什么区分原因 |
| --- | --- | --- |
| 按发车不动 | car_run、car_reset、step、机构/旋转等待状态 | 未满足入口条件、等待未退出、命令未发送分别记录 |
| 四轮不同步或少一个轮动作 | 每轮地址、同步装填、发送队列、广播触发 | 捕获实际串口帧，不能只看上层下发过函数 |
| 横移距离不准 | 方向、轮径、CHASSIS_STRAFE_K、地面与载荷 | 固定距离多次实测，分开记录纵向和横向 |
| 转向抖动/不到位 | my_yaw 是否新鲜、目标参考、PID 输出、稳定计数 | 区分控制符号、死区、角度复位、输出限幅 |
| 视觉有目标但车不跟 | 当前任务、Maix 接收开关、UART 所有权、原始 OK 和偏差 | 先确认指令/回包，再看主控解析、最后看 PID 输出 |
| 画面显示阈值修改后无效 | 模块路径、JSON、加载日志、MASK 和面积诊断 | 区分源码默认值、已保存值和当前生效值 |
| 未满足覆盖也判稳定 | camera.c 的 B2 稳定候选逻辑 | 当前 OK=1 可参与 stable，与旧测试/说明不同 |
| 升降/伸缩等待结束但位置不对 | 零点、旧反馈换算、超时/重试分支 | busy 清零可能来自超时，不等同到位成功 |
| 雷达步骤执行但没有新障碍 | lidar_process() 是否运行、scan_num 是否更新 | 雷达上电/启动、DMA 接收、消费出完整圈是不同环节 |
| 全局位姿不更新 | chassis_odom_tick() 的调用、起点、四轮新样本和 IMU epoch | 当前正式入口没有周期调用，先确认接入再调算法 |
| 停止后再次发车状态异常 | hmi_stop、car_run、step>=90、机构速度/busy、子状态机 | 停止动作和重新初始化不等价，见下列问题清单 |

## 4. 当前问题与维护决策

这些问题来自源码核对和已有测试复现；本次没有改变逻辑，也没有放宽测试断言。

| 编号 | 证据与现状 | 后续处理时要确认 |
| --- | --- | --- |
| M01 | camera.c 的 maix_filter_handle() 把非零 OK 纳入稳定候选；camera_host_test.c 的 b2_found_only() 仍要求 OK=1 不稳定，实际断言失败 | B2 是否必须满足覆盖 OK=2 才准进入抓取；统一协议、实现、测试和上层消费 |
| M02 | Emm_V5_Rx_Reset() 清旧计数和有效位，但没有清实际 emm_v5_rx_ring_len；odom_host_test.c 的复位隔离检查失败 | 完整清理跨复位半帧，并验证 UART 错误、初始化和普通重启的区别 |
| M03 | NOW run() 开头 rx_handle() 被注释，uart_control_active 初始为 False；后续接收依赖此开关 | 正式联调时人工决定开启；同步核对全部设备模块与 JSON |
| M04 | task_receive() 向 Color_Code/Area_Code 枚举数组各 memcpy 3 字节；这不是复制 3 个枚举元素 | 按元素转换/赋值并验证完整 15 字节任务码；不能用初始化默认任务代替扫码链路验收 |
| M05 | main() 的 lidar_process() 被注释，chassis_odom_tick()/map_nav_task()/map_obstacle_pose_task() 无正式周期调用 | 接入阶段明确调度、位姿来源和控制所有权；不能在简历中写成整车导航已跑通 |
| M06 | send_circle_conduct() 可选择 D1/D2/D3，但 maix_start_ring() 和 NOW 当前只支持 D2；D1/D3 请求被拒绝 | 比赛圆环任务与仅数字 2 的视觉版本是否匹配；记录拒绝后的状态，避免沿用旧任务 |
| M07 | hmi_stop() 清 car_run、置 step=99，并让周期服务发急停；run_is_ok() 会阻止正常控制分支。重新发车时 step>=90 又被改回 0 | 逐项确认待发队列、伸缩速度模式、各 busy 和 static 子步骤的复位；不能只看 step=99 |
| M08 | map_pose.h 的北零约定与 chassis_odom_update() 按东零积分的说明/公式存在参考差异 | 用实测四方向统一 IMU、积分、map_tf2d 和地图 offset，不能只靠命名推断 |
| M09 | catch_x_is_ok() 等稳定计数按控制调用累加，未以新 FRAME_ID 计数 | 文档中的“20帧”实际是名义 20 个控制周期，不能作为 20 张独立图像证据 |

涉及上述行为的修复应单独评审，并重跑针对性回归。本次发现问题不表示所有实车任务都无法运行，也不表示默认任务或现场旧版本能验证当前代码。

## 5. 构建主控

需要 CMake 3.22 及以上、Ninja、可用的 arm-none-eabi 工具链。先让这些工具能在当前终端找到，然后在 gongxun_1 目录运行：

~~~powershell
cmake --preset Debug
cmake --build --preset Debug
~~~

本次为避免覆盖现有构建，使用独立目录：

~~~powershell
cmake -S . -B build/DocumentationCheck -G Ninja '-DCMAKE_BUILD_TYPE=Debug' '-DCMAKE_TOOLCHAIN_FILE=E:/code/gongxun/gongxun_1/cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_MAKE_PROGRAM=D:/STM32CubeCLT/STM32CubeCLT_1.18.0/Ninja/bin/ninja.exe'
cmake --build build/DocumentationCheck
~~~

上例含本机路径，其他电脑应替换。PowerShell 下将整个 -D参数=值 加引号，避免工具链文件名被错误拆分。编译完成只代表生成固件，不代表已下载或验证车辆动作。

## 6. 运行已有回归

主机 C 测试使用 Windows GCC，不能用 ARM 编译器运行生成的程序。以下在 gongxun_1 目录执行，Compiler 路径可替换：

~~~powershell
./Test/Host/run_camera_test.ps1 -Compiler 'D:/CLion/CLion 2025.2.2/bin/mingw/bin/gcc.exe'
./Test/Host/run_odom_test.ps1 -Compiler 'D:/CLion/CLion 2025.2.2/bin/mingw/bin/gcc.exe'
./Test/Host/run_map_nav_test.ps1 -Compiler 'D:/CLion/CLion 2025.2.2/bin/mingw/bin/gcc.exe'
~~~

按改动范围另选 run_map_pose_test、run_map_plan_test、run_map_pillar_test、run_map_observe_test 等脚本，不要求每次纯文档调整都运行所有硬件无关测试。

在 vision/maix_now 目录运行：

~~~powershell
python -B -m unittest discover -s tests
~~~

在 lidar_replay 目录运行下列命令可验证回放工具；本次未重新运行它，不沿用历史次数作为当前结果：

~~~powershell
python -B -m unittest discover -s tests -v
~~~

遇到系统临时目录拒绝访问，应先处理执行环境，再判断算法或配置测试是否失败。不要为了得到绿色结果而直接改断言。

## 7. 实车记录应包含什么

记录提交号、实际固件/视觉包、设备 JSON、机械/地面/照明条件、测试步骤和结果。运动记录至少包括目标距离/角度、独立实测值、重复次数；视觉记录应区分采集/检测 FPS、实际成功接收频率、误检/漏检和任务成功率。

一次完整流程跑通、重复运行稳定、量化达到指标分别记录。比赛成绩附正式名称、级别、年份和证书；个人贡献由本人确认后再进入简历。
