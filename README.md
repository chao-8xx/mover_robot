# 工训搬运机器人

基于 STM32F407VET6 的比赛机器人软件仓库，包含麦克纳姆轮底盘、闭环步进驱动器通信、IMU 航向控制、视觉辅助对准、抓放机构、串口屏及雷达相关模块。

维护基线：2026-11-12 核对本地 master，功能提交为 **0dcaf8b（2026-11-07）**。本次整理增加文档和注释，保留运行逻辑与现场参数。设备实际下载的固件、视觉应用及保存参数需要另行核对。

## 从哪里开始看

| 文档 | 主要内容 |
| --- | --- |
| [整车架构与调度](docs/architecture.md) | 目录分工、启动顺序、主循环和中断、任务状态机、模块接入状态 |
| [运动控制与抓放机构](docs/motion-control.md) | 坐标和单位、四轮解算、同步启动、PID、视觉对准、机构动作 |
| [通信与数据约定](docs/interfaces.md) | 串口分工、视觉协议、有效/稳定状态、编码器接口、参数来源 |
| [维护、调参与问题排查](docs/maintenance.md) | 修改入口、调参顺序、故障排查、当前已知问题、构建与回归命令 |
| [本次核对与验证记录](docs/verification-2026-11-12.md) | 当前测试结果、文档与源码差异、验证范围 |
| [机器人／运动控制简历素材](docs/resume-project-notes.md) | 项目介绍、技术证据、个人贡献与成果待核实项、面试准备 |

## 仓库结构

~~~text
gongxun_1/              STM32 主控工程
  Core/                CubeMX 外设配置与 main.c
  BSP/                 串口、驱动器、IMU、摄像头、雷达等驱动
  APP/                 底盘、机构、PID、路径搜索、地图与导航
  Control/             整车初始化、任务数据和比赛状态机
  Test/                板端测试入口、主机 C 回归测试
  docs/                雷达、地图、里程计等专项设计与历史记录
vision/
  maix_now/            最近提交的独立视觉应用，版本 2.4.0
  maix_main/           另一套视觉实现，与 NOW 分开维护
  maix_last/           保留版本
  maix_old_10.15/      历史版本
  openmv_qr/           OpenMV 扫码程序
lcd/                   串口屏工程及字体资源
lidar_replay/          电脑端雷达回放、参数对比及 C 源码对照工具
docs/                  整车维护入口与项目素材
~~~

## 当前运行方式

主控使用主循环与定时中断协作。主循环处理摄像头和扫码服务；TIM6 每 1 ms 处理发送/计时，TIM7 每 5 ms 更新 IMU 等服务，TIM2 每 10 ms 推进比赛状态机。

比赛路径调用保留 bfs 命名的 5×5 加权路径搜索。另有按毫米位姿运行的道路地图与分段导航模块，但当前正式入口没有周期调用里程计、道路建图和分段导航，不能将模块存在理解为整车已启用自主导航。

当前 Maix NOW 源码保留本地触屏调试状态：run() 循环开头的 rx_handle() 被注释。完整主控联调需要人工检查该设置和设备文件，见 [通信说明](docs/interfaces.md)。

## 构建与验证

主控使用 CMake、Ninja 和 Arm GNU 工具链；现有 Debug/Release 预设位于 [CMakePresets.json](gongxun_1/CMakePresets.json)。具体命令见 [维护手册](docs/maintenance.md)。

最近核对：NOW 视觉 157 项测试通过，独立道路导航 48 项检查通过；主控摄像头接口和里程计回归仍有失败，详情见 [验证记录](docs/verification-2026-11-12.md)。电脑测试、固件编译、设备下载和实车验收分别记录。

## 已有专项资料

- [视觉 NOW 使用说明](vision/maix_now/README.md)及[历史验证记录](vision/maix_now/VALIDATION.md)
- [主控摄像头接口和主机回归](gongxun_1/Test/Host/README.md)
- [底盘航迹推算](gongxun_1/docs/chassis_odom_2026-10-22.md)
- [地图位姿接口](gongxun_1/docs/map_pose_interface_2026-10-20.md)
- [道路分段导航](gongxun_1/docs/map_nav_2026-10-21.md)
- [任务方向多圈观察](gongxun_1/docs/lidar_task_observation_2026-10-23.md)
- [雷达回放工具](lidar_replay/README.md)

历史文档中的通过次数、现场参数和功能状态只对应其记录时的版本；维护时以当前源码和重新执行的结果为准。
