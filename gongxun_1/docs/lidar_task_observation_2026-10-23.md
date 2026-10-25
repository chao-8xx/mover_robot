# 按任务方向确认小柱：实现与使用

本轮基于黑柱实测：1200mm已有一组100/100，1500mm三组182/300，1800mm稀少或无有效回波。算法保留实际接收和360格点云，不补点。多圈确认提高证据的一致性，不改变真实单圈命中率，也不凭单点识别颜色或拟合直径50mm的圆。

## 当前运行方式

- `Core/Src/main.c` 中 `LIDAR_PILLAR_TEST_ENABLE=0` 为默认：整车初始化，里程计发布真实位姿，运行已有的静止 `map_pose_test()`，默认观察 N8→N5 的 R11。车放右下启停区2朝北，短按KEY1设置已有里程计起点；等待IMU和四轮反馈产生有效位姿。此入口不发车、不转向。
- 宏为1：独立原始点/360格对照，跳过整车初始化及正式地图入口，保留原来测试方法。此模式不会执行新的道路确认模块。
- 两种模式在主循环互斥。旧代码主循环无条件调用独立实验和位姿地图，会争用全局cloud；本轮已修正。
- 有活动的9节点 `map_nav_scan` 任务时，`map_obstacle_pose_task()`从导航读取当前节点和下一节点，自动选择当前路径的下一道路。封路后由已有导航规划逻辑选择新路，再切换观察方向。
- 其他任务在停车后调用 `map_nav_observe_begin(current_node,next_node)`，主循环继续 `map_obstacle_pose_task()`；同一请求重复调用不清空窗口。结束调用 `map_nav_observe_end()`。原来的 `control_process()`比赛步骤和旧BFS没有被改写成9节点导航，也没有自动启动导航任务。

```c
// 例：当前已经在N8停车，下一步准备进入N5，优先观察R11。
map_nav_observe_begin(8, 5);
// 主循环持续执行已有真实位姿地图入口，不在10ms控制中断中做点云计算。
map_obstacle_pose_task();
```

任务选择必须与实际所在节点区域相符；当前道路中间停车不能冒充已到下一节点。未调用begin的旧地图测试入口仍用原有连续两圈规则，新窗口确认不会再送入该两圈计数器。

## 观察选区

从静态地图取得目标道路区域和前方节点区域，按实际雷达位置、车头朝向和安装偏移计算局部角区，带3°余量。角区跨0°时 `start>end`。对完整单圈聚类先做原有尺寸/距离/点数筛选，再检查真实点是否位于选区、角区和目标道路。

选区只包含相邻目标道路及前方节点，不是整个前方方向的所有远处道路；也不是当前停车节点的全方向近场保护。原始360°接收保持。选区角度余量不会扩张地图道路边界。一个真实点在节点内时，仍沿用原有“影响相连出口”的保守道路映射。

这套选区用于确认存在障碍，不提供道路完全被看清的证明。没有候选、5圈未确认、位姿失效、截断或没有新扫描均不能自动设置FREE；新版观察入口不授权显式放行。导航通行许可仍由上层完整观察流程提供。

## 位置匹配参数

`APP/Inc/map_nav.h`：

| 参数 | 初值 | 含义 |
|---|---:|---|
| MAP_OBSERVE_MATCH_MM | 80.0f | 不同圈候选到固定参考位置的最大距离，mm |
| MAP_OBSERVE_WINDOW | 5 | 最新有效连续扫描窗口，位图最多8圈 |
| MAP_OBSERVE_MIN_HITS | 3 | 窗口中至少命中的不同扫描数 |
| MAP_OBSERVE_MAX_CANDIDATES | 16 | 固定候选容量，满时overflow显式置1 |
| MAP_OBSERVE_ANGLE_MARGIN_DEG | 3.0f | 道路几何角区余量，degree |

80mm是跨圈关联半径，与现有100mm单圈连接距离、100mm平面尺寸上限不同。候选使用已有地图坐标，限一次停车观察，复用已有TF而不增加运动补偿。固定anchor不逐圈跟随移动，避免链式漂移。同一圈同一候选只记一次，位置相近的碎片不会制造多票。

判断阈值时先看日志：`now=0`且没有frame候选是本圈无合格选区候选；有frame候选且`mismatch`增加表示新位置未匹配到已有参考，不一定是错误，也可能是另一个物体。只在确认同一固定物体有有效候选却被分成多个位置时，依据其坐标散布调整半径。不要用扩大半径弥补原始回波缺失。缩小半径减少不同物体混票，但可能增加漏关联。修改窗口时要求1≤MIN_HITS≤WINDOW≤8。

## 确认与中断

命中/缺失/命中/命中可在第4圈确认。确认圈直接提交近5圈真实点涉及的道路，没有额外两圈等待。窗口内历史状态与本圈current_hit分开；仅当前有新回波时提交确认。

同一scan_num不重复处理。跳圈、采集间隔超过1000ms、定位/底盘状态失效、接收丢圈、任务方向改变均清待确认记录；任务层恢复后先丢弃过渡扫描。已经BLOCKED的道路保持，候选过期不自动解封。每个候选保存最近5圈的真实道路点归属，不用包围框内部空白封路。

## 串口诊断

已有位姿/道路日志之外每秒打印：

```text
[OBSERVE] N8->N5 R11 angle=... scan=... frames=... now=... tracks=... confirmed=...
 pending=0x... block=0x... mismatch=... overflow=... cost=...us first=...ms
[OBSERVE C0] xy=(...) d=... points=... span=(...) hits=3/5 now=1 last=...
```

- `now`（摘要）是本圈合格聚类数量；候选行的`now`是本圈是否确实命中。
- `confirmed`是当前窗口命中次数达标的候选数量；`block`仅表示本圈新回波支持提交的道路。长期封路看`POSE ROAD`，不能把旧BLOCKED当成新命中。
- `first=-1`是本次窗口还没确认；确认耗时从第一个有效观察圈采集结束计算，不含停车和过渡圈等待。
- `overflow=1`包括上游聚类截断或16个候选不足，不静默覆盖。
- `cost`只统计map_observe_update的有效处理阶段，含位置确认及地图提交，不含串口、接收解析、点云转换和单圈聚类；DWT测量可能包含中断耗时。实际新增耗时需板端日志确认。

## 本轮逐文件变更

| 文件 | 修改及目的 |
|---|---|
| APP/Inc/map_nav.h | 合入观察参数、候选/诊断结构及map_nav_observe_*接口。 |
| APP/Src/map_nav.c | 合入停车观察、固定位置关联、5圈3次及诊断；统一同步下一观察道路，原动作状态机保持。 |
| APP/Inc/map_plan.h | 声明道路选区和雷达角区纯几何接口。 |
| APP/Src/map_plan.c | 合入目标道路/前方节点范围、实际雷达角区计算。 |
| APP/Inc/map_update.h | 声明已确认道路入口。 |
| APP/Src/map_update.c | 新入口校验扫描身份、疑似道路阻止放行、确认直接封路并更新变更计数。旧两圈入口保留。 |
| APP/Inc/map_lidar.h | 公布实际点到道路的共用接口。 |
| APP/Src/map_lidar.c | 共用原有实际点归属规则，避免新版/旧版重复几何实现。 |
| Control/Inc/task.h | 说明任务方向和新确认接口使用约定。 |
| Control/Src/task.c | 导航scan阶段选择下一道路；任务改变后丢过渡圈；运动/定位/接收中断清窗口；有任务时调用新确认。 |
| BSP/Inc/headfile.h | 通过现有map_nav.h提供观察接口，不再包含独立观察头文件。 |
| CMakeLists.txt | 移除独立map_observe.c/.h编译条目，复用map_plan/map_nav。 |
| Core/Src/main.c | 默认模式宏置0，独立采样实验与正式位姿地图主循环互斥；保留用户注释掉旧测距调用的修改。 |
| Test/Src/map_stage_test.c | 现有静止位姿入口默认演示N8→N5，并增加每秒观察/候选诊断，不启动车辆。 |
| Test/Host/map_observe_host_test.c（已有） | 保留窗口边界测试，补充规划角区与实际导航plan→scan同步，不允许观察测试触发底盘动作。 |
| Test/Host/run_map_observe_test.ps1（已有） | 链接实际map_plan/map_nav及感知源码，运行观察回归。 |
| Test/Host/lidar_pillar_native_host_test.c（已有未跟踪文件） | 合成1800mm回放显式配置1800mm，避免随用户1500mm台架宏变动而误失败。 |
| Test/Host/odom_include/headfile.h（已有未跟踪文件） | 只通过现有map_nav.h提供观察类型。 |
| Test/Host/perception_include/headfile.h | 补齐合并后真实导航/位姿/底盘声明。 |
| Test/Host/nav_include/headfile.h | 复用已有感知测试头文件，使同一导航源码参与回归。 |
| Test/Host/run_map_nav_test.ps1 | 补齐合并后实际感知源码依赖。 |
| Test/Host/run_odom_test.ps1（已有未跟踪文件） | 链接实际导航及感知源码，使原有map_pose_test继续参与回归。 |
| docs/superpowers/plans/2026-10-23-pillar-observation.md（新增） | 实施步骤和验证边界。 |
| 本文（新增） | 参数、入口、诊断、限制、逐文件变更及验证记录。 |

## 验证记录

- 新主机测试：91 checks / 0 failures；使用真实聚类/TF/道路源码，外设由主机替代，不代表黑柱实测确认率。独立审查发现的静默超时已增加主循环轮询清理和恢复扫描屏障，并经复查。
- 既有小柱地图：79 checks / 0 failures。
- 原始采样：242 checks / 0 failures。
- 定位：54 checks / 0 failures。
- 导航：48 checks / 0 failures。
- 路径规划：352512次路径对比 / 0 failures。
- 里程计：178 checks passed。
- Debug ARM固件编译链接通过：RAM41920B/128KiB，Flash121580B/512KiB。
- 独立原始实验模式编译链接通过：RAM40400B，Flash107356B。
- 观察结构812B，静止诊断另有同大小静态快照；不保存多圈点云，无堆分配。
- 本轮没有烧录，没有板端新耗时/新检出率结论，没有提交或推送Git。

本轮开始时tracked文件无未提交修改；已有未跟踪的原始雷达测试、里程计测试及相关说明已保留。执行期间另有并行保存的main.c旧测距入口注释、map_nav.h/c格式调整、map_stage_test.c正面只读诊断及台架测试头文件，均保留；台架目标仍为1500mm。这些格式/正面诊断改动不计入本轮观察模块的实现归属。除上表明确修改的几个测试文件外，未改其他原有未跟踪文件。修改前入口文件及状态备份在build/host_checks/pillar_observe_20261023_baseline。


## 文件收拢与导航复杂度（本次调整）

按用户要求，没有新增文件；原APP/Inc/map_observe.h和APP/Src/map_observe.c已移除。观察几何放入map_plan，停车确认放入map_nav。调用点统一为map_nav_observe_*，不保留兼容头文件。task.c只调用map_nav_observe_sync()选择导航下一道路，实际雷达接收和恢复屏障仍在任务层。

本次涉及既有文件：APP/Inc|Src/map_plan、APP/Inc|Src/map_nav；Control/Inc/task.h、Control/Src/task.c；Test/Src/map_stage_test.c；BSP/Inc/headfile.h；CMakeLists.txt；Test/Host/map_observe_host_test.c、run_map_observe_test.ps1、run_map_nav_test.ps1、run_odom_test.ps1、perception_include/headfile.h、nav_include/headfile.h、odom_include/headfile.h；本说明和既有实施计划。每项分别用于迁移实现、统一调用、移除旧依赖、验证迁移及更新说明。map_obs和map_update本次没有修改算法。

map_nav原有8状态仍保留，包含规划、观察、转向、移动、到位和结束/异常处理。文件合并不等于减少这些行为。复杂度主要来自它完整管理底盘动作，而control_process也管理任务动作；后续若大幅精简，应先统一动作执行者，再决定map_nav是否只做下一道路、观察结果和重规划协调。当前不把这两套执行器同时启用，不在文件迁移时删除到位、过期定位、道路许可和重复指令保护。

本次验证：观察91项、导航48项、小柱79项、里程计178项全部通过；原始采样242项通过；路径规划352512次比对无失败；Debug固件编译链接通过，RAM41920B，Flash121580B。未烧录，未获得新的板端耗时或黑柱检出率证据。
