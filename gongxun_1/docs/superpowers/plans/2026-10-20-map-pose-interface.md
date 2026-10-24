# 雷达导航位姿接口实施计划

**目标：** 为队友的地图X/Y定位预留发布接口，与现有my_yaw一起形成统一位姿，供真实雷达建图读取。

**范围：** 本次完成位姿接口、地图任务入口、桌面测试和固件编译；自动驾驶调度、运动中停车、自由空间确认属于后续导航阶段，不在当前主循环启用。

**源码依据：** `BSP/Src/hwt101.c`的`imu_get()`形成my_yaw；`BSP/Src/timer.c`每5ms更新；`Control/Src/control.c`的`track_dir_to_yaw()`定义东向0度、北向90度。`APP/Src/tf2d.c`的车头为局部+Y，因此默认地图变换角为`my_yaw - 90度`，应允许起点对齐后设置偏移。

## 执行步骤

- [x] 先写`Test/Host/map_pose_host_test.c`，覆盖默认无效、四方向和多圈角度、超时/未来时间/时钟回绕、坏数据撤销旧快照、角度偏移变更。
- [x] 运行`Test/Host/run_map_pose_test.ps1`确认接口尚缺失。
- [x] 新建`APP/Inc/map_pose.h`和`APP/Src/map_pose.c`。接口：`map_pose_init()`、`map_pose_yaw_offset_set(offset_deg)`、`map_pose_update(x_mm,y_mm,imu_yaw_deg,tick,valid)`、`map_pose_get(tick,&pose)`。发布和读取都在主循环；无动态分配。超时暂设500ms，可在头文件调整；它是软件默认值，不是实测定位周期。
- [x] 新建`map_obstacle_pose_task()`，从位姿接口取得有效快照后转交原有`map_obstacle_task()`；无效时仍调用原入口走失效处理，保留雷达接收服务和确认计数清理。
- [x] 加入构建和统一头文件；保持现有主循环和旧BFS调用入口。
- [x] 再运行桌面测试、Debug固件编译和`git diff --check`，写清队友接入方法与角度/时间/有效性约定。

## 接入约定

定位模块在主循环发布`map_pose_update(x_mm, y_mm, my_yaw, HAL_GetTick(), localization_valid && yaw_valid)`；单位mm/度，坐标与`map_track.c`一致。必须发布本次真实更新，不能每次循环用旧X/Y刷新时间。yaw有效性由发布者保证；现有`hwt101.valid`和`last_update_ms`表示任意有效IMU帧，并不单独证明角度帧新鲜。

初始朝向按现有控制定义默认东向0度；若起点方向不同，调用`map_pose_yaw_offset_set(实际地图车头角 - 当前my_yaw)`，随后重新发布。地图车头角：北0度、东-90度、西90度、南-180度。IMU重新归零、定位重置时先发布valid=0，再重新对齐发布。

## 导航后续边界

在该接口之上继续实现：可信当前节点与目标节点、下一道路观测与放行、Dijkstra路径执行、真实到位判断、运动中障碍停车、停车位置恢复与重规划。`car_is_busy()==0`不等价于物理到位；`CHASSIS_IDLE`也不独立证明车已停稳。不能因本接口完成就宣称自动避障闭环完成。
