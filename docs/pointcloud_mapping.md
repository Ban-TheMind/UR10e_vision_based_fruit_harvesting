# 点云建图与只规划验收

本阶段新增 `mapping` 模式：真实 UR10e 状态、固定 RealSense、点云处理、MoveIt OctoMap 与只规划服务。它不启动 YOLO、夹爪服务或采摘流程，不发送运动轨迹。项目规划服务拒绝执行类命令；该模式同时关闭 MoveIt 的轨迹执行能力。其他已有模式仍沿用原来的抓取/调试流程，**不能将 all/demo 当作本阶段的点云避障模式**。

代码在工作站完成离线检查，不代表 Foxy 编译、真实点云建图或现场避障已经验收。当前外参仍待重新标定，夹爪碰撞模型仍是简化 BOX；本阶段只检验观测、建图和规划连接，不批准真机运动或完整抓取。

## 实际数据链

```text
RealSense PointCloud2（原始时间戳与相机坐标）
  -> pointcloud_filter_node（删除 NaN/Inf/零向量，保留 RGB 等完整点记录；可靠发布）
  -> MoveIt PointCloudOctomapUpdater（距离限制、行列抽样、自身过滤、坐标变换）
  -> move_group 的 OctoMap / Planning Scene
  -> 既有 OMPL/T-RRT 碰撞规划

同一外参文件 + RealSense 内部 TF
  -> calibration_tf_node：base -> camera_link
  -> 驱动维护的 optical frames；UR 模型维护 base_link -> base
```

Foxy 的更新插件订阅使用默认可靠 QoS。清理节点以 sensor-data QoS 接收驱动数据，再可靠发布 `/harvesting/scene_points`，不依赖不同 RealSense 版本是否提供 QoS 启动参数。它只做数据清理与频率限制，不声称执行体素降采样或工作区 ROI 裁剪。`point_subsample` 是 MoveIt 的行列抽样，`max_range` 是传感器射线范围。

`site.json` 的 `perception` 集中配置原始/清理/过滤后点云话题、相机根坐标、地图分辨率、更新频率和超时。旧 profile 没有该节时使用相同默认值。`camera.calibration_file` 继续是唯一外参来源；为空时读取 camera 包内默认 JSON。没有把桌面抓取的 `xy_offset_m` 经验补偿写进 TF，也没有自动将 `calibration_verified` 改为 true。

默认值和部署校验统一由 `harvesting_config` 提供；Foxy 启动文件与 mapping-check 共用规则，插件 YAML 只保留插件声明。mapping 按自身所需条件检查，不再要求 YOLO 解释器、夹爪或示教抓取参数；实时地图就绪仍按下述门槛判定。

相机 TF 连接的是驱动根 `camera_link`，不会把已有的 `camera_color_optical_frame` 再挂一个父节点。组合关系为 `base_T_root = base_T_optical × inverse(root_T_optical)`。UR 的 base_link/base 旋转保留在机器人 TF 中，避免重复转换。

作者示例 `backWall`、`sideWall`、`table`、`ceiling` 的生成代码和加载参数已删除。启动新模式前停止旧进程并重启 move_group，避免旧内存场景仍保留这些对象；检查工具会拒绝含这些旧 ID 的场景。真实桌面仍应由相机测量进入地图，清理节点不会删除桌面。

## 就绪门槛

启用本模式时，`plan_cartesian` / `plan_joint` 在规划前后都检查：

1. 原始点云与地图更新插件的 filtered cloud 已收到，结构有效且含有限 XYZ。
2. 两者源时间戳及本地接收时间都在 `pointcloud_max_age` 内，帧一致且处理没有明显落后。
3. 在点云原始时间戳，相机 frame 可变换到规划 frame。
4. 实际 `/get_planning_scene` 服务可响应，返回的 OctoMap 坐标/分辨率符合配置，能解码为 OcTree，且至少有一个占据叶节点。

不满足时返回规划失败并记录具体原因，不回退到空环境。`/pointcloud_scene_ready` 是 `std_srvs/Trigger`，提供相同检查及可读原因。原有 MovementRequest 的接口和两个只规划 command 保持不变。

独立后台观察节点处理 TF、点云和场景服务响应，避免主规划服务同步等待时阻塞这些回调。Foxy 的 GetPlanningScene 地图没有可用的更新时间戳，因此没有误用地图 header 时间判断新鲜度；filtered cloud 保留源 header，且 Foxy 插件在写图完成后发布它。

这些门槛只证明插件在处理观测、规划场景中有地图，不证明标定正确、没有盲区或模型准确。未知空间不自动获得已观测空闲的含义；本实现不声称覆盖动态避障、执行期重规划或形式化安全保证。当前苹果同样会被建入地图，夹爪接触与抓起后的持物碰撞处理留待下一阶段。

## 本机检查

```bash
bash scripts/project check
```

无 NumPy 的纯离线解释器会跳过点记录清理测试；在现场已有 NumPy 的 ROS Python 或工作站含 NumPy 的解释器再运行：

```bash
python3 tools/test_pointcloud_mapping.py
```

## 现场步骤（尚待执行）

1. 停止原有 robot/all/camera/demo 进程，保持当前实验布置不动。
2. 运行 `bash scripts/project site-check`，核实 Foxy 感知插件、octomap_msgs 与相机点云参数；安装缺失依赖后重新构建，不能跳过缺失插件。
3. `bash scripts/project build` 完整重建工作区。
4. `bash scripts/project mapping`。这会连接机械臂控制器读取状态并启动相机，不请求机械臂或夹爪动作。
5. `bash scripts/project mapping-check`。先核对 RViz 中真实桌面、已知物体与机器人位置是否对齐；地图输出本身不算标定通过。

使用自定义现场外参时，继续设置现有 profile 的 `camera.calibration_file`，用 `HARVEST_PROFILE` 为两个终端指定同一文件。

RViz 默认显示实际规划场景和 OctoMap；原始点云、MoveIt processed cloud 可分开开启。若自定义点云话题，RViz 图层话题也要相应修改。不要在其它启动模式或 RViz 中执行运动来代替本阶段验证。

### 证明地图影响了规划

使用现场记录的六维目标，不提供未经验证的默认目标。关节角为标准 UR 顺序、单位弧度；笛卡尔目标是当前接口接受的 tool0 XYZ/RPY（米/弧度）。

```bash
# 把 Q1...Q6 替换为现场记录的有限数值
bash scripts/project mapping-check --plan-joint Q1 Q2 Q3 Q4 Q5 Q6
# 或把 X Y Z R P YAW 替换为现场记录的目标
bash scripts/project mapping-check --plan-cartesian X Y Z R P YAW
```

验收要记录三个结果：无遮挡时目标可规划；放置盒状障碍使目标配置与其碰撞时，地图更新后该目标被拒绝；移走障碍并清图重新观测后恢复。再另外验证存在绕行路线时的规划路径。仅比较两次随机采样产生的轨迹是否不同，不足以证明地图生效。

```bash
bash scripts/project mapping-check --clear-map
```

清图命令只用于独立试验间消除旧占据残影，之后必须等待新观测重新填图；地图为空时规划仍被拒绝。单相机遮挡区域不能靠移走物体立即获得完整观测，清图也不能代替处理盲区。

首轮故障验证：停止相机后，等待超过新鲜度阈值，新的只规划请求应被拒绝；TF 缺失、filtered cloud 未收到、地图无占据节点等情形也必须明确失败。恢复节点后重新通过检查，不能以降低门槛作为恢复方式。

## 实现依据

- [MoveIt2 Foxy 点云更新器](https://github.com/moveit/moveit2/blob/foxy/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)：参数、订阅 QoS、自身过滤及 filtered cloud 发布时机。
- [Foxy OccupancyMapMonitor](https://github.com/moveit/moveit2/blob/foxy/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp)：命名 sensor 参数。
- [Foxy PlanningSceneMonitor](https://github.com/moveit/moveit2/blob/foxy/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)：地图维护与规划 frame。
- [Foxy PlanningScene](https://github.com/moveit/moveit2/blob/foxy/moveit_core/planning_scene/src/planning_scene.cpp)：GetPlanningScene 的 OctoMap 输出。
