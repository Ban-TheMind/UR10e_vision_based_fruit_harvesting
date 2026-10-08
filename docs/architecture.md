# 项目职责

- `harvesting_bringup`：启动入口、现场配置、相机标定数据；不包含采摘算法。
- `end_effector_description`：机械臂末端几何、安装关系、机械臂运动学标定。
- `ur10e_moveit_config_official`：当前使用的规划配置（基于上游并有本地改动）。
- `ur10e_moveit_config`：保留的 Setup Assistant 配置；不在默认启动链路中。
- `camera`：图像同步、目标检测、三维定位、可视化。
- `gripper`：RG2 HTTP 通信；地址、超时由参数提供。
- `moveit_path_planner`：规划场景和运动请求；规划与执行结果分别处理。
- `custom_interface`：模块间 ROS 请求定义。
- `demo_package`：采摘策略和共用请求处理；不直接连接设备。
- `experiments/legacy`：历史实验代码，不注册为正式可执行入口。
- `scripts`：环境、构建和启动包装；`docs`：使用与现场验证。

原包名保持不变，避免同时改变 Python 导入和 ROS 接口。
`display.launch.py` 转发统一入口，默认使用假硬件；旧 `auxiliary.launch.py` 仅提示新入口。
环境缓存与 build/install/log 继续采用工作空间标准布局。

当前阶段没有加入 MuJoCo、没有宣称完成物理抓取仿真，也没有自动确认现场标定。
