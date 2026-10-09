# 分支部署约定

- `main`：工位 Humble + Pixi。
- `codex/foxy-ubuntu2004`：实验室 Ubuntu 20.04 / ROS2 Foxy，保留独立推理环境。
- `codex/pre-foxy-baseline-20260928`：按用户要求同步到 main 的当前内容，旧版本仍可从提交历史查看。

三条分支共同使用外部固定相机、同一 Camera → UR base 标定矩阵、base_link 规划坐标及官方 Robotiq SDK。
环境适配可以不同，共同实现不得独立漂移。
目前标定仍需已知点验证，官方 SDK 的状态读取也需在实际夹爪上确认；默认不执行设备命令。

在更新远程引用后，运行 `python3 scripts/check_branch_alignment.py --remote origin` 检查已提交内容。
该检查只读取本地 Git 引用，不访问网络或设备；未提交更改不属于检查范围。
每次改动共同文件，都应同步到另外两条分支并重新运行检查。
