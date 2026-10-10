# 唯一维护版本

只维护 Ban-TheMind fork 的 `main`，以机械臂旁 Ubuntu 20.04 / ROS2 Foxy 为唯一部署目标。
fork 是你的仓库，main 是仓库中的主分支，不是两个版本。
本地和 fork 只保留 `main`。原 Foxy 与基线分支的提交历史已合并进 main，多余分支已删除。
原来的工位 Humble/Pixi 启动代码已从当前版本移除，旧内容仍在提交历史。
本次同步没有安装、卸载或修改任何电脑的 ROS2 环境。

当前内容来自现场 Foxy 最新版本，包含工厂标定、固定外部相机转换、官方串口夹爪 SDK、headless 修复和带状态检查的 Home 入口。
现场继续使用 `tools/foxy_site_preflight.sh` 和 `tools/ur10e_control.sh`；具体操作见 `docs/foxy_ubuntu2004.md`。
工位用于阅读修改代码和运行 `python tools/offline_preflight.py`（需要 numpy 和 C++ 编译器等已有依赖）。
离线检查不会启动机器人或证明真机动作成功。工位不再作为第二套完整机器人运行环境维护。

后续直接在 main 开发、提交和推送。其他电脑从你的 fork 的 main 更新，不再维护分支同步流程。
更新前检查本地改动，再执行 `git fetch origin` 和 `git merge --ff-only origin/main`。
