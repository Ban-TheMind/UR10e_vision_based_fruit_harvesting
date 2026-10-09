# Official upstream dependency

Source: https://github.com/robotiq/grippers
Commit: 9017b5707cb28a8ea21fd7c4e84a50f7c56aaa5d
SDK version: 1.1.0. License: BSD-3-Clause (see LICENSE).

sdk_cpp/ is copied unchanged from this commit. Vendoring pins the dependency
and permits offline/reproducible colcon builds without submodule setup.
Only the project bridge outside this directory is maintained by this project.
