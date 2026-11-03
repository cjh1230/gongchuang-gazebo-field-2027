# Ubuntu 22.04 分支（ubuntu-22.04）

目标组合：Ubuntu 22.04 Jammy、ROS 2 Humble、Gazebo Harmonic / Sim 8、Python 3.10、C++17。继续使用 Harmonic 的 SDF 和插件 API，R1～R13 坐标、抽签规则、地图和装置模型不变。

Gazebo 官方为 Jammy 提供 Harmonic 二进制包；Humble 的默认 Gazebo 配对是 Fortress，本分支采用官方文档列出的替代组合 **Humble + Harmonic**。应使用 OSRF 的 `ros-humble-ros-gzharmonic`，不能用默认 Fortress 桥接包代替。

## 安装依赖

先按 [ROS 2 Humble 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置 ROS，再按 [Gazebo Harmonic Jammy 安装文档](https://gazebosim.org/docs/harmonic/install_ubuntu/) 配置 OSRF 软件源。然后在目标 22.04 系统执行：

```bash
sudo apt update
sudo apt install --no-install-recommends \
  build-essential cmake \
  gz-harmonic libgz-sim8-dev libgz-plugin2-dev \
  ros-humble-ros-gzharmonic ros-humble-tf2-ros \
  python3-colcon-common-extensions python3-pip python3-yaml python3-pil \
  fonts-dejavu-core
```

`ros-humble-ros-gzharmonic` 与 `ros-humble-ros-gz*` 的 Fortress 包存在冲突，已有 Fortress 环境请先按 [Gazebo ROS 配对说明](https://gazebosim.org/docs/harmonic/ros_installation/) 处理，不要直接混装。本项目不会自动删除或替换系统包。不要用通用 rosdep 命令自动把已选定的 Harmonic 桥接改回 Humble 默认的 Fortress 桥接。

相机截图/转盘遥测验收还需要 `python3-gz-transport13`；二维码解码验收需要 `python3-opencv`。前者由 [Gazebo Transport 官方文档](https://gazebosim.org/api/transport/13/python.html) 指定：

```bash
sudo apt install python3-gz-transport13 python3-opencv
```

## 获取、生成、构建

```bash
git clone --branch ubuntu-22.04 https://github.com/cjh1230/gongchuang-gazebo-field-2027.git
cd gongchuang-gazebo-field-2027/gongchuang_gazebo_ws
source /opt/ros/humble/setup.bash
export GC_WORKSPACE="$PWD"
python3 -m pip install --target "$PWD/.python_deps" -r src/gongchuang_gazebo_world/requirements-build.txt
export PYTHONPATH="$PWD/.python_deps:$PYTHONPATH"
python3 src/gongchuang_gazebo_world/scripts/generate.py --seed 2027
colcon build --symlink-install --cmake-args -DGC_USE_ROS_GZ_VENDOR=OFF
source install/setup.bash
bash src/gongchuang_gazebo_world/scripts/start.sh random_seeded
```

```bash
bash src/gongchuang_gazebo_world/scripts/random_scene.sh 2027 auto random_seeded --mode manual_slots --slots R2 R7 R10 --debug-slots
ros2 launch gongchuang_gazebo_world competition.launch.py layout:=slots_debug
```

启动脚本优先使用 `GC_ROS_DISTRO`，其次使用当前 `ROS_DISTRO`，均未设置时默认 humble。自定义安装可用 `GC_ROS_SETUP` 指定 setup.bash。没有相应环境时明确报错，不继续使用错误的 ROS 版本。需要在 Jazzy 环境验证时可以显式 `GC_ROS_DISTRO=jazzy`。

CMake 在 Humble 默认直接查找系统 `gz-sim8` 与 `gz-plugin2`，无需 Jazzy 专属 vendor 包；package.xml 对 vendor 依赖加上发行版条件。可用 `GC_USE_ROS_GZ_VENDOR` 显式覆盖。

Ubuntu 22.04 自带 Pillow 9.0，本分支的地图对照工具兼容其重采样接口。其余 Python 程序保持 Python 3.10 兼容。

## 验证状态

已有主分支的全部 Gazebo 截图和物理记录来自 **Ubuntu 24.04 / Jazzy**，仅作原场地功能基线，不能视为 22.04 实机验收证据。本机没有 22.04 WSL，Docker daemon 未运行；本次不安装或修改系统环境。

本分支在现有 24.04 环境中执行了 32 项源码回归、脚本语法/4 种环境选择检查、关闭 vendor 查找的隔离构建及 Harmonic 服务端运行检查。[验证记录](../../../../evidence/ubuntu22.04/compatibility_checks.json) 明确区分实际检查环境与目标环境。这些检查不能代替 Jammy/Humble 的最终安装、桥接和 GUI 验收。

目标 22.04 环境复验：

```bash
python3 -m unittest discover -s src/gongchuang_gazebo_world/test -p 'test_*.py' -v
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 src/gongchuang_gazebo_world/scripts/runtime_check.py --output evidence/jammy_runtime
python3 src/gongchuang_gazebo_world/test/capture_obstacle_acceptance.py --output evidence/jammy_gazebo
```
