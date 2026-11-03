# 2027 工创赛智能搬运初赛场地

独立 ROS 2 Jazzy / Gazebo Harmonic 场地，版本 0.5；包含场地装置和物料，不含机器人或导航控制。已核对官方 PDF 第 3–8 页，包括初赛图、工位/物料尺寸、任务码与两批码垛要求。所有尺寸、位姿、地图与测试来自统一配置，精度性质见 [图纸依据](docs/provenance.md)。

## 构建与运行

GitHub 便携版本的构建、依赖和运行命令见 [仓库根 README](../../../README.md)。从工作区运行 `bash src/gongchuang_gazebo_world/scripts/start.sh random_seeded`。ROS launch 入口和物料阶段继续保留。

## 几何与规则

- 地面 2.4 × 2.4 m，顶面 z=0，连续碰撞板；十字车道宽 0.4 m。
- 四块 0.45 × 0.45 m 黄区；右上、右下两块 0.3 × 0.3 m 蓝区；颜色覆盖层无碰撞。
- 原料转盘直径 0.3 m、高 0.09 m，跨北边界；粗加工区在南侧、暂存区在西侧，0.58 × 0.15 m。
- 黑色障碍直径 0.05 m、高 0.10 m，固定且有真实碰撞；四个障碍场景都是测试布局，不是官方抽签结果。
- `blocked_corridor` 阻断 300 mm 底盘通过中央 400 mm 通道的局部截面，仍可能通过外侧灰区绕行；不声称全场不连通。
- 官方初赛图没有精加工区、成品区坐标。这两项在配置中禁用并标记待决赛规则，未擅自摆放实体。
- 转盘通过 C++17 Gazebo 系统插件驱动真实转动关节；默认匀速段 8 秒/圈、每 120° 停留 2 秒。官方仅给 6–10 秒/圈，停留次数和时长现场决定，默认停留方案是可配置测试值。`direction: 0` 按 `seed` 可复现抽取正/反向；设 +1/-1 可固定方向。
- 原料盘上三个独立动态物料相隔 120°、位于直径 200 mm 圆周；靠摩擦随盘运动，没有固定到转盘，可供后续机械臂抓取。物料为图 5 的底径 50 mm、顶径 30 mm、高 60 mm、两端直段各 8 mm 的形状；质量选 80 g，属于仿真暂定值。
- 每个工位有 1/2/3 编号及六级环。基于物料最大直径 50 mm，六环外径为 53/58/65/75/85/95 mm，线宽 1.5 mm；位置顺序按图 3 从左到右 3/2/1。圆环是无碰撞的平面标记，可用于后续评分器读取几何，当前未实现判罚与自动评分。
- 右侧内缘设 297 × 210 mm 的 A4 横向竖直板，中央二维码含边框为 80 × 80 mm。板中心高度 200 mm、厚 3 mm 为暂定值，位置参照图 1，实际随机范围待现场标定。真实 Gazebo 图像已成功解码。

## 两批物料场景

默认 `config/mission.yaml` 的 `mode: seeded` 按种子从六种颜色中抽三种不同颜色，随机生成两批顺序与工位编号，并同步二维码、物料外观和同色码垛映射。`mode: manual` 则使用其中保留的官方示例 `156+123+516+231`。实际生效内容见 `config/generated_mission.json`，均不是现场抽签结果。

| 场景 | 初始状态 |
|---|---|
| competition | 第一批三件在原料转盘，随盘转动 |
| batch1_coarse | 第一批按任务码放在粗加工区 |
| batch2_ready | 第一批留在暂存区，第二批三件在转盘 |
| batch2_coarse | 第一批在暂存区，第二批按第四组任务码放在粗加工区 |
| stacking_demo | 暂存区三组同色双层动态物料，接触与稳定性已实测 |

这些是分别加载的物理初始状态，不会自动瞬移搬运。示例第二批颜色 5/1/6、粗加工编号 2/3/1，最终暂存码垛编号应为 2/1/3，以匹配第一批颜色。抓取一次一件、跨区域前装载到机器人及回到抽签启停区的规则写入任务元数据，留给后续机器人任务管理器执行。

坐标沿用已有规划工程的左下原点，X 右、Y 上、Z 向上，场地中心 (1.2,1.2)。机器人初次导入应放在合法灰区，例如 (1.2,1.2)，其底部接触高度根据实际 URDF 碰撞体确定，不直接把 base_link 的 z 写死为 0。

## 地图

`maps/` 包含四种固定布局及一种种子布局对应的 transit、start_1、start_2、physical_nominal PGM/YAML 和 semantic ID 图。分辨率 5 mm，480 × 480，原点 [0,0,0]。规则禁行与物理占用分层，原料允许摆放范围保留 unknown。蓝区按任务阶段选择开放，普通行驶地图禁止使用。详见 [地图与现有规划接口](docs/map_interface.md)。

旧 path_planning 使用代码内置布局，不能直接读取本地图；提供 `config/planner_static_adapter.yaml` 对接已有 custom regions 接口，不修改旧工程。

## 修改后重建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ~/gongchuang_gazebo_ws/src/gongchuang_gazebo_world
# 修改 config/field_geometry.yaml 或 obstacle_layouts.yaml 后：
python3 scripts/generate.py
python3 test/test_geometry.py
cd ~/gongchuang_gazebo_ws
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 src/gongchuang_gazebo_world/scripts/runtime_check.py --output evidence/runtime
python3 src/gongchuang_gazebo_world/scripts/runtime_rules.py --output evidence/rules_v2
```

`runtime_check.py` 实际逐一运行十个 world，校验仿真时钟推进、工位与圆柱的实时位姿，并在独立接触测试场景中让圆柱受重力落地。它只终止自己创建的仿真进程。`contact_test.sdf` 的落体圆柱是测试试件，不是机器人。

`runtime_rules.py` 另行验证正反转、实际关节速度、停留角度、随盘物料位置和三组双层堆叠的稳定性，并加载各任务状态。几何测试共 16 项，另有 16 项候选位随机化测试。首次在新机器重新生成二维码资源时，将构建依赖装到工作区本地即可（运行已生成场地不需要此 Python 包）：

```bash
python3 -m pip install --target ~/gongchuang_gazebo_ws/.python_deps -r requirements-build.txt
```

## R1～R13 抽签 v5

默认 random_slots 从 13 个参考图候选点无放回抽取 2 位，不强制可解。支持 solvable_slots、blocked_slots、stress_slots、manual_slots；数量范围 0～13（stress 为 3～6），种子为 uint32。编号、坐标、标定依据、五种模式、蓝区边界及完整用法见 [随机化说明](docs/randomization.md)。

`config/random_generation.yaml` 集中保存候选坐标和导航任务；SDF、地图、JSON 和固定航向 A* 共用这些坐标。300 mm 车体另计 10 mm 安全裕量、10 mm 跟踪误差和 ±3° 航向误差预算。证书仅覆盖配置指定的任务；NO_PATH 场景照常保留。位置不是官方精确坐标，默认 2 个不是官方固定数量。

## 截图复现

```bash
bash ~/gongchuang_gazebo_ws/src/gongchuang_gazebo_world/scripts/start.sh audit_views
# 另一 WSL 终端，无需 ros_gz 图像桥接：
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=gongchuang_local
cd ~/gongchuang_gazebo_ws/src/gongchuang_gazebo_world
python3 scripts/capture.py --output ~/gongchuang_gazebo_ws/evidence/screenshots
python3 scripts/compare_map.py --screenshots ~/gongchuang_gazebo_ws/evidence/screenshots
```

top、isometric、cross、stations（原料俯视）、coarse、temporary、obstacles、qr_board、raw_close、stack_close 来自实际 Gazebo 相机；PNG 保留原始像素。`audit_stacking` 用相同相机展示码垛完成态，保存到另一目录以便比较。map_comparison 另行明确标注地图对照。相机只存在于验收 world，正常 competition 无传感器。GUI 还有原生截图按钮。

## 从交付副本迁移到另一台 WSL

需要 Ubuntu 24.04、ROS 2 Jazzy、C++17 编译器。经用户同意后安装：

```bash
sudo apt-get install --no-install-recommends ros-jazzy-ros-gz ros-jazzy-ros2run ros-jazzy-ros2launch python3-colcon-common-extensions python3-yaml python3-pil
mkdir -p ~/gongchuang_gazebo_ws/src
# 将本包复制到该 src 下；不要覆盖已有工作区中的同名包，先确认版本。
source /opt/ros/jazzy/setup.bash
cd ~/gongchuang_gazebo_ws
colcon build --symlink-install
```

Windows 启动脚本使用示例用户 的绝对 WSL 路径，迁移时应同步修改。后续 URDF 可通过 `ros_gz_sim create` 添加到 `gongchuang` world；本阶段未导入机器人，尚未验证你的实际 CAD/URDF。



## 本次验收

31 项原有及扩展回归在 v5 初轮通过；最终扩展为 32 项，另通过 CTest、10 个物理 world、6 个转盘/任务状态和 13 位真实障碍顶面接触。可执行 `scripts/audit_slots.py` 与 `test/capture_obstacle_acceptance.py` 复现统计及截图。公开验收摘要见 [ACCEPTANCE.md](../../../ACCEPTANCE.md)。
