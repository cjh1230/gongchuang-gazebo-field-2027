# 坐标与规划接口

v5 障碍位置唯一来源：`config/random_generation.yaml` 的 `candidate_slots.slots`；当前实际选择和可供后续 A* 读取的圆柱信息在 `maps/random_seeded_obstacles.json`。其中 `slot_id` 与 `entity_name` 一一对应，模型原点在底面 z=0，物理圆柱中心 z=0.05。调试编号没有碰撞，不写入任何占用地图。原有生产规划器未修改，必须显式读取此 JSON 或对应 PGM 才能使用新抽签布局。详见 [候选抽签说明](randomization.md)。

已只读检查已有 `navigation_tracking_20261102/src/path_planning/maps/field.yaml`、`src/field_map.cpp`、`src/planner_node.cpp`：原点在场地左下角 (0,0)，X 向右、Y 向上，范围 [0,2.4] × [0,2.4]。本场地与其保持一致，`world -> map` 为零平移、零旋转。场地中心为 (1.2,1.2,0)。若外部使用中心坐标，转换为本坐标需加 (1.2,1.2,0)，不能同时发布两个 map 父变换。

## 输出图层

- `*_transit.yaml/pgm`：普通行驶，蓝色区禁用，黄色/工位/圆柱占用，原料装置允许摆放范围为 unknown。
- `*_start_1.yaml/pgm`、`*_start_2.yaml/pgm`：仅开放指定一个启停区。任务管理器只能在该区的出发或返回阶段启用；整个底盘离开后切回 transit，禁止以启停区绕行。地图本身不能判断任务阶段，也不实现规则状态机。
- `*_physical_nominal.yaml/pgm`：已知名义静态实体投影。黄色、蓝色可物理行驶，与规则禁行分开。它不是雷达观测，也不是本地动态地图，不能据此清空未知障碍或覆盖 `/perception/local_costmap`。
- `*_semantic.pgm`：原始标签 ID 图，必须按 `metadata.json` 解释，不能直接交给 Nav2 当占用图。

版本 0.2 加入二维码板的真实厚度投影（语义 ID 8），按板位姿、宽度和厚度自动计算；在规则图与实体图中均占用。任务阶段物料都在原料或工位区域内，底盘规则地图不因批次改变；任务状态沿用 competition 对应地图。自由物料若被未来机器人推到车道上，应交给动态观测层更新，不把其初始位置固化成整场永久障碍。

分辨率 0.005 m，480 × 480，YAML 原点 [0,0,0]。PGM 行从上至下，OccupancyGrid 从左下起逐行；图像像素 (col,row) 中心对应 x=(col+0.5)r，y=(480-row-0.5)r。地图边缘以外应视为不可通行，不能默认自由。

规则图 PGM 0→占用 100，254→自由 0，205→未知 -1。圆柱按与像素方格相交保守栅格化，最大边界差为一个栅格；机器人外形膨胀由后续规划器处理，本包不预先把 300 mm 膨胀写入基础地图。中心十字宽 80 格。

## 接入现有 path_planning

已有规划器通过 `map.layout=competition_image` 使用内置矩形，**不会自动读取新 PGM/YAML**，也没有地图文件参数。本包提供同一几何源生成的 `config/planner_static_adapter.yaml`，采用已有的 `map.layout=custom`、`map.regions` 接口。它包含名义固定规则区域和保守原料范围，默认关闭两个蓝区；障碍栅格通过后续的静态地图读取适配或真实本地观测接入。不能只复制新 PGM 却仍保留旧 `competition_image` 配置。

适配配置用于仿真隔离命名空间，`map.confirmed=false`；它不声称现场坐标、TF 或机器人外形已确认，不修改现有导航工程，不启动规划节点，也不解除原规划器的传感器/坐标确认保护。若后续接入静态占用图，使用 nav_msgs/OccupancyGrid，frame_id=map，原点四元数 w=1，按上述 Y 翻转读取，不把规则图误当作实物传感器图层。
