# R1～R13 候选抽签系统 v5

编号来自用户 2026-11-03 提供的最新参考图。坐标是依据现有场地几何对齐的待标定参考值，**不是官方给定坐标**。默认双障碍是开发测试配置，不代表官方固定数量。

唯一位置源为 config/random_generation.yaml 的 candidate_slots.slots。已有 2.4 m 场地左下角为原点，X 向右、Y 向上，world 与 map 无偏移或旋转，障碍模型的 pose.z=0，圆柱实体中心在 z=0.05 m。

|编号|X / m|Y / m|图中关系|
|---|---:|---:|---|
|R1|2.125|1.625|右上侧外围|
|R2|1.625|2.125|上侧偏右|
|R3|0.775|2.125|上侧偏左|
|R4|0.275|1.625|左上侧外围|
|R5|0.275|0.775|左下侧外围|
|R6|0.775|0.275|下侧偏左|
|R7|1.625|0.275|下侧偏右|
|R8|2.125|0.775|右下侧外围|
|R9|1.625|1.200|中央十字右侧|
|R10|1.200|1.625|中央十字上侧|
|R11|0.775|1.200|中央十字左侧|
|R12|1.200|0.775|中央十字下侧|
|R13|1.200|1.200|中央十字中心|

黄区中心投影为 0.775 / 1.625 m；十字中心为 1.200 m；外围灰色车道中心取 0.275 / 2.125 m。参考图灰色编号方块视为候选区域示意，不新增实体工位，也不以其像素边框定义官方放置范围。位置只能人工标定后修改 xy_m；全 13 点均在生成前检查完整圆柱与黄区、蓝区、固定工位、原料保守包络、二维码板和场地边界。发现冲突会报告具体 R 编号并中止，不移动、不删除、不绕过。

## 五种模式

|模式|行为|
|---|---|
|random_slots|默认；对全部 13 位无放回抽 2 位；不筛选通行性|
|solvable_slots|按种子打乱固定数量的全部有限组合，选择指定任务 A* 有路径的第一组|
|blocked_slots|从固定数量组合中选择端点未占用、灰色主要通路断开的 NO_PATH / DISCONNECTED 组合；默认 3 位|
|stress_slots|按种子抽 3～6 位；可明确指定此范围内的数量；不筛选通行性|
|manual_slots|按输入顺序精确摆放指定编号，重复或未知编号报错；空列表表示零障碍|

普通模式支持 0～13 位；手动模式数量由列表决定，若显式数量不一致则报错。过滤模式穷举有限组合，找不到满足条件的组合时明确失败，不减少数量、不扩大车道、不改变候选坐标。

抽签使用 random.Random("gongchuang-slots-v2:<seed>") 与 sample，无放回。筛选组合采用单独的 gongchuang-filter-v2 流。物料颜色/二维码继续沿用原有 gongchuang-colors-v1 流。同版本、同配置、同模式、同种子和同数量可复现相同编号、坐标、二维码与物料任务。不同种子可能重复组合：双障碍只有 C(13,2)=78 种，不能要求 1000 个种子全部不同。

## Windows 启动（原本机交付入口）

GitHub 便携版按仓库根 README 的 Linux/WSL 命令运行；下列 PS1/CMD 示例保留说明原本机交付的界面，不是本仓库提供的跨机器安装器。

双击“随机场地.cmd”生成新种子并启动；双击“启动场地.cmd”打开已生成默认场景。脚本调用本机独立 WSL 工作区 $HOME/gongchuang_gazebo_ws。

~~~powershell
.\随机场地.ps1 -Seed 2027
.\随机场地.ps1 -Seed 42 -ObstacleCount 4
.\随机场地.ps1 -Seed 2027 -Mode solvable_slots -ObstacleCount 2
.\随机场地.ps1 -Seed 2027 -Mode blocked_slots
.\随机场地.ps1 -Seed 2027 -Mode stress_slots
.\随机场地.ps1 -Seed 2027 -Mode manual_slots -Slots R2,R7,R10 -DebugSlots
.\随机场地.ps1 -Seed 2027 -Stage random_stacking_demo
.\启动场地.ps1 -Layout slots_all
.\启动场地.ps1 -Layout slots_debug
.\启动场地.ps1 -Layout central_slots
.\启动场地.ps1 -Layout manual_slots
.\启动场地.ps1 -Layout stress_slots
~~~

--preset / -Preset 保留 obstacles_0～3、stress_4～6、central_blocked，分别对应数量测试与固定阻断组合 [R4,R10,R1]。预设不能与显式模式/手动编号混用。模式筛选结果以 generated_randomization.json 为准，blocked_slots 模式和固定 central_blocked 预设不保证同样的编号顺序。

## WSL 生成与复现

~~~bash
cd ~/gongchuang_gazebo_ws
export PYTHONPATH="$PWD/.python_deps"
python3 src/gongchuang_gazebo_world/scripts/generate.py --seed 2027
python3 src/gongchuang_gazebo_world/scripts/generate.py --seed 42 --mode manual_slots --slots R2 R7 R10 --debug-slots --output /tmp/gc_manual_42
bash src/gongchuang_gazebo_world/scripts/random_scene.sh 2027 auto random_seeded --mode stress_slots
~~~

random_scene.sh 的旧三位置参数 seed count stage 继续可用，count=auto 让模式决定数量。后续参数传给 generate.py；Gazebo 参数请通过 start.sh 传入。生成到 ~/gongchuang_gazebo_ws/generated/seed_<seed>_slots_<参数摘要>/，不会覆盖默认源场景。重新生成已有输出时必须有 .gongchuang_generated 标记，防止覆盖无关目录。

## 路径检查的精确范围

本包只做离线几何规划，不向机器人发布控制。原工程使用连通性搜索与 5 mm 数值边段余量；本版按本次要求实现固定航向四邻接 A*，保留 5 mm 数值网格余量，并另加 10 mm 安全裕量、10 mm 跟踪误差预算、±3° 航向误差包络。300 × 300 mm 实体轮廓不变。

完整矩形旋转误差的保守半宽为 0.15(cos3°+sin3°)+0.01+0.01 = 0.177644 m，网格检查再加 0.005 m。A* 路径压缩后逐段做完整连续矩形扫掠，与圆柱和矩形禁区相交检测；禁止斜向穿角。NO_PATH 指当前固定航向、分辨率和误差包络下该任务没有路径，不宣称实际机器人所有运动方式均无解。

navigation_task.start / goal 选择具名操作位。默认 raw_material=(1.2,2.1) → coarse_slot_2=(1.2,0.38)。可改为 goals_and_departures 定义的 qr_board、temporary_slot_1～3、coarse_slot_1～3、start_1_port 或 start_2_port。证书仅证明明确记录的指定导航任务，不声称完成全流程搬运、机械臂抓取或全部操作位均可达。

蓝色区普通通行始终禁用，原来的“仅开放被选启停区出发/返回”地图保留。原 300 mm 蓝区恰容纳 300 mm 车体，无法同时容纳非零姿态/跟踪预算；故 nominal_blue_departures 单列保留原名义轮廓的出入检测，不纳入稳健灰区任务证书。蓝区内安全出入还需要单独的精确停靠控制，不能把名义出入结果当作带完整误差预算的保证。灰区端口 X 调整为 1.90 m，使稳健轮廓与已关闭的蓝区分离；没有移动任何 R 候选位或修改车道。

## 地图与调试

SDF 实体、PGM 圆柱栅格、JSON 生成记录、离线 A* 全部消费同一候选坐标。maps/random_seeded_obstacles.json 含 slot_id、entity_name、seed、半径相关尺寸和统一坐标，供后续规划接口读取。原生产 path_planning 不修改、不启动；其现有静态区域适配器仍只表示固定区域，接入时应使用本次障碍 JSON 或栅格，不可假设旧规划器自动读取新地图。

slots_all 仅显示 13 个编号/圆环，无真实障碍；slots_debug 显示本次抽中圆柱和全部编号；central_slots=[R9,R10,R11,R12,R13]；manual_slots=[R2,R7,R10]；stress_slots 为按种子抽出的 3～6 位示例。调试模型只有 visual，没有 collision；默认 random_seeded 场景没有调试模型。--debug-slots 可为本次选定场景及其随机物料阶段启用标记。

## 验证命令

~~~bash
python3 -m unittest discover -s src/gongchuang_gazebo_world/test -p 'test_*.py' -v
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 src/gongchuang_gazebo_world/scripts/audit_slots.py --output evidence/slots_v5/statistics
python3 src/gongchuang_gazebo_world/test/capture_obstacle_acceptance.py --output evidence/slots_v5/gazebo
python3 src/gongchuang_gazebo_world/scripts/runtime_check.py --output evidence/slots_v5/runtime
python3 src/gongchuang_gazebo_world/scripts/runtime_rules.py --output evidence/slots_v5/rules
~~~

capture_obstacle_acceptance 启动真实 Gazebo 相机、读取实时位姿，验证 7 类视图和 13 位圆柱的真实物理顶面接触。旧“连续采样且默认强制可解”测试已按新规则替换，场地尺寸、地图分层、圆环、物料、二维码及码垛断言继续保留。
