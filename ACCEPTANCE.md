# R1～R13 验收摘要 v0.5.0

`ubuntu-22.04` 分支提示：下面是原 Ubuntu 24.04/Jazzy 场地验收基线，不能作为 Ubuntu 22.04/Humble 的实机运行记录。本分支的适配内容、当前验证范围与安装步骤见 [Ubuntu 22.04 说明](gongchuang_gazebo_ws/src/gongchuang_gazebo_world/docs/ubuntu22.04.md)。

执行日期：2026-11-03。环境：Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 8.15.0。截图均为实际 Gazebo 相机输出；SVG 为配置坐标图。

## 候选坐标

world/map 左下角原点，X 向右、Y 向上，单位 m。全部坐标是参考图对齐的待标定值。

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


中央五点：北 R10、西 R11、中 R13、东 R9、南 R12。生成前校验全部 13 位的完整圆柱占地，冲突时报告编号，不移动、删除或跳过候选位置。

## 实测结果

|项目|结果|
|---|---|
|单元/几何测试|32 通过|
|CTest|2 测试，0 错误，0 失败|
|物理基础回归|10 个 world 的 SDF、时钟及实时位姿通过|
|转盘/物料/码垛|6 种状态通过；正反转、停留、随盘摩擦、同色双层码垛|
|位姿统一|7 组截图场景实体与统一配置一致，XY 容差 1e-8 m|
|真实碰撞|13 位分别落下动态试件，中心 z≈0.11 m；空白对照≈0.01 m，容差 2 mm|
|1000 种子|2000 次位置选择，0 次单场重复，全部 78 种双障碍组合|
|任务可达性|896 PATH、104 NO_PATH；普通模式保留无解布局|
|二维码|真实相机成功解出 456+312+654+321|
|既有设施|42 个原模型/配置及相关源码文件哈希保持一致|

默认指定任务为 raw_material=(1.2,2.1) → coarse_slot_2=(1.2,0.38)。PATH 证书只证明这个配置任务，不等于全流程搬运保证。车体 300 × 300 mm，10 mm 安全裕量、10 mm 跟踪预算、±3° 航向包络。蓝区名义出入检查单列，不纳入稳健灰区任务证书。

[1000 种子完整统计](evidence/slots_v5/statistics/distribution_1000.json) · [32 项测试日志](evidence/slots_v5/unit_tests.log) · [物理接触结果](evidence/slots_v5/gazebo/results.json)

## 实际运行截图

|场景|截图|
|---|---|
|默认两障碍 R1、R13|[俯视图](evidence/slots_v5/gazebo/default_2/top.png)|
|13 位编号及抽中实体|[调试图](evidence/slots_v5/gazebo/slots_debug/top.png)|
|仅 13 个标记、0 障碍|[全部候选图](evidence/slots_v5/gazebo/slots_all/top.png)|
|中央 R9～R13|[中央图](evidence/slots_v5/gazebo/central_slots/top.png)|
|手动 R2、R7、R10|[手动图](evidence/slots_v5/gazebo/manual_slots/top.png)|
|随机 3 障碍压力示例|[压力图](evidence/slots_v5/gazebo/stress_slots/top.png)|
|R4、R10、R1 阻断主要通路|[阻断图](evidence/slots_v5/gazebo/blocked_slots/top.png)|

普通场景没有候选调试模型，调试模型没有 collision。坐标、地图、实体和生成记录共用配置，未修改既有生产导航工程。参考图不足以确认官方绝对坐标，因此保留未标定声明。
