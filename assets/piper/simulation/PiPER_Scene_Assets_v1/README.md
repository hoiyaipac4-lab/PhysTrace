> 匿名审稿包说明：本页保留原始归档说明；其中 Blender 工程属于私人原始档案，审稿副本提供对应的导出模型。完整收录范围以仓库根目录的清单为准。

# PiPER 场景与资产包

## 完整场景
Isaac Sim 打开 `scene/scene.usda`。Windows 交互调试运行 `scene/启动调试.cmd`；默认 Isaac Sim 6.0.1，其他安装位置请设置 ISAAC_SIM_DIR。

当前摆放：机械臂在黑色固定板左侧大开口，两只夹具在底座左右，固定板距桌面右侧 8 mm。全局相机仍在修复，未包含；Camera 为调试观察视角。

## 单独使用的资产
- 机械臂：`scene/robot/piper/piper.usda`，与其 payloads、Textures 文件夹一起保留。
- 升降桌：`scene/desk/lift_table.usda`，保留同目录依赖。
- 黑色固定板：`assets/Black_Mounting_Plate/robot_base_rebuilt.blend` 和 OBJ/MTL。源坐标数值单位按毫米估计；场景内已转换为米。
- 最新独立黑色夹具：`assets/Clamp_Final_Black_v1/IsaacSim/Clamp.usdc`，另附该定稿的可编辑模型、报告与参数。
- 旧房间可复用资产：`scene/room/assets/` 下门、背景墙及地板、灯、沙发，各自的 asset.usda。

资产可以分别引用，不要单独挪走 USD 而遗失同目录依赖。完整场景中的夹具是固定安装表示，独立夹具保留原定稿结构。

本包是装配调试版本。已完成局部升降和关节动作检查，不代表完整运动范围、实际摩擦夹紧或承载验证。详细限制及检查数据见 `scene/调试说明.md` 与 `scene/evidence/`。
