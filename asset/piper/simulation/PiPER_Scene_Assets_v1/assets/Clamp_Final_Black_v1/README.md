> 匿名审稿包说明：本页保留原始归档说明；其中 Blender 工程属于私人原始档案，审稿副本提供对应的导出模型。完整收录范围以仓库根目录的清单为准。

# 机械臂零件 · 黑色定稿 v1

扫描形状为 C 形螺旋夹具，四件独立重建，黑色外观。已完成报告所列条件下的 Isaac Sim 实际运行验收。

- **Isaac Sim 打开入口：** [Isaac_CheckScene.usda](IsaacSim/Isaac_CheckScene.usda)
- **可复用仿真资产：** [Clamp.usdc](IsaacSim/Clamp.usdc)
- **可编辑模型：** [Black_Clamp.blend](模型/Black_Clamp.blend)
- **通用模型：** [Black_Clamp.obj](模型/Black_Clamp.obj)，同目录材质文件需一起保留。
- **验收结果与使用条件：** [验收报告](验收报告.md)
- **扫描对比：** [几何与偏差报告](检查报告/几何与偏差报告.md)

![Isaac 实测姿态](预览/Isaac实测姿态.png)

建议保留整个目录结构。场景按 960 Hz 配置，含检查用固定夹具；需要自由物体时引用独立 Clamp.usdc。尺寸和物性为估计，内部连接使用等效关节；尚未作实物标定和机器人训练任务认证。
