# PiPER URDF 来源与合理性核查

## 结论

当前模型是**基于松灵 AgileX 官方 PiPER URDF 修改的场景版本**，不是原封不动的厂商文件。`piper_scene_v4.zip` 的 V4 指资产包修订版，不能据此称为厂商硬件“PiPER V4”。

对照官方仓库 `agilexrobotics/piper_ros`，固定提交 `ac41fcbcdda598f01b51cf6175ed9a24d0dacadc`：

- 8 个运动关节的原点、轴向、上下限、effort 与 velocity 均与 `piper_description.urdf` 一致。
- base_link、link1–link8 的原始 URDF 质量、质心、惯量与官方一致。
- 原始 10 个机器人碰撞网格的三角面数量与官方 STL 一致；双向最大顶点距离小于 8.5×10⁻⁹ m，属于格式转换的浮点误差。
- gripper_base 被修改：质量从 0.45 kg 增至约 0.57 kg；质心与惯量同时调整；新增腕部相机与支架的视觉／碰撞网格。
- 增加了 gripper_base 局部 Z +90 mm 的无质量 TCP 坐标系；该 TCP 不是已验证的官方／实机标定结果。
- 外观被替换为细分扫描网格和贴图；不能把扫描外观与厂商原始 CAD 外观混称为同一个文件。

## “九个刚体惯量主轴反了”是否真实存在

**存在于原先的 USD 场景，原始 URDF 没有这个问题。**

USD 保存了惯量的三个主值和主轴旋转四元数。原版本主值正确，但旋转方向用了逆方向：按 USD 约定重建的 `R D Rᵀ` 与 URDF 的惯量矩阵不一致，换用逆旋转后才一致。已在覆盖层中修正九个物理刚体的主轴四元数，没有改变原始 URDF。

修复后既比对了保存的 USD，也读取了 **PhysX 运行时真实惯量矩阵**。九个刚体中最大绝对分量误差约 `9.25e-9 kg·m²`；文件侧最大误差约 `1.59e-9 kg·m²`。这不是“凭外观判断”，也不是认定厂商模型错误。

此外，joint7/8 的线速度曾错误套用弧度转角度系数，变成 57.29578 m/s；现已恢复官方 URDF 的 1 m/s。关节几何和限位保持原值。

证据：`evidence/urdf_comparison.json`、`evidence/runtime_inertia_comparison.json`、`evidence/vendor/comparison.json`、`evidence/vendor/mesh_comparison.json`。

## 修改是否合理

由修改前后质量和质心反推，新增部件质量约 0.12 kg，其合成质心在 gripper_base 局部坐标约 `[-0.06823, -0.02879, 0.02807] m`，位于相机／支架区域。反推附加惯量为正定，满足刚体惯量三角不等式。因此**数学上自洽、几何位置合理**。

但没有找到这 120 g 的称重和惯量实测记录，不能把“合理估计”说成“真实标定”。夹具、支架弹簧、桌子和电器的部分物性也是源包工程估计。仿真通过不证明它们与实物力学完全一致。

## 固件匹配

官方说明：`S-V1.6-3` 之前应使用 `piper_description_old.urdf`；该版本及更新固件使用 `piper_description.urdf`。主要区别包括关节 2、3 的零位约定相差约 2°。

当前资产与后一版本一致。用户确认目前不知道实机固件，并要求继续仿真核查。因此**实机固件匹配仍未验证**，没有发任何实机控制命令。

官方来源：
- https://github.com/agilexrobotics/piper_ros/blob/ac41fcbcdda598f01b51cf6175ed9a24d0dacadc/src/piper_description/urdf/piper_description.urdf
- https://github.com/agilexrobotics/piper_ros/blob/noetic/README(EN).md
- USD 惯量定义：https://openusd.org/25.11/api/class_usd_physics_mass_a_p_i.html

## 可移植文件

`robot_description/piper.urdf`：V4 的机械定义，网格和贴图引用改为包内相对路径。
`robot_description/piper.source.urdf`：V4 原文件逐字节保留。
`robot_description/piper.pre_v4.source.urdf`：前一版本原文件。
`robot_description/provenance.json`：来源与哈希说明。

这些源文件与修复后的场景覆盖层分开保存，方便逐项复核。

## 仿真运动一致性

独立使用 URDF 正向运动学，按 Isaac Sim 实际读回的 100 组关节状态计算 link1–link8 位姿，再与原生物理引擎连杆位姿比较：最大位置误差约 3.30e-7 m（0.00033 mm），最大旋转误差约 9.42e-7 rad（0.000054°），属于数值误差量级。**这证明本次轨迹内仿真模型与 URDF 的运动学一致，不证明与真实硬件标定一致。**

本机 USB-CAN 已接通，接口 can0、1 Mbps。三个用户手动改变后的静止姿态，URDF 与控制器计算末端的位置差分别为 0.0793、0.0769、0.1076 mm，姿态差均低于 0.005°；旧版 URDF 对应位置差约 10 mm。证据位于 evidence/hardware/。这是控制器内部运动学模型的一致性证据，不是外部实测定位精度。固件版本字符串仍未读出，TCP、手眼和基座坐标等尚未实测标定。全部采集只接收，未发送控制帧。
