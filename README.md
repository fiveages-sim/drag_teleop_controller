# drag_teleop_controller

**机械臂拖动遥操作控制器包**，基于 **Pinocchio RNEA** 计算静态重力矩。



## 工作原理

```
┌──────────────────────────── ros2_control ────────────────────────────┐
│                                                                      │
│        DragTeleopController (本包, 500Hz update)                     │
│    ┌──────────────────────────────────────────────┐                  │
│    │ 读 position 状态 → 组装 Pinocchio q（按名称）│                  │
│    │ rnea(q, v=0, a=0) → 静态重力矩 τ_g           │                  │
│    │ 写入命令接口（均可选，缺失自动跳过）:        │                  │
│    │   effort  = clamp(τ_g)      ← 重力前馈       │                  │
│    │   position = 当前值          ← 保位（关键）  │                  │
│    │   velocity = 0               ← 速度前馈清零  │                  │
│    └──────────────────────────────────────────────┘                  │
│                          │ command interfaces                        │
│                          ▼                                           │
│  ht_ros2_control / PantheraHardwareInterface (复用现有包，零改动)    │
│    full_control: pos_vel_tqe_kp_kd() → 电机                          │
└──────────────────────────────────────────────────────────────────────┘
```

- 动力学模型：从 `/robot_description`（URDF 参数）用 `pinocchio::urdf::buildModelFromXML` 构建。
- 重力矩：`pinocchio::rnea`（零速度、零加速度），即纯静态重力补偿。
- **保位语义**：硬件接口在 `full_control` 下会把 position 命令直接下发电机，
  因此控制器每个周期把 `position` 写为**当前实测值**，防止未写 position 时被当作 0 目标（回零）。
- **命令接口可选**：`command_interface_configuration()` 返回 `ALL`（只 claim 硬件实际
  导出的接口），`on_activate` 按名挑选 `position`/`velocity`/`effort`：
  - `position` 有 → 写当前值；无 → 跳过（警告）
  - `velocity` 有 → 写 0；无 → 跳过（警告）
  - `effort` 有 → 写重力矩；无 → 警告一次，控制器以位置保持模式继续运行（不退出）
- 夹爪（`hold_joints`）：`position` 状态必需、命令可选，位置跟随当前值，保持不动作。
- 模型中没有状态来源的关节（如 mimic 的 `gripper_joint2`）q 置 0，对重力矩影响可忽略。

## 代码结构（解耦）

| 文件 | 职责 |
|---|---|
| `src/drag_teleop_controller.cpp` | 控制器生命周期（on_init/configure/activate/update）、接口 claim、重力补偿与力反馈的组装 |
| `src/gravity_compensation.cpp` + `include/.../gravity_compensation.hpp` | Pinocchio 静态重力矩（URDF → 模型 → RNEA） |
| `src/force_feedback.cpp` + `include/.../force_feedback.hpp` | 位置弹簧力反馈：订阅 mapper 逆映射、Δq 计算、kp 状态机线程（独立节点 + SyncParametersClient） |

控制器只持有 `std::unique_ptr<GravityCompensation>` 与 `std::unique_ptr<ForceFeedback>`，
`update()` 中分别调用 `calculateStaticTorques(q)` 与 `update(dt, state_values)`，
力反馈的 Δq 叠加到 position 命令（`q_master + Δq`）。

## 依赖

- ROS2: `controller_interface` `hardware_interface` `pluginlib` `rclcpp` `rclcpp_lifecycle`
- 动力学: `pinocchio`（`ros-jazzy-pinocchio`）`urdf`
- 运行时: `controller_manager` `joint_state_broadcaster` `robot_state_publisher` `xacro`

## 编译

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drag_teleop_controller --symlink-install
```

## 使用

### Mock 验证（无需硬件/电机）

```bash
source install/setup.bash
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py
# 默认 hardware:=mock_components、type:=dual
```

检查控制器状态：

```bash
ros2 control list_controllers --controller-manager /drag_teleop/controller_manager
# drag_teleop_controller 应显示 active
```

### 真机双臂

```bash
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py hardware:=real
```

- 单臂/单侧：`type:=single` / `type:=left` / `type:=right`（URDF 由 xacro 按 type 展开）。


### 低刚度（拖动）模式

kp/kd 是硬件接口参数，通过 `hardware_` 前缀启动参数设置（仅 `hardware:=real`
生效；每臂 6 值 CSV，dual 时 xacro 自动拼接为 12 值）：

```bash
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py hardware:=real \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.2, 0.2, 0.2, 0.2, 0.2, 0.2" \
  hardware_gripper_kp:=0.001 hardware_gripper_kd:=0.01
```

kp/kd 经 xacro 写入 URDF hardware 段，硬件接口加载即生效（无需启动后调参数服务器）。

### 力反馈（从臂碰撞回推主臂）

当从臂碰到障碍物无法跟随主臂时，主臂会向操作者施加反向阻力。原理是
**位置弹簧**：控制器订阅 `teleop-joint-mapper` 逆映射发布的期望位置
（主臂参考系下的从臂实际位置 `q_ref`，话题 `feedback.joint_state_topic`，
默认 `/joint_mapper/feedback_joint_state`），计算误差

$$e_i = q_{master,i} - q_{ref,i}$$

超过死区后把主臂位置命令反向偏移（限幅 + 斜坡平滑）：

$$\Delta q_i = -G\cdot\text{sat}(e_i - \delta),\qquad q_{cmd,i} = q_{master,i} + \Delta q_i$$

反馈力矩由电机位置环自动产生：$\tau_{fb} = kp_{fb}\cdot\Delta q_i$。由于拖动模式
kp≈0.01 时力矩放大率太低（2Nm 需 Δq=200rad），反馈激活期间控制器会经
`/controller_manager` 参数服务把硬件 `joint_kp` **临时提升**到 `feedback.kp`
（默认 4.0），退出反馈自动恢复原值（硬件 ~200ms 刷新生效）。

反馈激活条件（同时满足，否则 Δq=0 且 kp 保持原值）：

1. `feedback.enabled: true`（默认）；
2. 反馈话题在 `feedback.input_timeout`（默认 0.2s）内有新数据——
   mapper 仅在从臂 FSM 处于 MOVEJ 时发布，因此**从臂停止跟随会自动解除反馈**。

启动：

```bash
# 与拖动模式相同，默认 feedback.enabled=true（yaml 配置）
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py hardware:=real \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.2, 0.2, 0.2, 0.2, 0.2, 0.2"

# 快速关闭（不重启从臂链路）
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py feedback:=false
```

调参建议：`feedback.kp` 决定反馈力大小（kp×Δq，kp=4 且 Δq 满偏 0.5rad 时约 2Nm，
远小于 21/36Nm 限幅）；`feedback.gain` 决定误差→Δq 的灵敏度；`feedback.deadzone`
吸收主从零点偏置（主臂自然悬停误差，避免常驻阻力）；`feedback.delta_q_rate`
限制 Δq 变化速率，防止误差突变时猛推。

调试话题（控制器 ns 内，如 `/drag_teleop/feedback_delta_q_debug`，Float64MultiArray）：

```bash
ros2 topic echo /drag_teleop/feedback_delta_q_debug  # 实时查看 Δq（力矩 = 当前 kp × Δq）
ros2 param get /drag_teleop/controller_manager joint_kp  # 确认反馈激活时 kp 已提升
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `type` | `dual` | 臂组合：`single` / `left` / `right` / `dual` |
| `hardware` | `mock_components` | `real` / `real_usb`（ht_ros2_control）\| `mock_components` \| `gz` \| `isaac` |
| `namespace` | `drag_controller` | 所有节点（RSP/CM/RViz）的 ROS namespace；`/` = 全局 |
| `rviz` | `false` | 是否启动 RViz2（默认不启动） |
| `feedback` | `true` | 覆盖 `feedback.enabled`：`true`/`false`；空 = 使用 yaml 值 |
| `description_package` | `panthera_ht_description` | 描述包：URDF 与关节接口均从该包解析 |

#### 前缀参数（与 ocs2 demo.launch.py 同格式，经 `build_xacro_mappings` 透传）

| 前缀 | 生效条件 | 示例 |
|---|---|---|
| `xacro_xxx:=` | 总是生效 | `xacro_control_mode:=pd_control`、`xacro_config_file:=/path/PantheraDual.yaml`、`xacro_usb_select:=usb-0:1.2` |
| `hardware_xxx:=` | 仅 `hardware:=real/real_usb` | `hardware_joint_kp:=`、`hardware_config_file:=` |
| `robot_profile:=` | 提供 `hardware:`/`xacro:` 段（robot.local.yaml） | `robot_profile:=/path/robot.local.yaml` |

kp/kd/control_mode 等硬件参数**没有专用 launch 参数**，统一走前缀参数
（`hardware_` 真机 / `xacro_` 通用），默认值由描述包 xacro 决定。

### 关节信息自动解析（不再硬编码）

`ARM_JOINTS` / `GRIPPER_JOINT` / `ARM_MAX_EFFORT` 从描述包的
`xacro/ros2_control/interfaces.xacro` 动态解析：

- 关节名：`<joint name="${prefix}jointN">` → 去掉 `${prefix}` 得到裸名；
- 夹爪关节：名字含 `gripper` 的关节；
- 力矩限幅：`<param name="max_torque">` 数值（按关节顺序）；
- 解析失败时回退到内置默认值（`joint1..6` / `gripper_joint` / `[21,36,36,21,10,10]`）。

### RViz

```bash
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py rviz:=true
# 可选：rviz:=true namespace:=my_ns
```

RViz 与 RSP/CM 处于同一 namespace，配置（`config/drag_teleop.rviz`）使用
相对话题名（`tf`、`robot_description`），固定坐标系为 `world`。

### 参数（`config/controller_params.yaml`）

| 参数 | 说明 | 默认（双臂） |
|---|---|---|
| `joints` | 重力补偿关节（臂关节） | 12 个 `left/right_joint1..6` |
| `hold_joints` | 仅位置保持关节 | `left/right_gripper_joint` |
| `max_effort` | 力矩限幅 Nm（空 = 不限幅） | `[21,36,36,21,10,10]×2` |
| `gravity_vector` | 世界系重力加速度 | `[0, 0, -9.81]` |
| `urdf_param` | URDF 来源参数 | `robot_description` |

#### 力反馈参数（`feedback.*` 嵌套段）

| 参数 | 说明 | 默认值 |
|---|---|---|
| `feedback.enabled` | 是否启用力反馈 | `true` |
| `feedback.joint_state_topic` | mapper 逆映射话题（绝对名） | `/joint_mapper/feedback_joint_state` |
| `feedback.input_timeout` | 反馈数据超时（s），超时自动关闭反馈 | `0.2` |
| `feedback.gain` | Δq = −G·(e−δ) | `0.3` |
| `feedback.deadzone` | 误差死区（rad） | `0.02` |
| `feedback.kp` | 反馈激活期间硬件 joint_kp 临时值 | `4.0` |
| `feedback.kd` | 反馈激活期间 joint_kd 临时值（≤0 不改） | `0.0` |
| `feedback.kp_param_name` | 硬件 kp 参数名（不同 hardware 可能不同） | `joint_kp` |
| `feedback.kd_param_name` | 硬件 kd 参数名 | `joint_kd` |
| `feedback.max_delta_q` | Δq 限幅（rad），力矩上限 = kp×max_delta_q | `0.5` |
| `feedback.delta_q_rate` | Δq 斜坡速率（rad/s） | `2.0` |

> kp/kd 由硬件接口（`ht_ros2_control`）作为 ROS 参数管理（rqt 可调），
> 本控制器不写 kp/kd 命令接口，因此无 `use_pd`/`hold_kp`/`hold_kd` 参数。

## 注意事项（已确认的硬件行为）

1. **kp/kd 不能为 0**：硬件接口 `full_control` 分支在 `kp <= 0` 时会回退到
   `interfaces.xacro` 的默认增益（20~40）。如需"更软"请给正小值（如 0.5~5）。
2. **与 OCS2 控制器互斥**：`ocs2_arm_controller` 与本控制器都 claim 相同的命令接口，
   不能同时激活；可用 `ros2 control swap_controllers` 切换。
3. **双臂 `body_rpy` 非零**时，`gravity_vector` 需按安装方向旋转（默认假设竖直安装）。

## 验证（已完成）

- **编译**：`colcon build --packages-select ht_gravity_compensation` 通过（Jazzy，C++17 + pinocchio 4.0）。
- **Mock 冒烟**：`gravity_compensation.launch.py`（`hardware:=mock_components type:=dual`）下控制器
  `configure → activate` 成功，日志确认 `Configured: 12 compensated joints, 2 hold joints,
  Pinocchio model nq=16`；`/joint_states` 以 500Hz 发布；`ros2 control list_controllers` 显示 active。
- **动力学数值**（pinocchio python 对同一 URDF 在零位 RNEA）：

  | 关节 | τ_g (Nm) | 说明 |
  |---|---|---|
  | joint1 | 0.000 | 绕垂直轴，无重力分量 ✓ |
  | joint2 | 1.857 | 前臂重力负载 |
  | joint3 | 4.161 | 最大重力负载关节 |
  | joint4 | 0.977 | |
  | joint5/6 | ≈0 | 零位竖直 |
  | gripper* | 0 | 平移/对称关节 |

  左右臂对称，数值在 `max_torque` 限幅（21/36 Nm）范围内，合理。

