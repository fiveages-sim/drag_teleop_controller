# drag_teleop_controller

**主从拖动遥操作控制器包**（Panthera HT 双臂）。

一个控制器插件同时支持 `master`（操作者拖动、重力补偿）与 `slave`（跟随主臂）
两种角色，通过 `role` 启动参数区分。支持 `position` / `mix` / `effort` 三种
控制模式与 `position` / `effort` 两种力反馈（仅 master）。关节映射在发布
`teleop_states` 前完成；夹爪由 `adaptive_gripper_controller` 控制（launch
自动 spawn，配置与 ocs2 启动一致）。

## 架构

```
┌─ master 进程（role:=master）─────────────┐   ┌─ slave 进程（role:=slave）─────────────┐
│ DragTeleopController (500Hz)             │   │ DragTeleopController (500Hz)             │
│  读硬件状态（12 臂 + 2 夹爪状态）          │   │  读硬件状态（12 臂 + 2 夹爪状态）          │
│  τ_G = rnea(q, 0, 0)                     │   │  τ_model = M a + C v + G（q̈ 数值微分）    │
│  订阅 /drag_teleop_slave/teleop_states   │   │  订阅 /drag_teleop_master/teleop_states   │
│  计算 q_cmd / τ_cmd（mode × feedback）    │   │  计算 q_cmd / τ_cmd（mode，ruckig 可选）   │
│  发布 /drag_teleop_master/teleop_states  │   │  发布 /drag_teleop_slave/teleop_states    │
│    （正映射：master→slave 参考系）         │   │    （逆映射：slave→master 参考系）         │
│  moveJ_pub:=true 时发布 ocs2 moveJ 命令   │   │                                          │
└──────────────────────────────────────────┘   └──────────────────────────────────────────┘
```

- **发布前映射**：master 发布正映射（slave 关节名），slave 发布逆映射（master
  关节名），接收方直接使用；effort 按功率守恒映射 `τ_master = sign·scale·τ_slave`。
- **夹爪**：12 臂关节由本控制器控制；夹爪由 `left/right_gripper_controller`
  （`adaptive_gripper_controller`）控制，只做位置映射（master 侧
  `moveJ_pub:=true` 时发布映射后的夹爪位置到 `/<joint>/position_command`）。
- **模式切换**：`/drag_teleop_{role}/teleop_mode`（position|mix|effort）、
  `/drag_teleop_{role}/teleop_feedback`（false|position|effort，仅 master）
  自定义服务，运行时可切换。

## 控制模式 `mode`

| 模式 | 说明 | hardware 需求 |
|------|------|---------------|
| `position` | 仅位置控制，无力反馈（**仅 slave 可用**） | hardware 只需 position 命令接口 |
| `mix` | 同时下发 position / velocity / effort 三种命令，由 hardware 内部混合并计算最终关节力矩；力反馈使用 $q_m$、$q_s$ 的误差实现 | hardware 需 position + velocity + effort 三种命令接口 |
| `effort` | 控制器直接下发 effort | hardware 需 effort 命令接口；对 mix 型 hardware，可在启动参数指定 `hardware_joint_kp/kd:=0` 关闭位置反馈，从而实现 effort 控制 |

### role × mode 的 hardware 需求

| role | mode | position 命令 | velocity 命令 | effort 命令 |
|------|------|:---:|:---:|:---:|
| master | mix | 仅开启力反馈时写入（当前 + Δq） | 无（写 0） | 重力补偿力矩 |
| master | effort | 保位（当前值） | 无（写 0） | 重力补偿力矩 + 力反馈力矩 |
| slave | position | 目标位置（主臂） | — | — |
| slave | mix | 目标位置（主臂） | 目标速度（主臂） | 重力补偿力矩 |
| slave | effort | 保位（当前值） | 无（写 0） | 重力补偿力矩 + 位置速度跟踪力矩 |

> 控制器在 `on_activate` 时按所选 `mode` 校验所需命令接口，缺失则激活失败。

## 反馈类型 `feedback`（仅 master 生效）

| 主臂 mode | 从臂 mode | 可用反馈 |
|-----------|-----------|----------|
| mix | position | false、position |
| mix | mix | false、position |
| mix | effort | false、position |
| effort | position | false、position、effort（仅当从臂存在 effort 状态接口） |
| effort | mix | false、position |
| effort | effort | false、position、effort（仅当从臂存在 effort 状态接口） |

- `false` / `none`：无力反馈
- `position`：基于位置误差的力反馈（$\Delta q = -G \cdot \text{sat}(q_m - q_s - \text{dead\_zone})$）
- `effort`：基于从臂外部力矩的力反馈（$\tau_{cmd} = -G \cdot \tau_{ext,slave} + \tau_G$）

## 控制律

### 从臂（slave）

**外部力矩估计**（三种模式共用）：

$$\tau_{model,slave} = M(q)\ddot q + C(q, \dot q)\dot q + G(q)$$

$$\tau_{ext,slave} = \tau_{state,slave} - \tau_{model,slave}$$

> 若从臂不存在 effort 状态接口，令 $\tau_{state,slave} = \tau_{cmd,slave}$。
> $\ddot q$ 由速度状态数值微分 + 低通滤波得到。

**slave-position**：

$$q_{cmd,slave} = q_{state,master}$$

**slave-mix**：

$$q_{cmd,slave} = q_{state,master}, \qquad \dot q_{cmd,slave} = \dot q_{state,master}$$

$$\tau_{cmd,slave} = \tau_G(q_{cmd,slave})$$

**slave-effort**：

$$\tau_{imp,slave} = K_p (q_{state,master} - q_{state,slave}) + K_d (\dot q_{state,master} - \dot q_{state,slave})$$

$$\tau_{cmd,slave} = \tau_{imp,slave} + \tau_G(q_{cmd,slave})$$

### 主臂（master）

**master-\*-feedback(false)**（无力反馈）：

$$q_{cmd,master} = q_{state,master}, \qquad \dot q_{cmd,master} = 0$$

$$\tau_{cmd,master} = \tau_G(q_{state,master})$$

**master-mix-feedback(position)**（位置偏移，由硬件位置环产生反馈力）：

$$\Delta q_{feedback,master} = -G \cdot (q_{state,master} - q_{state,slave})$$

$$q_{cmd,master} = q_{state,master} + \Delta q_{feedback,master}, \qquad \dot q_{cmd,master} = 0$$

$$\tau_{cmd,master} = \tau_G(q_{state,master})$$

**master-effort-feedback(position)**（阻抗力矩）：

$$q_{cmd,master} = -G \cdot (q_{state,master} - q_{state,slave})$$

$$\tau_{imp,master} = K_p (q_{cmd,master} - q_{state,slave})$$

$$\tau_{cmd,master} = \tau_{imp,master} + \tau_G(q_{state,master})$$

**master-effort-feedback(effort)**（外部力矩反馈）：

$$\tau_{cmd,master} = -G \cdot \tau_{ext,slave} + \tau_G(q_{state,master})$$

> 符号约定：$q_{state,master}$ / $q_{state,slave}$ 为映射到同一参考系后的关节
> 状态（发布前已映射，接收方直接使用）；$G$ 为反馈增益（`master.feedback.*.gain`），
> 误差经死区（`dead_zone`）与限幅（`max_delta_q` / `max_ext_effort`）处理；
> $K_p$ / $K_d$ 为阻抗参数（`slave.control.effort.kp/kd`）。

## 编译

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drag_teleop_controller --symlink-install
```

## 使用

### Mock 验证（无需硬件）

```bash
# 终端 1：master
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
  role:=master hardware:=mock_components

# 终端 2：slave
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
  role:=slave hardware:=mock_components
```

检查控制器状态与话题：

```bash
ros2 control list_controllers --controller-manager /drag_teleop_master/controller_manager
# drag_teleop_controller / left_gripper_controller / right_gripper_controller 应 active

ros2 topic echo /drag_teleop_master/teleop_states
ros2 topic echo /drag_teleop_slave/teleop_states
```

### 真机

```bash
# master（低刚度拖动 + 位置力反馈 + 发布 ocs2 moveJ 命令）
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
  role:=master hardware:=real mode:=effort feedback:=position moveJ_pub:=false \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.1, 0.1, 0.1, 0.1, 0.1, 0.1" \
  hardware_gripper_kp:="0.01" hardware_gripper_kd:=""0.1

# slave（effort 模式跟随）
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
  role:=slave hardware:=real mode:=effort \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.1, 0.1, 0.1, 0.1, 0.1, 0.1" 
```

### 模式切换

```bash
ros2 service call /drag_teleop_master/teleop_mode \
  drag_teleop_controller/srv/TeleopMode "{mode: effort}"
ros2 service call /drag_teleop_master/teleop_feedback \
  drag_teleop_controller/srv/TeleopFeedback "{mode: position}"
```

### 自定义控制器参数文件

```bash
ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
  role:=slave controller_params:=/path/to/my.yaml
```

## 启动参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `role` | （必选） | `master` \| `slave` |
| `robot` | `panthera_ht` | 机器人名（自动寻找 `{robot}_description` 包） |
| `type` | `dual` | `single` \| `left` \| `right` \| `dual` |
| `hardware` | `mock_components` | `real` \| `real_usb` \| `mock_components` \| `gz` \| `isaac` |
| `mode` | （yaml） | `position`（仅 slave）\| `mix` \| `effort` |
| `feedback` | （yaml） | `false` \| `position` \| `effort`（仅 master） |
| `input_topic` | `auto` | 对侧状态话题；auto = `/drag_teleop_{对侧}/teleop_states` |
| `moveJ_pub` | `false` | 发布 ocs2 moveJ + 夹爪命令（仅 master） |
| `use_sim_time` | `false` | 仿真时间 |
| `namespace` | `/drag_teleop` | 基础命名空间，与 role 拼接为 `/drag_teleop_{role}` |
| `rviz` | `false` | 启动 RViz |
| `controller_params` | 本包 `config/ros2_control.yaml` | 控制器参数配置文件（机器人专用软连接如 `panthera_ht_2_panthera_ht.yaml` 亦可） |
| `xacro_*` / `hardware_*` | — | xacro 参数透传（`hardware_*` 仅 `hardware:=real` 生效） |

## 代码结构

| 文件 | 职责 |
|------|------|
| `src/drag_teleop_controller.cpp` | 控制器生命周期、接口 claim、update 编排、服务回调 |
| `src/joint_mapper.cpp` | 关节映射/逆映射/力矩映射（纯算法） |
| `src/dynamics.cpp` | Pinocchio 动力学（τ_G、τ_model、q̈ 数值微分） |
| `src/control_calculator.cpp` | 控制量计算（role × mode × feedback 纯函数） |
| `src/ocs2_publisher.cpp` | ocs2 moveJ + 夹爪命令发布（master + moveJ_pub） |
| `srv/TeleopMode.srv` / `TeleopFeedback.srv` | 模式切换服务 |
| `config/ros2_control.yaml` | 默认控制器参数（master/slave 共用一段 + 夹爪控制器）；`panthera_ht_2_panthera_ht.yaml` 为指向它的软连接 |
| `launch/drag_teleop_controller.launch.py` | 启动（role 参数） |

## 依赖

- ROS2: `controller_interface` `hardware_interface` `pluginlib` `rclcpp`
  `rclcpp_lifecycle` `sensor_msgs` `std_msgs` `ruckig` `rosidl_default_generators`
- 动力学: `pinocchio`（`ros-jazzy-pinocchio`）`urdf`
- 运行时: `controller_manager` `joint_state_broadcaster` `robot_state_publisher`
  `xacro` `adaptive_gripper_controller`

d