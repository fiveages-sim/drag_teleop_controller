## drag_teleop_controller 重构

### 启动参数

- `role` ：机器人角色，可选值为 `master` 或 `slave`
- `robot` :  机器人，如 `panthera_ht` ，根据该参数自动寻找 `{robot}_description` 包
- `type` ： 机器人类型，如 `dual` 、 `single` 、 `left` 、 `right`
- `hardware` ： 机器人硬件类型，如 `real` 、 `mock_components`
- `mode` : 控制模式，可选值为 `position`(仅 slave 生效) 、 `mix` 、 `effort`
- `feedback` ： 是否有力反馈/反馈类型（仅对 master 生效），可选值为 `false/none` 、 `position`(基于位置误差的力反馈) 、 `effort`（基于额外力的力反馈）
- `input_topic` ：订阅的反馈的关节状态话题(绝对名称)，默认为空或者 `auto` 为 `/drag_teleop_{master|slave}/teleop_state`
- `moveJ_pub` ：仅主臂启动有效， false/true ，默认 false，是否发布到 moveJ 的命令接口，具体配置到 controller yaml 中
- `use_sim_time` ： 是否使用仿真时间，可选值为 `true` 或 `false`
- `namespace` ： 本控制器命名空间，默认值为 `/drag_teleop` ，自动与 `role` 参数拼接为 `/drag_teleop_{master|slave}`
- `rviz` ： 是否启动 rviz，可选值为 `true` 或 `false`
- `xacro_{args}` ： xacro 参数，总是生效
- `hardware_{args}` ： xacro参数，仅 hardware:=real/real_usb 生效


### 控制模式 `mode`

控制模式说明：
1. `position` ：仅位置控制，适用于 hardware 只有 position 命令接口的形式，无力反馈（仅 slave 可用）
2. `mix` ：MIX模式，适用于 hardware 同时使用 position velctory effort 三种命令并在内部混合并计算最终关节力矩的情况，如果有反馈就使用 q_m q_s 的误差实现(我原本的方案)
3. `effort` ：effort 模式，控制器直接下发 effort ，适用于 hardware 有 effort 接口的情况（对于 mix 模式的 hardware ，可在启动参数指定 hardware_joint_kp/kd 为 0 来关闭位置反馈，从而实现 effort 控制）

role与控制模式：

role | mode | hardware
----|----|----|
master | mix | position(仅开启力反馈) & velocity(无) & effort(重力补偿力矩)
master | effort | effort(重力补偿力矩+力反馈力矩)
slave | position | position(目标位置)
slave | mix | position(目标位置) & velocity(目标速度) & effort(重力补偿力矩)
slave | effort | effort(重力补偿力矩+位置速度跟踪力矩)

### 反馈类型 `feedback`

反馈类型表：

主臂 | 从臂 | 反馈类型
----|----|----
mix | position | false
mix | position | position
mix | mix | false
mix | mix | position
mix | effort | false
mix | effort | position
effort | position | false
effort | position | position
effort | position | effort(仅当从臂存在 effort 状态接口)
effort | mix | false
effort | mix | position
effort | effort | effort(仅当从臂存在 effort 状态接口)
effort | effort | false
effort | effort | position
effort | effort | effort

### 计算

- slave-position:
$$\tau_{model,slave} = M(q)\ddot q + C(q, \dot q)\dot q + G(q)$$
$$\tau_{ext,slave} = \tau_{state,slave} - \tau_{model,slave}$$

$$q_{cmd,slave} = q_{state,master}$$

- slave-mix:
$$\tau_{model,slave} = M(q)\ddot q + C(q, \dot q)\dot q + G(q)$$
$$\tau_{ext,slave} = \tau_{state,slave} - \tau_{model,slave}$$

$$q_{cmd,slave} = q_{state,master}$$
$$\dot q_{cmd,slave} = \dot q_{state,master}$$
$$\tau_{cmd,slave} = \tau_G(q_{cmd,slave})$$

- slave-effort:
$$\tau_{model,slave} = M(q)\ddot q + C(q, \dot q)\dot q + G(q)$$
$$\tau_{ext,slave} = \tau_{state,slave} - \tau_{model,slave}$$
如果从臂不存在 effort 状态接口，令 $\tau_{state,slave} = \tau_{cmd,slave}$
$$\tau_{imp,slave} = K_p (q_{state,master} - q_{state,slave}) + K_d (\dot q_{state,master} - \dot q_{state,slave})$$

$$\tau_{cmd,slave} = \tau_{imp,slave} + \tau_G(q_{cmd,slave})$$

- master-*-feedback(false):
$$q_{cmd,master} = q_{state,master}$$
$$\dot q_{cmd,master} = 0$$
$$\tau_{cmd,master} = \tau_G(q_{state,master})$$

- master-mix-feedback(position):
$$\Delta q_{feedback,master} = - G \cdot (q_{state,master} - q_{state,slave})$$
$$q_{cmd,master} = q_{state,master} + \delta q_{feedback,master}$$
$$\dot q_{cmd,master} = 0$$
$$\tau_{cmd,master} = \tau_G(q_{state,master})$$

- master-effort-feedback(position):
$$q_{cmd,master} = -G \cdot (q_{state,master} - q_{state,slave})$$
$$\tau_{imp,master} = K_p (q_{cmd,master} - q_{state,slave})$$
$$\tau_{cmd,master} = \tau_{imp,master} + \tau_G(q_{state,master})$$

- master-effort-feedback(effort):
$$\tau_{cmd,master} = -G \cdot \tau_{ext,slave} + \tau_G(q_{state,master})$$



### 话题/服务

主臂状态发布话题： `/namespace_master/teleop_states` `sensor_msgs/msg/JointState`
主臂控制模式切换服务： `/namespace_master/teleop_mode` `string` (mix,effort)
主臂反馈模式切换服务： `/namespace_master/teleop_feedback` `string` (none/false,position,effort)
从臂状态发布： `/namespace_slave/teleop_states` `sensor_msgs/msg/JointState`
主臂控制模式切换服务： `/namespace_master/teleop_mode` `string` (position,mix,effort)



| 字段                | 主臂发布                  | 从臂发布           |
| ----------------- | --------------------- | -------------- |
| `name`            | 主臂关节名                 | 从臂关节名          |
| `position`        | 当前 $q_{state,master}$   | 当前 $q_{state,slave}$       |
| `velocity`        | 当前 $\dot q_{state,master}$ | 当前 $\dot q_{state,slave}$  |
| `effort`          | 0 | 从臂反馈 $\tau_{ext,slave}$，如果有，否则 0 |
| `header.stamp`    | 时间                  | 时间           |
| `header.frame_id` | `"master"`            | `"slave"`      |


### 启动流程

主臂控制：
判断 hardware interface 的 command 和 state 接口是否满足所选的启动模式 `mode` 和 `feedback`
hardware interface 读取 $q_{state,master}, \dot q_{state,master}, \tau_{state,master}$ 状态
话题读取 $q_{state,slave}, \dot q_{state,slave}, \tau_{ext,slave}$ 并做关节映射
根据控制模式和反馈模式计算 $q_{cmd,master}, \dot q_{cmd,master}, \tau_{imp,master}, \tau_{cmd,master}$
写入 hardware interface 和发布 `/namespace_master/teleop_states` 话题

从臂控制：
判断 hardware interface 的 command 和 state 接口是否满足所选的启动模式 `mode` 和 `feedback`
hardware interface 读取 $q_{state,slave}, \dot q_{state,slave}, \tau_{state,slave}$ 状态
话题读取 $q_{state,master}, \dot q_{state,master}, \tau_{ext,master}$ 并做关节映射
根据控制模式计算 $q_{cmd,slave}, \dot q_{cmd,slave}, \tau_{imp,slave}, \tau_{cmd,slave}, \tau_{model,slave}, \tau_{ext,slave}$ ，进行 ruckig 平滑或者不平滑
写入 hardware interface 和发布 `/namespace_slave/teleop_states` 话题


注意：计算相关量时可以根据启动配置全部计算了，即使不需要写入或者发布；从臂总是要发布其位置、速度、额外力矩到 `/namespace_slave/teleop_states` 话题；主臂总是要发布其位置、速度到 `/namespace_master/teleop_states` 话题；发布的机器人状态都是不带映射的状态，谁接收谁做映射(除了主臂发布的moveJ命令)



### 控制器配置文件

```yaml
drag_teleop_slave:
  ros__parameters:
    master:
      joints:
        - left_joint1
        - left_joint2
        - left_joint3
        - left_joint4
        - left_joint5
        - left_joint6
        - right_joint1
        - right_joint2
        - right_joint3
        - right_joint4
        - right_joint5
        - right_joint6
      max_effort: [21.0, 36.0, 36.0, 21.0, 10.0, 10.0,
                   21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
      publish_topic: teleop_state # 发布的机器人状态话题，拼接为 /namespace_master/teleop_state
      # 此处不需要订阅的机器人状态话题，该话题由 launch 指定
      feedback:
        position: # 基于位置误差的力反馈参数
          gain: [0.6, 0.6, 0.6, 0.6, 0.6, 0.6,
                 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
          dead_zone: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
                      0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
          max_delta_q: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                        0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        effort:  # 基于外部力的力反馈参数
          gain: [0.6, 0.6, 0.6, 0.6, 0.6, 0.6,
                 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
          max_ext_effort: [7.0, 12.0, 12.0, 7.0, 3.3, 3.3,
                           7.0, 12.0, 12.0, 7.0, 3.3, 3.3]
      ocs2_cmd: # 对于主臂遥操作 ocs2 的从臂的参数 moveJ 和 gripper_controller
        pub_rate: 100
        move_j:
          joints:
            - left_joint1
            - left_joint2
            - left_joint3
            - left_joint4
            - left_joint5
            - left_joint6
            - right_joint1
            - right_joint2
            - right_joint3
            - right_joint4
            - right_joint5
            - right_joint6
          cmd_topic: /ocs2_arm_controller/target_joint_position
      gripper: # 先做关节映射后发布
        {joint: left_gripper_joint , cmd_topic: /left_gripper_joint/position_command}
        {joint: right_gripper_joint, cmd_topic: /right_gripper_joint/position_command}


    slave:
      joints:
        - left_joint1
        - left_joint2
        - left_joint3
        - left_joint4
        - left_joint5
        - left_joint6
        - right_joint1
        - right_joint2
        - right_joint3
        - right_joint4
        - right_joint5
        - right_joint6

      max_effort: [21.0, 36.0, 36.0, 21.0, 10.0, 10.0,
                   21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
      publish_topic: teleop_state # 发布的机器人状态话题，拼接为 /namespace_master/teleop_state
      # 此处不需要订阅的机器人状态话题，该话题由 launch 指定
      control:
        effort: #力矩控制模式的参数
        kp: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
             30.0, 30.0, 30.0, 30.0, 30.0, 30.0,]
        kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
             3.0, 3.0, 3.0, 3.0, 3.0, 3.0]

    mapper:
      master_joints: 
        - left_joint1
        - left_joint2
        - left_joint3
        - left_joint4
        - left_joint5
        - left_joint6
        - left_gripper_joint
        - right_joint1
        - right_joint2
        - right_joint3
        - right_joint4
        - right_joint5
        - right_joint6
        - right_gripper_joint
      slave_joints:
        - left_joint1
        - left_joint2
        - left_joint3
        - left_joint4
        - left_joint5
        - left_joint6
        - left_gripper_joint
        - right_joint1
        - right_joint2
        - right_joint3
        - right_joint4
        - right_joint5
        - right_joint6
        - right_gripper_joint
      offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
              1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      sign: [1, 1, 1, 1, 1, 1, 1,
             1, 1, 1, 1, 1, 1, 1]
```

### 代码结构

节点文件：订阅、发布、hardware 写入/读取
控制器文件：不同控制和反馈模式的控制量计算（如有必要，将动力学、重力补偿等也拆分为一个文件）
映射文件：关节映射/逆映射
ocs2 发布相关

