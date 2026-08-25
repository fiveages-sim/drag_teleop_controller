"""Drag Teleop Controller launch (master / slave role).

Starts robot_state_publisher + ros2_control_node with this package's controller
configuration, then spawns the drag teleop controller for the given role plus
the two gripper controllers (left/right, same config as the ocs2 launch).

Usage:
  # master (dragged by operator, gravity compensated)
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
    role:=master hardware:=mock_components

  # slave (follows the master)
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
    role:=slave hardware:=mock_components

  # real hardware, master with position force feedback + ocs2 moveJ publish
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
    role:=master hardware:=real feedback:=position moveJ_pub:=true \
    hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
    hardware_joint_kd:="0.1, 0.1, 0.1, 0.1, 0.1, 0.1"

  # custom controller params file
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py \
    role:=slave controller_params:=/path/to/my.yaml

  # hardware_/xacro_ prefixed args (same format as ocs2 demo.launch.py):
  #   xacro_xxx:=    always applied, e.g. xacro_control_mode:=pd_control
  #   hardware_xxx:= only with hardware:=real/real_usb, e.g.
  #     hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01"
  #     hardware_joint_kd:="0.1, 0.1, 0.1, 0.1, 0.1, 0.1"
  #     hardware_gripper_kp:=0.001 hardware_gripper_kd:=0.01
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from robot_common_launch import (
    build_xacro_mappings,
    create_robot_profile_launch_arguments,
)


def _inject_controller_params(yaml_path, urdf, role, overrides):
    """把 robot_description 与 launch 覆盖参数合并进配置 yaml 的控制器分节。

    控制器 on_configure 需要 URDF（构建 Pinocchio 模型），而 URDF 由 launch 从
    xacro 展开、无法写死在 yaml 中，因此必须在此注入。role/mode/feedback/
    input_topic/moveJ_pub 等 launch 参数同样注入（覆盖 yaml 默认值）。

    参数文件解析规则（rcl_yaml_param_parser）：每一层键都是节点名的一部分，
    支持通配符。顶层 `/**` 匹配任意深度的节点名前缀，因此：
      /**/controller_manager        -> 匹配任意 namespace 下的 CM 节点
      /**/drag_teleop_controller    -> 匹配任意 namespace 下的控制器节点
    无需按 namespace 改写顶层键。

    该文件同时会被 CM 作为控制器节点的 --params-file 转发，因此把
    robot_description 合并进 `/**/drag_teleop_controller` 的 ros__parameters，
    控制器 configure 时就能从自身参数读到。
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f) or {}
    wildcard = data.setdefault("/**", {})
    ctrl_node = wildcard.setdefault("drag_teleop_controller", {})
    ctrl_params = ctrl_node.setdefault("ros__parameters", {})
    ctrl_params["robot_description"] = urdf
    for key, value in overrides.items():
        ctrl_params[key] = value
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="drag_teleop_cm_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, allow_unicode=True)
    return path


def launch_setup(context, *args, **kwargs):
    role = context.launch_configurations["role"]
    if role not in ("master", "slave"):
        raise RuntimeError(
            "role must be 'master' or 'slave', got '{}'".format(role))
    robot = context.launch_configurations["robot"]
    robot_type = context.launch_configurations["type"]
    hardware = context.launch_configurations["hardware"]
    use_sim_time = context.launch_configurations["use_sim_time"] == "true"
    # 基础 namespace（默认 /drag_teleop），与 role 拼接为 /drag_teleop_{role}
    namespace = context.launch_configurations["namespace"]
    if namespace in ("", "/"):
        namespace = "/drag_teleop"
    namespace = namespace.rstrip("/") + "_" + role
    rviz_enabled = context.launch_configurations["rviz"] == "true"
    robot_profile = context.launch_configurations.get("robot_profile", "")

    # 控制器参数文件（controller_params 参数；默认本包 config/ros2_control.yaml）
    controller_params = context.launch_configurations["controller_params"]
    if not controller_params:
        controller_params = os.path.join(
            get_package_share_directory("drag_teleop_controller"),
            "config", "ros2_control.yaml")

    # launch 覆盖参数（注入控制器 ros__parameters，键与新 yaml 结构对应）
    overrides = {"role": role}
    # 控制模式：master -> master.control.mode，slave -> slave.control.type
    mode_key = "master.control.mode" if role == "master" else "slave.control.type"
    mode = context.launch_configurations.get("mode", "")
    if mode:
        overrides[mode_key] = mode
    feedback = context.launch_configurations.get("feedback", "")
    if feedback:
        overrides["master.feedback.type"] = feedback
    input_topic = context.launch_configurations.get("input_topic", "")
    if input_topic and input_topic != "auto":
        overrides["input_topic"] = input_topic
    else:
        # auto：注入对侧话题（master 订阅 slave，slave 订阅 master）
        other_role = "slave" if role == "master" else "master"
        overrides["input_topic"] = "/drag_teleop_{}/teleop_states".format(other_role)
    moveJ_pub = context.launch_configurations.get("moveJ_pub", "")
    if moveJ_pub:
        overrides["master.ocs2_cmd.enabled"] = moveJ_pub == "true"

    # 参照 ocs2_arm_controller demo.launch.py：用 robot_common_launch 的
    # build_xacro_mappings 统一构建 xacro mappings，自动处理 hardware_/xacro_
    # 前缀启动参数（与 OCS2 语义一致）：
    #   xacro_xxx:=   总是生效（任何 hardware），如 xacro_control_mode:=pd_control
    #   hardware_xxx:= 仅 hardware:=real/real_usb 生效，如 hardware_joint_kp:=
    # 以及 robot_profile 的 hardware:/xacro: 段注入。
    # kp/kd/control_mode 等硬件参数没有专用 launch 参数，统一走前缀参数。
    # 注意：kp/kd 不能直接给 ros2_control_node 传同名参数——硬件接口 expose
    # 时会用 URDF 值覆盖节点参数，因此必须经 xacro mappings 写入 URDF。
    mappings = build_xacro_mappings(
        hardware, context.launch_configurations, robot_profile or None)

    # Expand the ros2_control URDF from the description package (read-only reuse).
    description_package = robot + "_description"
    xacro_file = os.path.join(
        get_package_share_directory(description_package),
        "xacro", "ros2_control", "robot.xacro")
    doc = xacro.process_file(xacro_file, mappings=mappings)
    urdf = doc.toprettyxml(indent="  ")

    nodes = []
    nodes.append(LogInfo(
        msg="\n===============================================================\n"
            "[drag_teleop_controller] \n"
            " role={} robot={} type={} \n"
            "hardware={} namespace='{}' rviz={} \n"
            "mode={} feedback={} moveJ_pub={} \n"
            "controller_params={} \n"
            "description_package={} \n"
            "===============================================================".format(
                role, robot, robot_type, hardware, namespace, rviz_enabled,
                overrides.get(mode_key, "(yaml)"),
                overrides.get("master.feedback.type", "(yaml)"),
                overrides.get("master.ocs2_cmd.enabled", "(yaml)"),
                controller_params, description_package)
        ))

    # robot_state_publisher publishes TF and /<ns>/robot_description from the
    # same URDF. TransformBroadcaster 硬编码发布到绝对名 "/tf" 和 "/tf_static"
    # （不受节点 namespace 影响），因此使用 "frame_prefix" 参数指定 frame 前缀。
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_description": urdf,
            "frame_prefix": namespace + "/",
            "publish_frequency": 100.0,
            "use_tf_static": True,
            "use_sim_time": use_sim_time,
        }],
    ))

    # ros2_control_node: controller configuration from the params file.
    # robot_description 不通过 CM 参数传入——CM 默认订阅相对话题
    # "robot_description"（RSP 在 ns 内发布同名相对话题，自动匹配）。
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            _inject_controller_params(controller_params, urdf, role, overrides),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    ))

    # joint_state_broadcaster: 发布 joint_state 主题，用于其他节点订阅。
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

    # drag_teleop_controller spawner（master/slave 共用同一控制器名，
    # role 已通过参数注入）：参数已通过 CM 配置 yaml 注入。
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["drag_teleop_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

    # 夹爪不再由 AdaptiveGripperController 控制：drag_teleop_controller
    # 直接 claim 夹爪命令接口（master 保位 / slave 跟随主臂夹爪状态，
    # velocity/effort 写 0）。若再 spawn 夹爪控制器会因命令接口被占用而
    # configure 失败，故两侧均不启动。

    # Optional RViz (default off). Runs inside the same namespace so relative
    # topic names in the config ("tf", "robot_description") follow the robot.
    if rviz_enabled:
        rviz_config = os.path.join(
            get_package_share_directory("drag_teleop_controller"),
            "config", "drag_teleop_controller.rviz")
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("role", description="Robot role: master | slave"),
        DeclareLaunchArgument("robot", default_value="panthera_ht",
                              description="Robot name (informational only)"),
        DeclareLaunchArgument("type", default_value="dual",
                              description="Arm composition: single | left | right | dual",
                              choices=["single", "left", "right", "dual"]),
        DeclareLaunchArgument("hardware", default_value="mock_components",
                              description="real | real_usb | mock_components | gz | isaac",
                              choices=["real", "real_usb", "mock_components", "gz", "isaac"]),
        DeclareLaunchArgument("mode", default_value="",
                              description="Control mode: position | mix | effort "
                                          "(empty = use yaml value; position is slave-only)"),
        DeclareLaunchArgument("feedback", default_value="",
                              description="Force feedback (master only): false | position | effort "
                                          "(empty = use yaml value)"),
        DeclareLaunchArgument("input_topic", default_value="auto",
                              description="Remote teleop_states topic (absolute name); "
                                          "auto = /drag_teleop_{other_role}/teleop_states"),
        DeclareLaunchArgument("moveJ_pub", default_value="false",
                              description="Publish ocs2 moveJ commands "
                                          "(master only, true|false); gripper "
                                          "position commands are always "
                                          "published per master.gripper config"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespace", default_value="/drag_teleop",
                              description="Base ROS namespace; role is appended "
                                          "('/drag_teleop' -> '/drag_teleop_master')"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Launch RViz2 (true|false)"),
        DeclareLaunchArgument(
            "controller_params", default_value="",
            description="Controller params yaml path (default: this package "
                        "config/ros2_control.yaml; robot-specific symlinks "
                        "like panthera_ht_2_panthera_ht.yaml also work)"),
        # 参照 ocs2 demo.launch.py：hardware_/xacro_ 前缀参数透传（无需逐个声明）：
        #   xacro_xxx:=   总是生效，如 xacro_control_mode:=pd_control
        #   hardware_xxx:= 仅 hardware:=real/real_usb 生效，如 hardware_joint_kp:=
        # kp/kd/control_mode 等硬件参数均无专用 launch 参数，统一走前缀参数。
        # robot_profile 提供 hardware:/xacro: 段（robot.local.yaml）
        *create_robot_profile_launch_arguments(),
        OpaqueFunction(function=launch_setup),
    ])