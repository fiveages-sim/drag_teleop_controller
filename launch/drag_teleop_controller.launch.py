"""Drag Teleop Controller launch.

Starts robot_state_publisher + ros2_control_node with this package's controller
configuration, then spawns the drag teleop controller.

No existing package is modified: the URDF is expanded from the description
package xacro at launch time and all controller parameters come from this
package's config/gravity_compensation.yaml (no launch-time overrides except
robot_description, which cannot be hard-coded in yaml).

Usage:
  # mock hardware (no motors, for validation)
  ros2 launch ht_gravity_compensation gravity_compensation.launch.py

  # real dual arm
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py hardware:=real

  # optional RViz (default off) and custom namespace
  ros2 launch drag_teleop_controller drag_teleop_controller.launch.py rviz:=true namespace:=my_ns

  # hardware_/xacro_ prefixed args (same format as ocs2 demo.launch.py):
  #   xacro_xxx:=    always applied, e.g. xacro_control_mode:=pd_control
  #   hardware_xxx:= only with hardware:=real/real_usb, e.g.
  #     hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01"
  #     hardware_joint_kd:="0.1, 0.1, 0.1, 0.1, 0.1, 0.1"
  #     hardware_gripper_kp:=0.001 hardware_gripper_kd:=0.01
  # kp/kd/control_mode 等硬件参数没有专用 launch 参数，统一走前缀参数。
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


def _inject_robot_description(yaml_path, urdf, feedback=None):
    """把 robot_description 合并进配置 yaml 的控制器分节，返回临时文件路径。

    控制器 on_configure 需要 URDF（构建 Pinocchio 模型），而 URDF 由 launch 从
    xacro 展开、无法写死在 yaml 中，因此必须在此注入。其余控制器参数
    （joints/hold_joints/max_effort 等）全部直接使用 yaml 中的值。

    feedback 非 None 时覆盖 feedback.enabled（feedback:=true/false 命令行参数），
    便于不重启整套从臂链路快速开关力反馈。

    参数文件解析规则（rcl_yaml_param_parser）：每一层键都是节点名的一部分，
    支持通配符。顶层 `/**` 匹配任意深度的节点名前缀，因此：
      /**/controller_manager              -> 匹配任意 namespace 下的 CM 节点
      /**/drag_teleop_controller -> 匹配任意 namespace 下的控制器节点
    无需按 namespace 改写顶层键（旧实现：`drag_controller/...` 前缀）。
    （`/*` 单个星号只匹配恰好一个路径 token，无 namespace 时会失效，故用 `/**`。）

    该文件同时会被 CM 作为控制器节点的 --params-file 转发（见 CM 日志 node
    arguments），因此把 robot_description 合并进
    `/**/drag_teleop_controller` 的 ros__parameters，
    控制器 configure 时就能从自身参数读到。
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f) or {}
    wildcard = data.setdefault("/**", {})
    ctrl_node = wildcard.setdefault("drag_teleop_controller", {})
    ctrl_params = ctrl_node.setdefault("ros__parameters", {})
    ctrl_params["robot_description"] = urdf
    if feedback is not None:
        # launch 参数值是字符串（"true"/"false"），直接写入 yaml 会被 dump 成
        # 带引号的字符串（feedback.enabled: 'true'），与控制器声明的 bool 类型
        # 不匹配，rclcpp 会忽略该 override（feedback.enabled 保持默认 false）。
        # 必须显式转成 bool，yaml 才会输出无引号的 true/false。
        ctrl_params["feedback.enabled"] = (
            str(feedback).strip().lower() in ("1", "true", "yes", "on"))
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="drag_teleop_cm_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, allow_unicode=True)
    return path


def launch_setup(context, *args, **kwargs):
    robot = context.launch_configurations["robot"]
    robot_type = context.launch_configurations["type"]
    hardware = context.launch_configurations["hardware"]
    use_sim_time = context.launch_configurations["use_sim_time"] == "true"
    namespace = context.launch_configurations["namespace"]
    # 空串 / "/" 都表示全局命名空间（launch CLI 无法传真正空串，用 "/" 即可）
    if namespace in ("", "/"):
        namespace = None
    # description_package = context.launch_configurations["description_package"]
    description_package = robot + "_description"
    rviz_enabled = context.launch_configurations["rviz"] == "true"

    robot_profile = context.launch_configurations.get("robot_profile", "")

    # 参照 ocs2_arm_controller demo.launch.py：用 robot_common_launch 的
    # build_xacro_mappings 统一构建 xacro mappings，自动处理 hardware_/xacro_
    # 前缀启动参数（与 OCS2 语义一致）：
    #   xacro_xxx:=   总是生效（任何 hardware），如 xacro_control_mode:=pd_control
    #   hardware_xxx:= 仅 hardware:=real/real_usb 生效，如 hardware_joint_kp:=
    # 以及 robot_profile 的 hardware:/xacro: 段注入。
    # kp/kd/control_mode 等硬件参数没有专用 launch 参数——全部通过前缀参数或
    # 描述包 xacro 默认值设置（机器人无关：不同硬件用各自的前缀参数）。
    # 注意：kp/kd 不能直接给 ros2_control_node 传同名参数——硬件接口 expose
    # 时会用 URDF 值覆盖节点参数，因此必须经 xacro mappings 写入 URDF。
    mappings = build_xacro_mappings(
        hardware, context.launch_configurations, robot_profile or None)

    # Expand the ros2_control URDF from the description package (read-only reuse).
    xacro_file = os.path.join(
        get_package_share_directory(description_package),
        "xacro", "ros2_control", "robot.xacro")
    doc = xacro.process_file(xacro_file, mappings=mappings)
    urdf = doc.toprettyxml(indent="  ")

    nodes = []
    nodes.append(LogInfo(
        msg=f"\n===============================================================\n"
            f"[drag_teleop_controller] \n robot={robot} type={robot_type} \n"
            f"hardware={hardware} namespace='{namespace}' rviz={rviz_enabled} \n"
            f"description_package={description_package} \n"
            f"==============================================================="
        ))

    # robot_state_publisher publishes TF and /<ns>/robot_description from the same URDF.
    # TransformBroadcaster 硬编码发布到绝对名 "/tf" 和 "/tf_static"（不受节点 namespace 影响），
    # 因此使用 "frame_prefix" 参数指定机器人描述文件的 frame 前缀。
    robot_state_frame_prefix = namespace+"/" if namespace else ""
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_description": urdf,
            "frame_prefix": robot_state_frame_prefix,
            "publish_frequency": 100.0,
            "use_tf_static": True,
            "use_sim_time": use_sim_time,
        }],
    ))

    # ros2_control_node: controller configuration from THIS package.
    # 配置 yaml 顶层键为 /** 通配符 + 子键分节（/**/controller_manager、
    # /**/drag_teleop_controller），匹配任意 namespace 下的节点，
    # 无需按 namespace 加前缀；控制器参数（joints/max_effort 等）全部
    # 直接使用 yaml 中的值，仅 robot_description 由 _inject_robot_description
    # 合并进 /**/drag_teleop_controller（URDF 无法写死在 yaml 中），
    # 因为该文件会同时作为控制器节点的 --params-file 加载。
    # robot_description 不通过 CM 参数传入——CM 默认订阅相对话题 "robot_description"
    # （RSP 在 ns 内发布同名相对话题，自动匹配）。
    config_yaml = os.path.join(
        get_package_share_directory("drag_teleop_controller"),
        "config", f"{robot}.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(
            get_package_share_directory("drag_teleop_controller"),
            "config", "ros2_controller_params.yaml")

    # controller_manager: 输入 controller yaml 配置、带有 ros2_control 的 URDF
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            _inject_robot_description(
                config_yaml, urdf,
                feedback=context.launch_configurations.get("feedback") or None),
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

    # drag_teleop_controller 控制器 spawner：参数已通过 CM 配置 yaml 注入。
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["drag_teleop_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

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
        DeclareLaunchArgument("robot", default_value="panthera_ht",
                              description="Robot name (informational only)"),
        DeclareLaunchArgument("type", default_value="dual",
                              description="Arm composition: single | left | right | dual",
                              choices=["single", "left", "right", "dual"]),
        DeclareLaunchArgument("hardware", default_value="mock_components",
                              description="real | real_usb | mock_components | gz | isaac",
                              choices=["real", "real_usb", "mock_components", "gz", "isaac"]),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespace", default_value="drag_teleop",
                              description="ROS namespace for robot_state_publisher, "
                                          "controller_manager and RViz (use '/' for global)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Launch RViz2 (true|false)"),
        DeclareLaunchArgument(
            "feedback", default_value="",
            description="Override feedback.enabled (true|false); empty = use yaml value. "
                        "Force feedback pulls the master arm back when the slave is blocked"),
        # 参照 ocs2 demo.launch.py：hardware_/xacro_ 前缀参数透传（无需逐个声明）：
        #   xacro_xxx:=   总是生效，如 xacro_control_mode:=pd_control
        #   hardware_xxx:= 仅 hardware:=real/real_usb 生效，如 hardware_joint_kp:=
        # kp/kd/control_mode 等硬件参数均无专用 launch 参数，统一走前缀参数。
        # robot_profile 提供 hardware:/xacro: 段（robot.local.yaml）
        *create_robot_profile_launch_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
