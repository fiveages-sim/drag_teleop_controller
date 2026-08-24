//
// DragTeleopController implementation.
//
#include "drag_teleop_controller/drag_teleop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <tuple>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_client.hpp"

PLUGINLIB_EXPORT_CLASS(
  drag_teleop_controller::DragTeleopController,
  controller_interface::ControllerInterface);

namespace drag_teleop_controller
{

controller_interface::CallbackReturn DragTeleopController::on_init()
{
  try
  {
    // ---- 启动参数 ----
    role_ = auto_declare<std::string>("role", role_);
    mode_str_ = auto_declare<std::string>("mode", mode_str_);
    feedback_str_ = auto_declare<std::string>("feedback", feedback_str_);
    input_topic_ = auto_declare<std::string>("input_topic", input_topic_);
    moveJ_pub_ = auto_declare<bool>("moveJ_pub", moveJ_pub_);
    gravity_vector_ = auto_declare<std::vector<double>>(
      "gravity_vector", gravity_vector_);
    // robot_description 通过 --params-file（launch 写入的控制器参数文件）传入：
    // rclcpp 的 override 只在参数被声明时落地，未声明的参数会被忽略，
    // 因此必须在此声明（值取自 override，缺省为空字符串）。
    auto_declare<std::string>(urdf_param_name_, "");

    // ---- master 段 ----
    master_joints_ = auto_declare<std::vector<std::string>>(
      "master.joints", master_joints_);
    master_max_effort_ = auto_declare<std::vector<double>>(
      "master.max_effort", master_max_effort_);
    master_publish_topic_ = auto_declare<std::string>(
      "master.publish_topic", master_publish_topic_);
    feedback_params_.gain = auto_declare<std::vector<double>>(
      "master.feedback.position.gain", {});
    feedback_params_.dead_zone = auto_declare<std::vector<double>>(
      "master.feedback.position.dead_zone", {});
    feedback_params_.max_delta_q = auto_declare<std::vector<double>>(
      "master.feedback.position.max_delta_q", {});
    feedback_params_.effort_gain = auto_declare<std::vector<double>>(
      "master.feedback.effort.gain", {});
    feedback_params_.max_ext_effort = auto_declare<std::vector<double>>(
      "master.feedback.effort.max_ext_effort", {});
    ocs2_config_.pub_rate = auto_declare<double>(
      "master.ocs2_cmd.pub_rate", ocs2_config_.pub_rate);
    ocs2_config_.move_j.joints = auto_declare<std::vector<std::string>>(
      "master.ocs2_cmd.move_j.joints", {});
    ocs2_config_.move_j.cmd_topic = auto_declare<std::string>(
      "master.ocs2_cmd.move_j.cmd_topic", "");
    gripper_joints_param_ = auto_declare<std::vector<std::string>>(
      "master.ocs2_cmd.gripper_joints", {});
    gripper_topics_param_ = auto_declare<std::vector<std::string>>(
      "master.ocs2_cmd.gripper_topics", {});

    // ---- slave 段 ----
    slave_joints_ = auto_declare<std::vector<std::string>>(
      "slave.joints", slave_joints_);
    slave_max_effort_ = auto_declare<std::vector<double>>(
      "slave.max_effort", slave_max_effort_);
    slave_publish_topic_ = auto_declare<std::string>(
      "slave.publish_topic", slave_publish_topic_);
    impedance_params_.kp = auto_declare<std::vector<double>>(
      "slave.control.effort.kp", {});
    impedance_params_.kd = auto_declare<std::vector<double>>(
      "slave.control.effort.kd", {});
    smooth_enabled_ = auto_declare<bool>("slave.smooth.enabled", smooth_enabled_);
    smooth_max_velocity_ = auto_declare<double>(
      "slave.smooth.max_velocity", smooth_max_velocity_);
    smooth_max_acceleration_ = auto_declare<double>(
      "slave.smooth.max_acceleration", smooth_max_acceleration_);
    smooth_max_jerk_ = auto_declare<double>(
      "slave.smooth.max_jerk", smooth_max_jerk_);

    // ---- mapper 段 ----
    mapper_config_.master_joints = auto_declare<std::vector<std::string>>(
      "mapper.master_joints", {});
    mapper_config_.slave_joints = auto_declare<std::vector<std::string>>(
      "mapper.slave_joints", {});
    mapper_config_.offset = auto_declare<std::vector<double>>(
      "mapper.offset", {});
    mapper_config_.scale = auto_declare<std::vector<double>>(
      "mapper.scale", {});
    mapper_config_.sign = auto_declare<std::vector<int>>(
      "mapper.sign", {});
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool DragTeleopController::loadRobotDescription(std::string & urdf_out)
{
  // 1) 自身参数（controller_manager 注入的 robot_description，launch 已直接写入控制器参数）
  if (get_node()->has_parameter(urdf_param_name_))
  {
    rclcpp::Parameter param;
    if (get_node()->get_parameter(urdf_param_name_, param) &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      !param.as_string().empty())
    {
      urdf_out = param.as_string();
      return true;
    }
  }
  // 2) 回退：从 controller_manager 节点读取。
  //    必须使用独立临时节点：SyncParametersClient 会创建自己的 SingleThreadedExecutor
  //    并把节点加入其中，而控制器节点已属于 controller_manager 的 executor（会报错）。
  try
  {
    auto probe_node = std::make_shared<rclcpp::Node>("drag_teleop_controller_urdf_probe");
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
      probe_node, "/controller_manager");
    if (!client->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Parameter services of /controller_manager not available within 500ms; "
        "cannot fall back to reading '%s' from it", urdf_param_name_.c_str());
      return false;
    }
    if (client->has_parameter(urdf_param_name_))
    {
      urdf_out = client->get_parameter<std::string>(urdf_param_name_);
      return !urdf_out.empty();
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Failed to read '%s' from /controller_manager: %s",
      urdf_param_name_.c_str(), e.what());
  }
  return false;
}

controller_interface::CallbackReturn DragTeleopController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ---- 解析并校验启动参数 ----
  if (role_ != "master" && role_ != "slave")
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'role' must be 'master' or 'slave', got '%s'", role_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!ControlCalculator::parseMode(mode_str_, control_mode_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'mode' must be 'position'|'mix'|'effort', got '%s'",
      mode_str_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (role_ == "master" && control_mode_ == ControlMode::Position)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Control mode 'position' is slave-only (master supports mix|effort)");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!ControlCalculator::parseFeedback(feedback_str_, feedback_mode_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'feedback' must be 'false'|'position'|'effort', got '%s'",
      feedback_str_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (role_ == "slave")
  {
    feedback_mode_ = FeedbackMode::None;  // 力反馈仅 master 生效
  }

  // ---- 本侧配置 ----
  if (role_ == "master")
  {
    joints_ = master_joints_;
    max_effort_ = master_max_effort_;
    publish_topic_ = master_publish_topic_;
  }
  else
  {
    joints_ = slave_joints_;
    max_effort_ = slave_max_effort_;
    publish_topic_ = slave_publish_topic_;
  }
  if (joints_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter '%s.joints' must not be empty", role_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 夹爪：mapper.master_joints 中不在本侧臂关节列表中的关节（只读状态）
  gripper_joints_.clear();
  for (const auto & name : mapper_config_.master_joints)
  {
    if (std::find(joints_.begin(), joints_.end(), name) == joints_.end())
    {
      gripper_joints_.push_back(name);
    }
  }

  // ---- 映射器 ----
  try
  {
    mapper_ = std::make_unique<JointMapper>(mapper_config_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to build JointMapper: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ---- 动力学（URDF → Pinocchio） ----
  std::string urdf;
  if (!loadRobotDescription(urdf))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to obtain robot URDF from parameter '%s' (make sure ros2_control_node "
      "receives 'robot_description', e.g. via the launch file)",
      urdf_param_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  try
  {
    dynamics_ = std::make_unique<Dynamics>(urdf, gravity_vector_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to build Pinocchio model from URDF: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_->isValid())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio model is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 建立 joints_ → 模型 JointIndex 映射
  const auto & model = dynamics_->getModel();
  model_joint_indices_.clear();
  model_joint_indices_.reserve(joints_.size());
  for (const auto & name : joints_)
  {
    if (model.existJointName(name))
    {
      model_joint_indices_.push_back(model.getJointId(name));
    }
    else
    {
      model_joint_indices_.push_back(std::numeric_limits<size_t>::max());
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Joint '%s' not found in the URDF model; its gravity/model torque "
        "will be 0.0", name.c_str());
    }
  }

  // ---- 订阅对侧状态 / 发布本侧状态 ----
  remote_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    input_topic_, rclcpp::QoS(10).reliable().transient_local(),
    std::bind(
      &DragTeleopController::remoteStateCallback, this, std::placeholders::_1));
  state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    publish_topic_, 10);

  // ---- 模式切换服务 ----
  mode_service_ = get_node()->create_service<drag_teleop_controller::srv::TeleopMode>(
    "teleop_mode",
    std::bind(
      &DragTeleopController::modeServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  feedback_service_ =
    get_node()->create_service<drag_teleop_controller::srv::TeleopFeedback>(
      "teleop_feedback",
      std::bind(
        &DragTeleopController::feedbackServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // ---- ocs2 发布（master + moveJ_pub） ----
  if (role_ == "master" && moveJ_pub_)
  {
    ocs2_config_.gripper.clear();
    const size_t g = std::min(
      gripper_joints_param_.size(), gripper_topics_param_.size());
    for (size_t i = 0; i < g; ++i)
    {
      Ocs2GripperConfig gc;
      gc.joint = gripper_joints_param_[i];
      gc.cmd_topic = gripper_topics_param_[i];
      ocs2_config_.gripper.push_back(gc);
    }
    ocs2_pub_ = std::make_unique<Ocs2Publisher>(get_node(), ocs2_config_, *mapper_);
    ocs2_pub_->configure();
  }

  // ---- ruckig 平滑（slave + smooth.enabled） ----
  if (role_ == "slave" && smooth_enabled_)
  {
    ruckig_inputs_.resize(joints_.size());
    ruckig_outputs_.resize(joints_.size());
    smooth_pos_.assign(joints_.size(), 0.0);
    smooth_vel_.assign(joints_.size(), 0.0);
    smooth_initialized_ = false;
  }

  // ---- q̈ 数值微分状态 ----
  prev_velocity_.assign(joints_.size(), 0.0);
  accel_.assign(joints_.size(), 0.0);
  accel_initialized_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured: role=%s mode=%s feedback=%s, %zu arm joints, %zu gripper "
    "joints, input='%s' publish='%s'",
    role_.c_str(), ControlCalculator::modeName(control_mode_).c_str(),
    ControlCalculator::feedbackName(feedback_mode_).c_str(),
    joints_.size(), gripper_joints_.size(), input_topic_.c_str(),
    publish_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DragTeleopController::command_interface_configuration() const
{
  // INDIVIDUAL：只 claim 12 臂关节命令接口（不 claim 夹爪，避免与
  // adaptive_gripper_controller 冲突）。统一声明 position/velocity/effort
  // 三个接口（panthera_ht 硬件均导出），on_activate 按 mode 校验所需接口。
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto & joints = (role_ == "master") ? master_joints_ : slave_joints_;
  for (const auto & name : joints)
  {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
    config.names.push_back(name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
DragTeleopController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto & joints = (role_ == "master") ? master_joints_ : slave_joints_;
  for (const auto & name : joints)
  {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
    config.names.push_back(name + "/effort");
  }
  // 夹爪 position 状态（只读，用于映射发布 teleop_states）
  for (const auto & name : mapper_config_.master_joints)
  {
    if (std::find(joints.begin(), joints.end(), name) == joints.end())
    {
      config.names.push_back(name + "/position");
    }
  }
  return config;
}

controller_interface::CallbackReturn DragTeleopController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto claim_state = [this](
    const std::string & full_name,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> & out)
  {
    for (auto & interface : state_interfaces_)
    {
      if (interface.get_name() == full_name)
      {
        out.emplace_back(interface);
        return true;
      }
    }
    return false;
  };
  auto claim_state_ptr = [this](
    const std::string & full_name,
    std::vector<hardware_interface::LoanedStateInterface *> & out)
  {
    for (auto & interface : state_interfaces_)
    {
      if (interface.get_name() == full_name)
      {
        out.push_back(&interface);
        return true;
      }
    }
    out.push_back(nullptr);  // 保持与 joints_ 对齐（缺失 = 可选接口不存在）
    return false;
  };
  auto claim_command = [this](
    const std::string & full_name,
    std::vector<hardware_interface::LoanedCommandInterface *> & out)
  {
    for (auto & interface : command_interfaces_)
    {
      if (interface.get_name() == full_name)
      {
        out.push_back(&interface);
        return true;
      }
    }
    out.push_back(nullptr);  // 保持与 joints_ 对齐（缺失 = 可选接口不存在）
    return false;
  };

  // 清空上次借用的接口
  position_state_.clear();
  velocity_state_.clear();
  effort_state_.clear();
  position_cmd_.clear();
  velocity_cmd_.clear();
  effort_cmd_.clear();
  gripper_position_state_.clear();

  // 臂关节：position 状态必需；velocity/effort 状态可选；命令接口按名挑选
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const std::string & base = joints_[i];
    if (!claim_state(base + "/position", position_state_))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to claim state interface for joint '%s' (position state missing)",
        base.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    claim_state_ptr(base + "/velocity", velocity_state_);
    claim_state_ptr(base + "/effort", effort_state_);
    claim_command(base + "/position", position_cmd_);
    claim_command(base + "/velocity", velocity_cmd_);
    claim_command(base + "/effort", effort_cmd_);
  }

  // 夹爪：position 状态（只读）
  for (const auto & name : gripper_joints_)
  {
    if (!claim_state(name + "/position", gripper_position_state_))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to claim state interface for gripper joint '%s' "
        "(position state missing)", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // 校验当前 mode 所需的命令接口
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const std::string & base = joints_[i];
    if (control_mode_ == ControlMode::Position)
    {
      if (!position_cmd_[i])
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Mode 'position' requires a position command interface for '%s'",
          base.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else if (control_mode_ == ControlMode::Mix)
    {
      if (!position_cmd_[i] || !velocity_cmd_[i] || !effort_cmd_[i])
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Mode 'mix' requires position/velocity/effort command interfaces "
          "for '%s'", base.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else  // Effort
    {
      if (!position_cmd_[i] || !effort_cmd_[i])
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Mode 'effort' requires position/effort command interfaces for '%s'",
          base.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  // 本侧是否有 effort 状态接口（无则 τ_ext 用 τ_cmd 代替）
  has_effort_state_ = true;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!effort_state_[i])
    {
      has_effort_state_ = false;
      break;
    }
  }

  // 重置平滑 / q̈ 微分状态
  smooth_initialized_ = false;
  accel_initialized_ = false;
  prev_velocity_.assign(joints_.size(), 0.0);
  accel_.assign(joints_.size(), 0.0);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Activated: role=%s, %zu arm joints claimed (effort state: %s), "
    "%zu gripper states",
    role_.c_str(), joints_.size(), has_effort_state_ ? "yes" : "no",
    gripper_joints_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DragTeleopController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  position_state_.clear();
  velocity_state_.clear();
  effort_state_.clear();
  position_cmd_.clear();
  velocity_cmd_.clear();
  effort_cmd_.clear();
  gripper_position_state_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

void DragTeleopController::remoteStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(remote_mutex_);
  remote_positions_.clear();
  remote_velocities_.clear();
  remote_efforts_.clear();
  for (size_t i = 0; i < message->name.size(); ++i)
  {
    if (i < message->position.size() && std::isfinite(message->position[i]))
    {
      remote_positions_[message->name[i]] = message->position[i];
    }
    if (i < message->velocity.size() && std::isfinite(message->velocity[i]))
    {
      remote_velocities_[message->name[i]] = message->velocity[i];
    }
    if (i < message->effort.size() && std::isfinite(message->effort[i]))
    {
      remote_efforts_[message->name[i]] = message->effort[i];
    }
  }
  remote_received_ = true;
}

void DragTeleopController::modeServiceCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<drag_teleop_controller::srv::TeleopMode::Request> request,
  const std::shared_ptr<drag_teleop_controller::srv::TeleopMode::Response> response)
{
  (void)request_header;
  ControlMode mode;
  if (!ControlCalculator::parseMode(request->mode, mode))
  {
    response->success = false;
    response->message =
      "invalid mode '" + request->mode + "' (position|mix|effort)";
    return;
  }
  if (role_ == "master" && mode == ControlMode::Position)
  {
    response->success = false;
    response->message = "mode 'position' is slave-only";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    control_mode_ = mode;
  }
  response->success = true;
  response->message = "mode switched to " + ControlCalculator::modeName(mode);
  RCLCPP_INFO(get_node()->get_logger(), "%s", response->message.c_str());
}

void DragTeleopController::feedbackServiceCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<drag_teleop_controller::srv::TeleopFeedback::Request> request,
  const std::shared_ptr<drag_teleop_controller::srv::TeleopFeedback::Response> response)
{
  (void)request_header;
  if (role_ == "slave")
  {
    response->success = false;
    response->message = "force feedback only applies to the master role";
    return;
  }
  FeedbackMode feedback;
  if (!ControlCalculator::parseFeedback(request->mode, feedback))
  {
    response->success = false;
    response->message =
      "invalid feedback '" + request->mode + "' (false|position|effort)";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    feedback_mode_ = feedback;
  }
  response->success = true;
  response->message =
    "feedback switched to " + ControlCalculator::feedbackName(feedback);
  RCLCPP_INFO(get_node()->get_logger(), "%s", response->message.c_str());
}

void DragTeleopController::smoothReference(
  double dt,
  const std::vector<double> & q_state,
  const std::vector<double> & v_state,
  const std::vector<double> & q_ref,
  const std::vector<double> & dq_ref,
  std::vector<double> & q_out,
  std::vector<double> & dq_out)
{
  const size_t n = joints_.size();
  if (!smooth_initialized_)
  {
    // 首次：从当前状态出发
    for (size_t i = 0; i < n; ++i)
    {
      ruckig_inputs_[i].current_position = {q_state[i]};
      ruckig_inputs_[i].current_velocity = {v_state[i]};
      ruckig_inputs_[i].current_acceleration = {0.0};
      ruckig_inputs_[i].target_position = {q_ref[i]};
      ruckig_inputs_[i].target_velocity = {dq_ref[i]};
      ruckig_inputs_[i].target_acceleration = {0.0};
      ruckig_inputs_[i].max_velocity = {smooth_max_velocity_};
      ruckig_inputs_[i].max_acceleration = {smooth_max_acceleration_};
      ruckig_inputs_[i].max_jerk = {smooth_max_jerk_};
      ruckig_inputs_[i].min_velocity = {-smooth_max_velocity_};
      ruckig_inputs_[i].min_acceleration = {-smooth_max_acceleration_};
      ruckig_inputs_[i].duration_discretization =
        ruckig::DurationDiscretization::Discrete;
      ruckig::Ruckig<1> trajectory(dt);
      trajectory.update(ruckig_inputs_[i], ruckig_outputs_[i]);
      smooth_pos_[i] = ruckig_outputs_[i].new_position[0];
      smooth_vel_[i] = ruckig_outputs_[i].new_velocity[0];
    }
    smooth_initialized_ = true;
  }
  else
  {
    for (size_t i = 0; i < n; ++i)
    {
      ruckig_inputs_[i].current_position = {smooth_pos_[i]};
      ruckig_inputs_[i].current_velocity = {smooth_vel_[i]};
      ruckig_inputs_[i].current_acceleration = {
        ruckig_outputs_[i].new_acceleration[0]};
      ruckig_inputs_[i].target_position = {q_ref[i]};
      ruckig_inputs_[i].target_velocity = {dq_ref[i]};
      ruckig_inputs_[i].target_acceleration = {0.0};
      ruckig::Ruckig<1> trajectory(dt);
      const auto result = trajectory.update(ruckig_inputs_[i], ruckig_outputs_[i]);
      if (result < 0 ||
        !std::isfinite(ruckig_outputs_[i].new_position[0]) ||
        !std::isfinite(ruckig_outputs_[i].new_velocity[0]))
      {
        smooth_pos_[i] = q_ref[i];
        smooth_vel_[i] = 0.0;
      }
      else
      {
        smooth_pos_[i] = ruckig_outputs_[i].new_position[0];
        smooth_vel_[i] = ruckig_outputs_[i].new_velocity[0];
      }
    }
  }
  q_out = smooth_pos_;
  dq_out = smooth_vel_;
}

void DragTeleopController::publishTeleopStates(
  const rclcpp::Time & time,
  const std::vector<double> & q_state,
  const std::vector<double> & v_state,
  const std::vector<double> & tau_ext,
  const std::vector<double> & gripper_pos)
{
  const size_t n = joints_.size();
  // 本侧 14 关节值（按本侧关节名）
  std::map<std::string, double> pos_values, vel_values;
  for (size_t i = 0; i < n; ++i)
  {
    pos_values[joints_[i]] = q_state[i];
    vel_values[joints_[i]] = v_state[i];
  }
  for (size_t i = 0; i < gripper_joints_.size(); ++i)
  {
    pos_values[gripper_joints_[i]] = gripper_pos[i];
    vel_values[gripper_joints_[i]] = 0.0;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;
  msg.header.frame_id = role_;

  if (role_ == "master")
  {
    // 正映射：master → slave 参考系（slave 关节名）
    std::vector<double> mapped_pos, mapped_vel;
    if (!mapper_->map(pos_values, mapped_pos) ||
      !mapper_->map(vel_values, mapped_vel))
    {
      return;
    }
    msg.name = mapper_->slave_joints();
    msg.position = mapped_pos;
    msg.velocity = mapped_vel;
    msg.effort.assign(mapped_pos.size(), 0.0);
  }
  else
  {
    // 逆映射：slave → master 参考系（master 关节名）
    std::vector<double> mapped_pos, mapped_vel;
    if (!mapper_->inverse_map(pos_values, mapped_pos) ||
      !mapper_->inverse_map(vel_values, mapped_vel))
    {
      return;
    }
    msg.name = mapper_->master_joints();
    msg.position = mapped_pos;
    msg.velocity = mapped_vel;
    // effort：τ_ext（12 臂）→ 功率守恒映射到 master 参考系；夹爪 0
    msg.effort.assign(mapped_pos.size(), 0.0);
    std::vector<double> tau_ext_mapped;
    if (mapper_->map_effort(tau_ext, tau_ext_mapped))
    {
      for (size_t i = 0; i < tau_ext_mapped.size(); ++i)
      {
        msg.effort[i] = tau_ext_mapped[i];
      }
    }
  }
  state_pub_->publish(msg);
}

controller_interface::return_type DragTeleopController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = std::max(period.seconds(), 1.0e-4);
  const size_t n = joints_.size();

  // 1) 读本侧状态（12 臂 + 2 夹爪）
  std::vector<double> q_state(n), v_state(n), tau_state(n);
  for (size_t i = 0; i < n; ++i)
  {
    q_state[i] = position_state_[i].get().get_optional().value_or(0.0);
    v_state[i] = velocity_state_[i] ?
      velocity_state_[i]->get_optional().value_or(0.0) : 0.0;
    tau_state[i] = effort_state_[i] ?
      effort_state_[i]->get_optional().value_or(0.0) : 0.0;
  }
  std::vector<double> gripper_pos(gripper_joints_.size(), 0.0);
  for (size_t i = 0; i < gripper_joints_.size(); ++i)
  {
    gripper_pos[i] = gripper_position_state_[i].get().get_optional().value_or(0.0);
  }

  // 2) 拷贝对侧订阅数据（发布前已映射，接收方直接用）
  std::map<std::string, double> remote_pos, remote_vel, remote_eff;
  bool remote_ok = false;
  {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    remote_pos = remote_positions_;
    remote_vel = remote_velocities_;
    remote_eff = remote_efforts_;
    remote_ok = remote_received_;
  }

  // 3) 组装模型 q / v / a（q̈ 数值微分 + 低通）
  const auto & model = dynamics_->getModel();
  Eigen::VectorXd q_model = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v_model = Eigen::VectorXd::Zero(model.nv);
  for (size_t i = 0; i < n; ++i)
  {
    const size_t jid = model_joint_indices_[i];
    if (jid != std::numeric_limits<size_t>::max())
    {
      const auto & jm = model.joints[jid];
      q_model[jm.idx_q()] = q_state[i];
      v_model[jm.idx_v()] = v_state[i];
    }
  }
  Eigen::VectorXd a_model = Eigen::VectorXd::Zero(model.nv);
  if (accel_initialized_)
  {
    constexpr double kAccelAlpha = 0.2;  // 低通系数
    for (size_t i = 0; i < n; ++i)
    {
      const double raw = (v_state[i] - prev_velocity_[i]) / dt;
      accel_[i] = kAccelAlpha * raw + (1.0 - kAccelAlpha) * accel_[i];
      const size_t jid = model_joint_indices_[i];
      if (jid != std::numeric_limits<size_t>::max())
      {
        a_model[model.joints[jid].idx_v()] = accel_[i];
      }
    }
  }
  else
  {
    accel_initialized_ = true;
  }
  prev_velocity_ = v_state;

  // 4) 动力学：τ_G(q_state)、τ_model(q, v, a)
  const Eigen::VectorXd tau_G_state = dynamics_->calculateGravity(q_model);
  const Eigen::VectorXd tau_model = dynamics_->calculateModelTorques(
    q_model, v_model, a_model);
  std::vector<double> tau_G_state_vec(n), tau_model_vec(n);
  for (size_t i = 0; i < n; ++i)
  {
    const size_t jid = model_joint_indices_[i];
    if (jid != std::numeric_limits<size_t>::max())
    {
      const size_t idx_v = model.joints[jid].idx_v();
      tau_G_state_vec[i] = tau_G_state[idx_v];
      tau_model_vec[i] = tau_model[idx_v];
    }
  }

  // 5) 对侧参考（本侧关节顺序；对侧数据缺失时保位）
  std::vector<double> q_ref(n, 0.0), dq_ref(n, 0.0), tau_ext_ref(n, 0.0);
  bool ref_ok = remote_ok;
  if (ref_ok)
  {
    for (size_t i = 0; i < n; ++i)
    {
      const auto it = remote_pos.find(joints_[i]);
      if (it == remote_pos.end())
      {
        ref_ok = false;
        break;
      }
      q_ref[i] = it->second;
      const auto itv = remote_vel.find(joints_[i]);
      dq_ref[i] = (itv != remote_vel.end()) ? itv->second : 0.0;
      const auto ite = remote_eff.find(joints_[i]);
      tau_ext_ref[i] = (ite != remote_eff.end()) ? ite->second : 0.0;
    }
  }
  if (!ref_ok)
  {
    q_ref = q_state;
    dq_ref.assign(n, 0.0);
    tau_ext_ref.assign(n, 0.0);
  }

  // 6) 模式快照（服务可切换）
  ControlMode mode;
  FeedbackMode feedback;
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    mode = control_mode_;
    feedback = feedback_mode_;
  }

  // 7) 计算控制量
  ControlResult result;
  if (role_ == "slave")
  {
    // ruckig 平滑（可选）
    std::vector<double> q_cmd_ref = q_ref;
    std::vector<double> dq_cmd_ref = dq_ref;
    if (smooth_enabled_)
    {
      smoothReference(dt, q_state, v_state, q_ref, dq_ref, q_cmd_ref, dq_cmd_ref);
    }
    // τ_G(q_cmd)
    Eigen::VectorXd q_cmd_model = Eigen::VectorXd::Zero(model.nq);
    for (size_t i = 0; i < n; ++i)
    {
      const size_t jid = model_joint_indices_[i];
      if (jid != std::numeric_limits<size_t>::max())
      {
        q_cmd_model[model.joints[jid].idx_q()] = q_cmd_ref[i];
      }
    }
    const Eigen::VectorXd tau_G_cmd = dynamics_->calculateGravity(q_cmd_model);
    std::vector<double> tau_G_ref_vec(n, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
      const size_t jid = model_joint_indices_[i];
      if (jid != std::numeric_limits<size_t>::max())
      {
        tau_G_ref_vec[i] = tau_G_cmd[model.joints[jid].idx_v()];
      }
    }
    result = ControlCalculator::calculateSlave(
      mode, q_state, v_state, tau_state, q_cmd_ref, dq_cmd_ref,
      tau_G_state_vec, tau_G_ref_vec, tau_model_vec, impedance_params_,
      has_effort_state_);
  }
  else
  {
    result = ControlCalculator::calculateMaster(
      mode, feedback, q_state, v_state, q_ref, dq_ref, tau_ext_ref,
      tau_G_state_vec, feedback_params_, impedance_params_);
  }

  // 8) 写命令接口（缺失接口跳过）
  for (size_t i = 0; i < n; ++i)
  {
    if (position_cmd_[i])
    {
      std::ignore = position_cmd_[i]->set_value(result.q_cmd[i]);
    }
    if (velocity_cmd_[i])
    {
      std::ignore = velocity_cmd_[i]->set_value(result.dq_cmd[i]);
    }
    if (effort_cmd_[i])
    {
      double tau = result.tau_cmd[i];
      if (!max_effort_.empty())
      {
        const double limit = max_effort_[std::min(i, max_effort_.size() - 1)];
        tau = std::clamp(tau, -limit, limit);
      }
      std::ignore = effort_cmd_[i]->set_value(tau);
    }
  }

  // 9) 发布 teleop_states（14 关节，映射后）
  publishTeleopStates(time, q_state, v_state, result.tau_ext, gripper_pos);

  // 10) ocs2 发布（master + moveJ_pub）
  if (ocs2_pub_)
  {
    std::map<std::string, double> local_values;
    for (size_t i = 0; i < n; ++i)
    {
      local_values[joints_[i]] = q_state[i];
    }
    for (size_t i = 0; i < gripper_joints_.size(); ++i)
    {
      local_values[gripper_joints_[i]] = gripper_pos[i];
    }
    ocs2_pub_->update(local_values);
  }

  return controller_interface::return_type::OK;
}

}  // namespace drag_teleop_controller