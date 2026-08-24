//
// Drag teleop controller (master / slave role).
//
// A single controller plugin that runs as either the master (dragged by the
// operator, gravity compensated) or the slave (follows the master) role.
//
// Architecture:
//   - The controller claims the 12 arm joints (position/velocity/effort
//     states + commands) plus the 2 gripper position states (read-only, for
//     mapped state publishing). Gripper commands are handled by
//     adaptive_gripper_controller (spawned by the launch file).
//   - Teleop states are mapped BEFORE publishing: the master publishes
//     forward-mapped states (slave joint names), the slave publishes
//     inverse-mapped states (master joint names), so the receiving side can
//     use them directly.
//   - The remote teleop_states topic is subscribed (input_topic, absolute
//     name) and used as the reference (q_ref / dq_ref / tau_ext_ref).
//   - Control quantities are computed by ControlCalculator (pure functions)
//     for every (role x mode x feedback) combination.
//   - The master can additionally publish ocs2 moveJ + gripper commands
//     (moveJ_pub:=true) via Ocs2Publisher.
//   - The slave can smooth the reference with Ruckig (slave.smooth).
//
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ruckig/ruckig.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "drag_teleop_controller/control_calculator.hpp"
#include "drag_teleop_controller/dynamics.hpp"
#include "drag_teleop_controller/joint_mapper.hpp"
#include "drag_teleop_controller/ocs2_publisher.hpp"
#include "drag_teleop_controller/srv/teleop_feedback.hpp"
#include "drag_teleop_controller/srv/teleop_mode.hpp"

namespace drag_teleop_controller
{

class DragTeleopController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DragTeleopController)

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool loadRobotDescription(std::string & urdf_out);
  void remoteStateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void modeServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<drag_teleop_controller::srv::TeleopMode::Request> request,
    const std::shared_ptr<drag_teleop_controller::srv::TeleopMode::Response> response);
  void feedbackServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<drag_teleop_controller::srv::TeleopFeedback::Request> request,
    const std::shared_ptr<drag_teleop_controller::srv::TeleopFeedback::Response> response);
  /// slave 参考平滑（ruckig，逐关节）
  void smoothReference(
    double dt,
    const std::vector<double> & q_state,
    const std::vector<double> & v_state,
    const std::vector<double> & q_ref,
    const std::vector<double> & dq_ref,
    std::vector<double> & q_out,
    std::vector<double> & dq_out);
  /// 发布 teleop_states（14 关节，映射后）
  void publishTeleopStates(
    const rclcpp::Time & time,
    const std::vector<double> & q_state,
    const std::vector<double> & v_state,
    const std::vector<double> & tau_ext,
    const std::vector<double> & gripper_pos);

  // ---- 参数（on_init 声明） ----
  std::string role_{"master"};          // master | slave
  std::string mode_str_{"mix"};         // position | mix | effort
  std::string feedback_str_{"false"};   // false | position | effort
  std::string input_topic_{"/drag_teleop_slave/teleop_states"};
  bool moveJ_pub_{false};
  std::vector<double> gravity_vector_{0.0, 0.0, -9.81};
  std::string urdf_param_name_{"robot_description"};

  // master 段
  std::vector<std::string> master_joints_;
  std::vector<double> master_max_effort_;
  std::string master_publish_topic_{"teleop_states"};
  FeedbackParams feedback_params_;
  Ocs2PublisherConfig ocs2_config_;
  std::vector<std::string> gripper_joints_param_;  // ocs2_cmd.gripper_joints
  std::vector<std::string> gripper_topics_param_;  // ocs2_cmd.gripper_topics

  // slave 段
  std::vector<std::string> slave_joints_;
  std::vector<double> slave_max_effort_;
  std::string slave_publish_topic_{"teleop_states"};
  ImpedanceParams impedance_params_;
  bool smooth_enabled_{false};
  double smooth_max_velocity_{3.0};
  double smooth_max_acceleration_{30.0};
  double smooth_max_jerk_{300.0};

  // mapper 段
  JointMappingConfig mapper_config_;

  // ---- 解析后的本侧配置（on_configure 填充） ----
  std::vector<std::string> joints_;        // 本侧 12 臂关节
  std::vector<double> max_effort_;         // 本侧力矩限幅
  std::string publish_topic_;              // 本侧发布话题（相对名）
  std::vector<std::string> gripper_joints_;  // 本侧夹爪关节（只读状态，2 个）

  // ---- 模式（服务可切换，mutex 保护） ----
  std::mutex mode_mutex_;
  ControlMode control_mode_{ControlMode::Mix};
  FeedbackMode feedback_mode_{FeedbackMode::None};

  // ---- 映射 / 动力学 ----
  std::unique_ptr<JointMapper> mapper_;
  std::unique_ptr<Dynamics> dynamics_;
  // joints_[i] 在模型中的 JointIndex（SIZE_MAX = 模型缺失，力矩置 0）
  std::vector<size_t> model_joint_indices_;

  // ---- 借用的硬件接口（on_activate 时填充） ----
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    position_state_;
  // velocity/effort 状态可选（缺失为 nullptr，读取时置 0）
  std::vector<hardware_interface::LoanedStateInterface *> velocity_state_;
  std::vector<hardware_interface::LoanedStateInterface *> effort_state_;
  std::vector<hardware_interface::LoanedCommandInterface *> position_cmd_;
  std::vector<hardware_interface::LoanedCommandInterface *> velocity_cmd_;
  std::vector<hardware_interface::LoanedCommandInterface *> effort_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    gripper_position_state_;
  // 本侧是否有 effort 状态接口（无则 τ_ext 用 τ_cmd 代替）
  bool has_effort_state_{true};

  // ---- 订阅 / 发布 ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  std::mutex remote_mutex_;
  std::map<std::string, double> remote_positions_;
  std::map<std::string, double> remote_velocities_;
  std::map<std::string, double> remote_efforts_;
  bool remote_received_{false};

  // ---- 服务 ----
  rclcpp::Service<drag_teleop_controller::srv::TeleopMode>::SharedPtr mode_service_;
  rclcpp::Service<drag_teleop_controller::srv::TeleopFeedback>::SharedPtr feedback_service_;

  // ---- ocs2 发布（master + moveJ_pub） ----
  std::unique_ptr<Ocs2Publisher> ocs2_pub_;

  // ---- ruckig 平滑（slave + smooth.enabled） ----
  std::vector<ruckig::InputParameter<1>> ruckig_inputs_;
  std::vector<ruckig::OutputParameter<1>> ruckig_outputs_;
  std::vector<double> smooth_pos_;  // 平滑后位置（12）
  std::vector<double> smooth_vel_;  // 平滑后速度（12）
  bool smooth_initialized_{false};

  // ---- q̈ 数值微分（低通） ----
  std::vector<double> prev_velocity_;
  std::vector<double> accel_;
  bool accel_initialized_{false};
};

}  // namespace drag_teleop_controller