//
// Gravity compensation controller for Panthera HT (single / dual arm).
//
// Architecture (matches the ht_ros2_control hardware interface contract):
//   - For each joint in `joints` (arm joints): claim the position state
//     interface (required) and, when available, the position/velocity/effort
//     command interfaces (all optional). Every update cycle:
//       * effort = static gravity torque from Pinocchio RNEA (clamped);
//         if the effort command interface is missing, a warning is logged
//         once and the controller keeps running (position-hold only)
//       * position = current measured position (hold in place, prevents the
//         hardware interface from treating an unclaimed/zero position command
//         as a move-to-zero target)
//       * velocity = 0 (no velocity feedforward)
//     (kp/kd are managed by the hardware interface as ROS parameters, rqt
//     tunable; this controller never writes them.)
//   - For each joint in `hold_joints` (e.g. gripper): position state
//     (required) + position command (optional), following the current value
//     (keep in place).
//
// Command interfaces are optional: command_interface_configuration() returns
// ALL (claiming only the interfaces the hardware actually exports), and
// on_activate() picks position/velocity/effort by name. Declaring a missing
// interface with INDIVIDUAL would make activation fail (claim throws).
//
// The URDF is read from the `urdf_param` parameter (default `robot_description`,
// injected by controller_manager or read from /controller_manager).
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"

#include "drag_teleop_controller/force_feedback.hpp"
#include "drag_teleop_controller/gravity_compensation.hpp"

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

  // ---- 参数 ----
  std::vector<std::string> joint_names_;      // 重力补偿关节（臂关节）
  std::vector<std::string> hold_joint_names_; // 仅位置保持关节（夹爪等）
  std::vector<double> max_effort_;            // 力矩限幅（Nm），空 = 不限幅
  std::vector<double> gravity_vector_{0.0, 0.0, -9.81};
  std::string urdf_param_name_{"robot_description"};

  // ---- 力反馈参数（feedback.*） ----
  bool feedback_enabled_{false};
  std::string feedback_joint_state_topic_{"/joint_mapper/feedback_joint_state"};
  double feedback_input_timeout_{0.2};
  double feedback_gain_{0.3};        // Δq = −G·e（无量纲，等效刚度 = kp×G）
  double feedback_deadzone_{0.02};   // |e| 死区（rad），吸收零点偏置/噪声
  double feedback_kp_{4.0};          // 反馈激活时关节 kp 目标
  double feedback_kd_{0.0};          // <=0 = 不改变 kd
  std::string feedback_kp_param_name_{"joint_kp"}; // 硬件 kp 参数名（不同 hardware 可能不同）
  std::string feedback_kd_param_name_{"joint_kd"}; // 硬件 kd 参数名
  double feedback_max_delta_q_{0.5}; // Δq 限幅（rad），力矩上限 = kp×max_delta_q
  double feedback_delta_q_rate_{2.0}; // Δq 斜坡速率（rad/s）

  // ---- 动力学 ----
  std::unique_ptr<GravityCompensation> gravity_;
  // joint_names_[i] 在模型中的 JointIndex（SIZE_MAX = 模型缺失，力矩置 0）
  std::vector<size_t> model_joint_indices_;

  // ---- 借用的硬件接口（on_activate 时填充） ----
  // 状态接口必需（reference_wrapper）；命令接口可选（原始指针，缺失为
  // nullptr，update 中跳过写入）。指针在 activate 期间有效（LoanedCommandInterface
  // 由 controller_manager 管理生命周期）。
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    hold_position_state_interface_;
  std::vector<hardware_interface::LoanedCommandInterface *> joint_position_command_interface_;
  std::vector<hardware_interface::LoanedCommandInterface *> joint_velocity_command_interface_;
  std::vector<hardware_interface::LoanedCommandInterface *> joint_effort_command_interface_;
  std::vector<hardware_interface::LoanedCommandInterface *> hold_position_command_interface_;

  // effort 命令接口缺失时仅警告一次（每次 activate 重置）
  bool effort_missing_warned_{false};

  // ---- 力反馈（位置弹簧，实现见 force_feedback.hpp/.cpp） ----
  std::unique_ptr<ForceFeedback> force_feedback_;
};

}  // namespace drag_teleop_controller
