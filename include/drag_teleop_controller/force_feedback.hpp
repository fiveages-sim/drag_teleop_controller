//
// Position-spring force feedback for drag teleop (master arm).
//
// When the slave arm is blocked by an obstacle and cannot follow the master,
// the master arm gets a restoring force. The controller subscribes to the
// inverse-mapped reference positions published by teleop-joint-mapper
// (slave positions expressed in the master frame), computes
//   e_i = q_master_i - q_ref_i
// and offsets the master position command by
//   Δq_i = -G · sat(e_i - δ),   q_cmd_i = q_master_i + Δq_i
// The motor position loop then produces the feedback torque τ = kp·Δq.
//
// Because drag mode uses a very low kp (≈0.01), the stiffness is boosted
// while feedback is active: a worker thread raises the hardware joint_kp
// parameter (via /controller_manager) to config.kp and restores the saved
// original values on exit (hardware refreshes gains every ~200ms).
//
#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace drag_teleop_controller
{

/// 力反馈参数（feedback.*，由控制器 on_init 声明后填充）。
struct ForceFeedbackConfig
{
  bool enabled{false};
  std::string joint_state_topic{"/joint_mapper/feedback_joint_state"};
  double input_timeout{0.2};   // 反馈数据超时（s），超时自动关闭反馈
  double gain{0.3};            // Δq = −G·(e−δ)
  double deadzone{0.02};       // 误差死区（rad），吸收主从静态偏差
  double kp{4.0};              // 反馈激活时硬件 kp 参数临时值
  double kd{0.0};              // <=0 = 不改 kd
  std::string kp_param_name{"joint_kp"};  // 硬件 kp 参数名（不同 hardware 可能不同）
  std::string kd_param_name{"joint_kd"};  // 硬件 kd 参数名
  double max_delta_q{0.5};     // Δq 限幅（rad），力矩上限 = kp×max_delta_q
  double delta_q_rate{2.0};    // Δq 斜坡速率（rad/s）
};

/// 位置弹簧力反馈（从臂碰撞回推主臂）。
///
/// 非 RT 部分（订阅回调、kp 状态机线程）与 RT 部分（update 计算 Δq）分离：
/// update() 只做锁内快照 + 纯数值计算，不分配、不发布（debug 发布节流在
/// update 内但仅拷贝 vector，可接受）；kp 提升走独立线程 + 参数服务。
class ForceFeedback
{
public:
  /// @param node 控制器节点（创建订阅/发布器；kp 线程内部另建独立节点）
  /// @param config 力反馈参数（feedback.*）
  /// @param joint_names 参与反馈的关节（与控制器 joint_names 一致）
  ForceFeedback(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ForceFeedbackConfig & config,
    const std::vector<std::string> & joint_names);

  /// 创建订阅/发布器并启动 kp 状态机线程（重复调用安全：先停旧线程）。
  void configure();
  /// 停止线程、清理订阅/发布器、复位状态（重复调用安全）。
  void deactivate();

  /// 每周期调用：计算 Δq（反馈未激活/数据超时则归零）。
  /// @param dt 控制周期（s）
  /// @param state_values 关节名 → 实测位置
  void update(
    double dt, const std::unordered_map<std::string, double> & state_values);

  /// 当前 Δq（与 joint_names 对齐，update 后有效）。
  const std::vector<double> & deltaQ() const { return delta_q_; }
  /// 反馈是否激活（enabled && 数据新鲜）。
  bool isActive() const { return feedback_active_.load(); }

private:
  void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void kpWorkerLoop();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  ForceFeedbackConfig config_;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_debug_pub_;
  unsigned int debug_counter_{0};

  std::mutex feedback_mutex_;
  std::map<std::string, double> feedback_positions_;  // 主臂参考系期望位置（逆映射）
  bool feedback_received_{false};
  std::chrono::steady_clock::time_point feedback_time_{};
  /// update() 写，kp 状态机线程读（atomic）
  std::atomic<bool> feedback_active_{false};
  std::vector<double> delta_q_;      // 当前 Δq（斜坡后）
  std::vector<double> prev_delta_q_; // 斜坡状态

  // ---- kp 状态机（非 RT 线程） ----
  std::string controller_manager_name_{"/controller_manager"};
  struct KpState
  {
    std::atomic<bool> active{false};
    std::atomic<bool> stop{false};
    bool original_saved{false};
    bool kp_is_array{true};   // 原 kp 参数类型（array 或标量），恢复时保持原类型
    bool kd_is_array{true};
    std::vector<double> original_kp;
    std::vector<double> original_kd;
    std::thread thread;
  };
  KpState kp_state_;
};

}  // namespace drag_teleop_controller