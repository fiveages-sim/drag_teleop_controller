//
// OCS2 moveJ / gripper command publisher (master side, moveJ_pub:=true).
//
// Publishes the forward-mapped (master -> slave frame) arm positions to the
// ocs2 moveJ topic (Float64MultiArray) and the mapped gripper positions to
// the per-joint position_command topics (Float64), so that a slave running
// the ocs2_arm_controller (moveJ mode) + adaptive_gripper_controller can
// follow the master.
//
// The mapping is applied here: the controller feeds the raw master-side
// 14-joint values (12 arms + 2 grippers) and this class maps them into the
// slave frame before publishing.
//
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "drag_teleop_controller/joint_mapper.hpp"

namespace drag_teleop_controller
{

/// moveJ 发布配置（master.ocs2_cmd.move_j 段）。
struct Ocs2MoveJConfig
{
  std::vector<std::string> joints;  // 12 臂关节（slave 参考系名，发布顺序）
  std::string cmd_topic;            // 如 /ocs2_arm_controller/target_joint_position
};

/// Ocs2Publisher 配置（master.ocs2_cmd 段，仅 moveJ）。
struct Ocs2PublisherConfig
{
  double pub_rate{100.0};
  Ocs2MoveJConfig move_j;
};

/// moveJ 命令发布器（仅 master 且 master.ocs2_cmd.enabled:=true 时启用）。
///
/// 非 RT 部分（定时器发布）与 RT 部分（update 缓存）分离：
/// update() 只做锁内快照 + 映射，定时器回调发布。
class Ocs2Publisher
{
public:
  /// @param node 控制器节点（创建发布器/定时器）
  /// @param config 发布配置（master.ocs2_cmd 段）
  /// @param mapper 关节映射器（14 关节，正映射 master -> slave 参考系）
  Ocs2Publisher(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Ocs2PublisherConfig & config,
    const JointMapper & mapper);

  /// 创建发布器与定时器（重复调用安全：先清理旧资源）。
  void configure();
  /// 清理发布器与定时器（重复调用安全）。
  void deactivate();

  /// 每周期调用：master 侧 14 关节值（按 master 关节名索引），
  /// 内部做正映射并缓存，定时器回调发布。
  void update(const std::map<std::string, double> & master_values);

private:
  void timerCallback();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  Ocs2PublisherConfig config_;
  const JointMapper & mapper_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr move_j_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::vector<double> move_j_data_;   // 12 维（slave 参考系，move_j.joints 顺序）
  bool data_valid_{false};
};

}  // namespace drag_teleop_controller