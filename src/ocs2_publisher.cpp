//
// Ocs2Publisher implementation.
//
#include "drag_teleop_controller/ocs2_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace drag_teleop_controller
{

Ocs2Publisher::Ocs2Publisher(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const Ocs2PublisherConfig & config,
  const JointMapper & mapper)
: node_(std::move(node)), config_(config), mapper_(mapper)
{
}

void Ocs2Publisher::configure()
{
  deactivate();

  move_j_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    config_.move_j.cmd_topic, 10);
  gripper_pubs_.clear();
  gripper_pubs_.reserve(config_.gripper.size());
  for (const auto & g : config_.gripper)
  {
    gripper_pubs_.push_back(
      node_->create_publisher<std_msgs::msg::Float64>(g.cmd_topic, 10));
  }
  move_j_data_.assign(config_.move_j.joints.size(), 0.0);
  gripper_data_.assign(config_.gripper.size(), 0.0);
  data_valid_ = false;

  const double rate = std::clamp(config_.pub_rate, 1.0, 1000.0);
  timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&Ocs2Publisher::timerCallback, this));

  RCLCPP_INFO(
    node_->get_logger(),
    "Ocs2Publisher: moveJ -> '%s' (%zu joints) @ %.1f Hz, %zu gripper topic(s)",
    config_.move_j.cmd_topic.c_str(), config_.move_j.joints.size(), rate,
    gripper_pubs_.size());
}

void Ocs2Publisher::deactivate()
{
  if (timer_)
  {
    timer_->cancel();
    timer_.reset();
  }
  move_j_pub_.reset();
  gripper_pubs_.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_valid_ = false;
  }
}

void Ocs2Publisher::update(
  const std::map<std::string, double> & master_values)
{
  // 正映射：master 参考系 → slave 参考系（14 关节，slave_joints 顺序）
  std::vector<double> mapped;
  if (!mapper_.map(master_values, mapped))
  {
    return;
  }
  // 构建 slave 参考系名 → 值
  std::map<std::string, double> slave_values;
  const auto & slave_joints = mapper_.slave_joints();
  for (size_t i = 0; i < slave_joints.size() && i < mapped.size(); ++i)
  {
    slave_values[slave_joints[i]] = mapped[i];
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // moveJ：按 move_j.joints（slave 臂关节名）顺序提取
  for (size_t i = 0; i < config_.move_j.joints.size(); ++i)
  {
    const auto found = slave_values.find(config_.move_j.joints[i]);
    if (found == slave_values.end())
    {
      return;
    }
    move_j_data_[i] = found->second;
  }
  // gripper：master 侧夹爪名 → mapper 索引 → slave 参考系名
  const auto & master_joints = mapper_.master_joints();
  for (size_t i = 0; i < config_.gripper.size(); ++i)
  {
    const auto it = std::find(
      master_joints.begin(), master_joints.end(), config_.gripper[i].joint);
    if (it == master_joints.end())
    {
      return;
    }
    const size_t idx = static_cast<size_t>(it - master_joints.begin());
    const auto found = slave_values.find(slave_joints[idx]);
    if (found == slave_values.end())
    {
      return;
    }
    gripper_data_[i] = found->second;
  }
  data_valid_ = true;
}

void Ocs2Publisher::timerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!data_valid_ || !move_j_pub_)
  {
    return;
  }
  std_msgs::msg::Float64MultiArray move_j_msg;
  move_j_msg.data = move_j_data_;
  move_j_pub_->publish(move_j_msg);

  for (size_t i = 0; i < gripper_pubs_.size(); ++i)
  {
    std_msgs::msg::Float64 gripper_msg;
    gripper_msg.data = gripper_data_[i];
    gripper_pubs_[i]->publish(gripper_msg);
  }
}

}  // namespace drag_teleop_controller