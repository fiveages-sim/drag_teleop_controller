//
// ForceFeedback implementation.
//
#include "drag_teleop_controller/force_feedback.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/parameter_client.hpp>

namespace drag_teleop_controller
{

ForceFeedback::ForceFeedback(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ForceFeedbackConfig & config,
  const std::vector<std::string> & joint_names)
: node_(std::move(node)), config_(config), joint_names_(joint_names)
{
}

void ForceFeedback::configure()
{
  delta_q_.assign(joint_names_.size(), 0.0);
  prev_delta_q_.assign(joint_names_.size(), 0.0);

  // 重复 configure（cleanup -> configure）时先停掉旧线程
  if (kp_state_.thread.joinable())
  {
    kp_state_.stop = true;
    kp_state_.thread.join();
  }
  kp_state_.stop = false;
  kp_state_.active = false;
  kp_state_.original_saved = false;

  if (!config_.enabled)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Force feedback disabled (feedback.enabled=false); position hold only");
    return;
  }

  feedback_subscription_ =
    node_->create_subscription<sensor_msgs::msg::JointState>(
      config_.joint_state_topic,
      rclcpp::QoS(10).reliable().transient_local(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message)
      { feedbackCallback(message); });
  feedback_debug_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "feedback_delta_q_debug", 10);
  RCLCPP_INFO(
    node_->get_logger(),
    "Force feedback enabled: subscribing '%s', "
    "gain=%.2f deadzone=%.3f max_delta_q=%.2f rate=%.2f, kp->%.2f",
    config_.joint_state_topic.c_str(),
    config_.gain, config_.deadzone, config_.max_delta_q,
    config_.delta_q_rate, config_.kp);

  // controller_manager 与控制器同 namespace（launch 设置）
  const std::string ns = node_->get_namespace();
  controller_manager_name_ =
    (ns.empty() || ns == "/") ? "/controller_manager" : ns + "/controller_manager";
  // kp 状态机线程：独立节点 + 自建 executor（SyncParametersClient 不能挂在
  // CM executor 的生命周期节点上，参考 gravity_compensation URDF probe 模式）
  kp_state_.thread = std::thread(&ForceFeedback::kpWorkerLoop, this);
}

void ForceFeedback::deactivate()
{
  // 停止 kp 状态机线程并恢复原 kp（尽力：线程退出前会走 ACTIVE->INACTIVE 恢复）
  if (kp_state_.thread.joinable())
  {
    kp_state_.stop = true;
    kp_state_.thread.join();
  }
  kp_state_.active = false;
  kp_state_.original_saved = false;

  feedback_subscription_.reset();
  feedback_debug_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_positions_.clear();
    feedback_received_ = false;
  }
  feedback_active_.store(false);
  std::fill(delta_q_.begin(), delta_q_.end(), 0.0);
  std::fill(prev_delta_q_.begin(), prev_delta_q_.end(), 0.0);
}

void ForceFeedback::feedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  feedback_positions_.clear();
  for (size_t i = 0; i < message->name.size() && i < message->position.size(); ++i)
  {
    if (std::isfinite(message->position[i]))
    {
      feedback_positions_[message->name[i]] = message->position[i];
    }
  }
  feedback_received_ = true;
  feedback_time_ = std::chrono::steady_clock::now();
}

void ForceFeedback::kpWorkerLoop()
{
  // SyncParametersClient 构造函数内部会创建自己的 SingleThreadedExecutor 并
  // 加入传入的节点（控制器节点运行在 CM executor 中，不能直接使用，因此这里
  // 用独立节点；不要再手动创建 executor，否则节点会被加入两个 executor 报错）。
  auto client_node = std::make_shared<rclcpp::Node>("drag_teleop_kp_client");
  auto client = std::make_shared<rclcpp::SyncParametersClient>(
    client_node, controller_manager_name_);

  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Feedback: parameter services of '%s' not available within 2s; "
      "kp stays unchanged (feedback still active, but stiffness is not boosted)",
      controller_manager_name_.c_str());
    return;
  }

  while (!kp_state_.stop.load())
  {
    const bool should = feedback_active_.load();
    if (should != kp_state_.active.load())
    {
      if (should)
      {
        // 激活：先读回原 kp/kd 保存（兼容任意拖动配置与参数类型），再提升到 config_.kp
        try
        {
          kp_state_.original_kp.clear();
          kp_state_.original_kd.clear();
          kp_state_.original_saved = false;
          // 参数名可配置（不同 hardware 可能不同）；类型自动判断：
          // double_array（每关节一个值）或 double（全局标量），恢复时保持原类型。
          if (client->has_parameter(config_.kp_param_name))
          {
            const auto params = client->get_parameters({config_.kp_param_name});
            if (!params.empty())
            {
              const auto & p = params[0];
              if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
              {
                kp_state_.original_kp = p.as_double_array();
                kp_state_.kp_is_array = true;
              }
              else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              {
                kp_state_.original_kp = std::vector<double>{p.as_double()};
                kp_state_.kp_is_array = false;
              }
            }
          }
          if (config_.kd > 0.0 && client->has_parameter(config_.kd_param_name))
          {
            const auto params = client->get_parameters({config_.kd_param_name});
            if (!params.empty())
            {
              const auto & p = params[0];
              if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
              {
                kp_state_.original_kd = p.as_double_array();
                kp_state_.kd_is_array = true;
              }
              else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              {
                kp_state_.original_kd = std::vector<double>{p.as_double()};
                kp_state_.kd_is_array = false;
              }
            }
          }
          if (!kp_state_.original_kp.empty())
          {
            std::vector<rclcpp::Parameter> params;
            if (kp_state_.kp_is_array)
            {
              params.emplace_back(
                config_.kp_param_name,
                std::vector<double>(kp_state_.original_kp.size(), config_.kp));
            }
            else
            {
              params.emplace_back(config_.kp_param_name, config_.kp);
            }
            if (config_.kd > 0.0 && !kp_state_.original_kd.empty())
            {
              if (kp_state_.kd_is_array)
              {
                params.emplace_back(
                  config_.kd_param_name,
                  std::vector<double>(kp_state_.original_kd.size(), config_.kd));
              }
              else
              {
                params.emplace_back(config_.kd_param_name, config_.kd);
              }
            }
            const auto results = client->set_parameters(params);
            const bool ok = std::all_of(
              results.begin(), results.end(),
              [](const rcl_interfaces::msg::SetParametersResult & r)
              { return r.successful; });
            kp_state_.original_saved = ok;
            RCLCPP_INFO(
              node_->get_logger(),
              "Feedback ON: %s -> %.2f (%s; hardware applies in ~200ms)",
              config_.kp_param_name.c_str(), config_.kp,
              ok ? "saved original for restore" : "SET FAILED");
          }
          else
          {
            RCLCPP_WARN(
              node_->get_logger(),
              "Feedback ON: '%s' not found on '%s' (or not a number); proceeding "
              "without kp boost (feedback force will be weak)",
              config_.kp_param_name.c_str(), controller_manager_name_.c_str());
          }
        }
        catch (const std::exception & e)
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "Feedback ON: failed to set %s: %s",
            config_.kp_param_name.c_str(), e.what());
        }
      }
      else
      {
        // 关闭：恢复保存的原 kp/kd（保持原参数类型）
        if (kp_state_.original_saved)
        {
          try
          {
            std::vector<rclcpp::Parameter> params;
            if (kp_state_.kp_is_array)
            {
              params.emplace_back(config_.kp_param_name, kp_state_.original_kp);
            }
            else
            {
              params.emplace_back(
                config_.kp_param_name, kp_state_.original_kp.front());
            }
            if (!kp_state_.original_kd.empty())
            {
              if (kp_state_.kd_is_array)
              {
                params.emplace_back(config_.kd_param_name, kp_state_.original_kd);
              }
              else
              {
                params.emplace_back(
                  config_.kd_param_name, kp_state_.original_kd.front());
              }
            }
            client->set_parameters(params);
            RCLCPP_INFO(
              node_->get_logger(),
              "Feedback OFF: %s restored (hardware applies in ~200ms)",
              config_.kp_param_name.c_str());
          }
          catch (const std::exception & e)
          {
            RCLCPP_WARN(
              node_->get_logger(),
              "Feedback OFF: failed to restore %s: %s",
              config_.kp_param_name.c_str(), e.what());
          }
        }
        kp_state_.original_saved = false;
      }
      kp_state_.active.store(should);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void ForceFeedback::update(
  double dt, const std::unordered_map<std::string, double> & state_values)
{
  // e = q_master − q_ref（q_ref 来自 mapper 逆映射，主臂参考系下的从臂实际位置）。
  // 碰撞时从臂跟不上 → e 增大 → Δq = −G·sat(e−δ) 反向偏移位置命令 →
  // 电机位置环产生 kp·Δq 的反向力矩（往主臂回来的方向拉）。
  // 反馈不激活（feedback.enabled=false / 反馈话题超时）时 Δq=0，
  // position 保持当前实测（与旧版保位语义完全一致）。
  bool fb_on = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const auto now = std::chrono::steady_clock::now();
    fb_on =
      config_.enabled && feedback_received_ &&
      std::chrono::duration<double>(now - feedback_time_).count() <
        config_.input_timeout;
    if (fb_on)
    {
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        double e = 0.0;
        const auto it = feedback_positions_.find(joint_names_[i]);
        if (it != feedback_positions_.end())
        {
          const auto state_it = state_values.find(joint_names_[i]);
          if (state_it != state_values.end())
          {
            e = state_it->second - it->second;
          }
        }
        double target = 0.0;
        const double abs_e = std::abs(e);
        if (abs_e > config_.deadzone)
        {
          target =
            -config_.gain * (abs_e - config_.deadzone) * (e > 0.0 ? 1.0 : -1.0);
        }
        target =
          std::clamp(target, -config_.max_delta_q, config_.max_delta_q);
        // 斜坡速率限制：防止误差突变时位置命令跳变猛推手臂
        const double step = config_.delta_q_rate * dt;
        double dq = prev_delta_q_[i];
        dq += std::clamp(target - dq, -step, step);
        prev_delta_q_[i] = dq;
        delta_q_[i] = dq;
      }
    }
  }
  feedback_active_.store(fb_on);
  if (!fb_on)
  {
    // 反馈关闭/超时：Δq 归零（kp 已恢复低值，位置回到实测无冲击）
    std::fill(prev_delta_q_.begin(), prev_delta_q_.end(), 0.0);
    std::fill(delta_q_.begin(), delta_q_.end(), 0.0);
  }

  // debug 发布（节流 50Hz）：Δq 即反馈量（力矩 = 当前 kp × Δq，由电机内部产生）
  if (feedback_debug_pub_ && (++debug_counter_ % 10 == 0))
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = delta_q_;
    feedback_debug_pub_->publish(msg);
  }
}

}  // namespace drag_teleop_controller