//
// ControlCalculator implementation.
//
#include "drag_teleop_controller/control_calculator.hpp"

#include <algorithm>
#include <cmath>

namespace drag_teleop_controller
{
namespace
{

/// 逐关节饱和（限幅）。
double sat(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

/// 死区处理：|e| <= dz → 0，否则保留符号的 (|e| - dz)。
double applyDeadZone(double e, double dz)
{
  const double abs_e = std::abs(e);
  if (abs_e <= dz)
  {
    return 0.0;
  }
  return e > 0.0 ? (abs_e - dz) : -(abs_e - dz);
}

/// 位置误差 → Δq（带死区与限幅）：Δq = -G * sat(e_dz, max_delta_q)。
double positionDeltaQ(double e, double gain, double dead_zone, double max_delta_q)
{
  return -gain * sat(applyDeadZone(e, dead_zone), max_delta_q);
}

}  // namespace

ControlResult ControlCalculator::calculateSlave(
  ControlMode mode,
  const std::vector<double> & q_state,
  const std::vector<double> & dq_state,
  const std::vector<double> & tau_state,
  const std::vector<double> & q_ref,
  const std::vector<double> & dq_ref,
  const std::vector<double> & tau_G_state,
  const std::vector<double> & tau_G_ref,
  const std::vector<double> & tau_model,
  const ImpedanceParams & imp,
  bool has_effort_state)
{
  const size_t n = q_state.size();
  (void)tau_G_state;  // slave 使用 τ_G(q_cmd)（tau_G_ref），本参数保留接口一致性
  ControlResult r;
  r.q_cmd.assign(n, 0.0);
  r.dq_cmd.assign(n, 0.0);
  r.tau_imp.assign(n, 0.0);
  r.tau_cmd.assign(n, 0.0);
  r.tau_model = tau_model;
  r.tau_ext.assign(n, 0.0);
  r.delta_q.assign(n, 0.0);

  // 外部力矩估计：τ_ext = τ_state - τ_model
  // （无 effort 状态接口时用 τ_cmd 代替 τ_state，τ_cmd 在下方计算后回填）
  // 注意：本函数保持纯函数，τ_ext 的一阶低通（α=0.02）由
  // DragTeleopController::update 在拿到结果后统一施加。
  auto compute_tau_ext = [&]()
  {
    const std::vector<double> & tau_ref =
      has_effort_state ? tau_state : r.tau_cmd;
    for (size_t i = 0; i < n; ++i)
    {
      r.tau_ext[i] = tau_ref[i] - tau_model[i];
    }
  };

  switch (mode)
  {
    case ControlMode::Position:
      // 仅位置控制：q_cmd = q_ref；velocity/effort 不写
      r.q_cmd = q_ref;
      r.dq_cmd.assign(n, 0.0);
      r.tau_cmd.assign(n, 0.0);
      compute_tau_ext();
      break;

    case ControlMode::Mit:
      // 位置 + 速度 + 重力补偿力矩
      r.q_cmd = q_ref;
      r.dq_cmd = dq_ref;
      r.tau_cmd = tau_G_ref;  // τ_G(q_cmd)
      compute_tau_ext();
      break;

    case ControlMode::Effort:
      // 阻抗 + 重力补偿
      r.q_cmd = q_ref;
      r.dq_cmd = dq_ref;
      for (size_t i = 0; i < n; ++i)
      {
        const double kp = i < imp.kp.size() ? imp.kp[i] : 0.0;
        const double kd = i < imp.kd.size() ? imp.kd[i] : 0.0;
        r.tau_imp[i] =
          kp * (q_ref[i] - q_state[i]) + kd * (dq_ref[i] - dq_state[i]);
        r.tau_cmd[i] = r.tau_imp[i] + tau_G_ref[i];
      }
      compute_tau_ext();
      break;
  }

  return r;
}

ControlResult ControlCalculator::calculateMaster(
  ControlMode mode,
  FeedbackMode feedback,
  const std::vector<double> & q_state,
  const std::vector<double> & dq_state,
  const std::vector<double> & q_ref,
  const std::vector<double> & dq_ref,
  const std::vector<double> & tau_ext_ref,
  const std::vector<double> & tau_G_state,
  const FeedbackParams & fb,
  const ImpedanceParams & imp)
{
  (void)dq_state;
  (void)dq_ref;
  const size_t n = q_state.size();
  ControlResult r;
  r.q_cmd.assign(n, 0.0);
  r.dq_cmd.assign(n, 0.0);
  r.tau_imp.assign(n, 0.0);
  r.tau_cmd.assign(n, 0.0);
  r.tau_model.assign(n, 0.0);
  r.tau_ext.assign(n, 0.0);
  r.delta_q.assign(n, 0.0);

  // 位置误差力反馈 Δq（position 反馈共用）
  if (feedback == FeedbackMode::Position)
  {
    for (size_t i = 0; i < n; ++i)
    {
      const double gain = i < fb.gain.size() ? fb.gain[i] : 0.0;
      const double dz = i < fb.dead_zone.size() ? fb.dead_zone[i] : 0.0;
      const double max_dq = i < fb.max_delta_q.size() ? fb.max_delta_q[i] : 0.0;
      r.delta_q[i] = positionDeltaQ(q_state[i] - q_ref[i], gain, dz, max_dq);
    }
  }

  switch (mode)
  {
    case ControlMode::Position:
      // master 不支持 position 模式（仅 slave）；按 mit 语义保位
      r.q_cmd = q_state;
      r.dq_cmd.assign(n, 0.0);
      r.tau_cmd = tau_G_state;
      break;

    case ControlMode::Mit:
      // 位置命令 = 当前 + Δq（位置弹簧由硬件位置环实现）
      r.q_cmd = q_state;
      r.dq_cmd.assign(n, 0.0);
      if (feedback == FeedbackMode::Position)
      {
        for (size_t i = 0; i < n; ++i)
        {
          r.q_cmd[i] += r.delta_q[i];
        }
      }
      r.tau_cmd = tau_G_state;
      break;

    case ControlMode::Effort:
      // 位置命令保位（防硬件回零）
      r.q_cmd = q_state;
      r.dq_cmd.assign(n, 0.0);
      if (feedback == FeedbackMode::Position)
      {
        // τ_imp = Kp * Δq
        for (size_t i = 0; i < n; ++i)
        {
          const double kp = i < imp.kp.size() ? imp.kp[i] : 0.0;
          r.tau_imp[i] = kp * r.delta_q[i];
          r.tau_cmd[i] = r.tau_imp[i] + tau_G_state[i];
        }
      }
      else if (feedback == FeedbackMode::Effort)
      {
        // τ_cmd = -G * sat(τ_ext_ref, max_ext_effort) + τ_G(q_state)
        for (size_t i = 0; i < n; ++i)
        {
          const double gain = i < fb.effort_gain.size() ? fb.effort_gain[i] : 0.0;
          const double max_ext = i < fb.max_ext_effort.size() ? fb.max_ext_effort[i] : 0.0;
          const double tau_fb = -gain * sat(tau_ext_ref[i], max_ext);
          r.tau_cmd[i] = tau_fb + tau_G_state[i];
        }
      }
      else
      {
        r.tau_cmd = tau_G_state;
      }
      break;
  }

  return r;
}

bool ControlCalculator::parseMode(const std::string & s, ControlMode & out)
{
  if (s == "position")
  {
    out = ControlMode::Position;
    return true;
  }
  if (s == "mit" || s == "mix") // "mix" 为旧名，向后兼容
  {
    out = ControlMode::Mit;
    return true;
  }
  if (s == "effort")
  {
    out = ControlMode::Effort;
    return true;
  }
  return false;
}

bool ControlCalculator::parseFeedback(const std::string & s, FeedbackMode & out)
{
  if (s == "false" || s == "none")
  {
    out = FeedbackMode::None;
    return true;
  }
  if (s == "position")
  {
    out = FeedbackMode::Position;
    return true;
  }
  if (s == "effort")
  {
    out = FeedbackMode::Effort;
    return true;
  }
  return false;
}

std::string ControlCalculator::modeName(ControlMode mode)
{
  switch (mode)
  {
    case ControlMode::Position:
      return "position";
    case ControlMode::Mit:
      return "mit";
    case ControlMode::Effort:
      return "effort";
  }
  return "unknown";
}

std::string ControlCalculator::feedbackName(FeedbackMode mode)
{
  switch (mode)
  {
    case FeedbackMode::None:
      return "false";
    case FeedbackMode::Position:
      return "position";
    case FeedbackMode::Effort:
      return "effort";
  }
  return "unknown";
}

}  // namespace drag_teleop_controller