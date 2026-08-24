//
// Control quantity calculation for drag teleop (pure functions).
//
// Computes the command quantities for every (role x mode x feedback)
// combination defined in TODO.md. All vectors are 12-D (arm joints only;
// grippers are handled by adaptive_gripper_controller).
//
// Reference formulas (TODO.md):
//   slave-position:
//     tau_model = M(q)a + C(q,v)v + G(q)
//     tau_ext   = tau_state - tau_model
//     q_cmd     = q_ref
//   slave-mix:
//     tau_model / tau_ext as above
//     q_cmd  = q_ref
//     dq_cmd = dq_ref
//     tau_cmd = tau_G(q_cmd)
//   slave-effort:
//     tau_imp = Kp(q_ref - q_state) + Kd(dq_ref - dq_state)
//     tau_cmd = tau_imp + tau_G(q_cmd)
//     tau_ext = (has_effort_state ? tau_state : tau_cmd) - tau_model
//   master-*-feedback(false):
//     q_cmd = q_state, dq_cmd = 0, tau_cmd = tau_G(q_state)
//   master-mix-feedback(position):
//     delta_q = -G * sat(e - dead_zone, max_delta_q), e = q_state - q_ref
//     q_cmd = q_state + delta_q, dq_cmd = 0, tau_cmd = tau_G(q_state)
//   master-effort-feedback(position):
//     delta_q as above; tau_imp = Kp * delta_q
//     tau_cmd = tau_imp + tau_G(q_state), q_cmd = q_state (hold)
//   master-effort-feedback(effort):
//     tau_cmd = -G * sat(tau_ext_ref, max_ext_effort) + tau_G(q_state)
//
#pragma once

#include <string>
#include <vector>

namespace drag_teleop_controller
{

enum class Role
{
  Master,
  Slave,
};

enum class ControlMode
{
  Position,
  Mix,
  Effort,
};

enum class FeedbackMode
{
  None,
  Position,
  Effort,
};

/// 控制量计算结果（全部按本侧关节顺序，12 维）。
struct ControlResult
{
  std::vector<double> q_cmd;     // 位置命令
  std::vector<double> dq_cmd;    // 速度命令
  std::vector<double> tau_imp;   // 阻抗力矩
  std::vector<double> tau_cmd;   // 最终力矩命令
  std::vector<double> tau_model; // 模型力矩（slave 估计外部力矩用）
  std::vector<double> tau_ext;   // 外部力矩估计（slave 发布用）
  std::vector<double> delta_q;   // 位置偏移（master 力反馈）
};

/// 力反馈参数（master.feedback 段）。
struct FeedbackParams
{
  std::vector<double> gain;          // 位置反馈增益 G
  std::vector<double> dead_zone;     // 死区（rad）
  std::vector<double> max_delta_q;   // Δq 限幅（rad）
  std::vector<double> effort_gain;   // 力矩反馈增益 G
  std::vector<double> max_ext_effort;  // 外部力矩限幅（Nm）
};

/// 阻抗参数（slave.control.effort 段）。
struct ImpedanceParams
{
  std::vector<double> kp;
  std::vector<double> kd;
};

/// 纯函数控制量计算（无 ROS 依赖，可独立单元测试）。
class ControlCalculator
{
public:
  /// 从臂计算。
  /// @param q_state/dq_state/tau_state 本侧（slave）状态
  /// @param q_ref/dq_ref 对侧（master）映射后的目标（本侧关节顺序）
  /// @param tau_G_state τ_G(q_state)
  /// @param tau_G_ref   τ_G(q_ref)
  /// @param tau_model   M(q)a + C(q,v)v + G(q)（按 q_state/v_state/a_state）
  /// @param imp         阻抗参数（effort 模式）
  /// @param has_effort_state 本侧是否有 effort 状态接口
  static ControlResult calculateSlave(
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
    bool has_effort_state);

  /// 主臂计算。
  /// @param q_state/dq_state 本侧（master）状态
  /// @param q_ref/dq_ref 对侧（slave）映射后的目标（master 参考系）
  /// @param tau_ext_ref 对侧外部力矩（映射到 master 参考系）
  /// @param tau_G_state τ_G(q_state)
  /// @param fb 力反馈参数
  /// @param imp 阻抗参数（effort 模式 + position 反馈时用）
  static ControlResult calculateMaster(
    ControlMode mode,
    FeedbackMode feedback,
    const std::vector<double> & q_state,
    const std::vector<double> & dq_state,
    const std::vector<double> & q_ref,
    const std::vector<double> & dq_ref,
    const std::vector<double> & tau_ext_ref,
    const std::vector<double> & tau_G_state,
    const FeedbackParams & fb,
    const ImpedanceParams & imp);

  /// 字符串 → 枚举（非法返回 false）。
  static bool parseMode(const std::string & s, ControlMode & out);
  static bool parseFeedback(const std::string & s, FeedbackMode & out);
  static std::string modeName(ControlMode mode);
  static std::string feedbackName(FeedbackMode mode);
};

}  // namespace drag_teleop_controller