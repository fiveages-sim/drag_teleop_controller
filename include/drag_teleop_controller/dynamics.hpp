//
// Pinocchio-based dynamics for drag teleop (master / slave).
//
// Builds a Pinocchio model from a URDF string and provides:
//   - static gravity torques:        tau_G(q)      = rnea(q, 0, 0)
//   - full model torques:            tau_model(q)  = M(q)a + C(q,v)v + G(q)
// The slave uses tau_model to estimate external torques:
//   tau_ext = tau_state - tau_model
//
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

namespace drag_teleop_controller
{

/// 基于 Pinocchio 的动力学封装（重力矩 / 完整模型力矩）。
class Dynamics
{
public:
  /// @param urdf_string 完整 URDF XML（如 /robot_description 参数内容）
  /// @param gravity 世界系重力加速度 [gx, gy, gz]（默认 [0, 0, -9.81]；
  ///               双臂 body_rpy 非零时需按安装方向旋转）
  explicit Dynamics(
    const std::string & urdf_string,
    const std::vector<double> & gravity = {0.0, 0.0, -9.81});

  /// 静态重力矩（零速度零加速度 RNEA），模型 nq 维输入，nv 维输出。
  Eigen::VectorXd calculateGravity(const Eigen::VectorXd & q) const;

  /// 完整模型力矩 tau_model = M(q)a + C(q,v)v + G(q)。
  Eigen::VectorXd calculateModelTorques(
    const Eigen::VectorXd & q, const Eigen::VectorXd & v,
    const Eigen::VectorXd & a) const;

  const pinocchio::Model & getModel() const { return model_; }
  size_t getNumJoints() const { return model_.nq; }
  bool isValid() const { return model_.nq > 0; }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
};

}  // namespace drag_teleop_controller