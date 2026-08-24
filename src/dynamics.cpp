//
// Dynamics implementation (Pinocchio RNEA).
//
#include "drag_teleop_controller/dynamics.hpp"

#include <stdexcept>

namespace drag_teleop_controller
{

Dynamics::Dynamics(
  const std::string & urdf_string, const std::vector<double> & gravity)
{
  if (urdf_string.empty())
  {
    throw std::invalid_argument("Dynamics: empty URDF string");
  }

  pinocchio::urdf::buildModelFromXML(urdf_string, model_);
  data_ = pinocchio::Data(model_);

  if (gravity.size() == 3)
  {
    model_.gravity = pinocchio::Motion::Zero();
    model_.gravity.linear() << gravity[0], gravity[1], gravity[2];
  }
}

Eigen::VectorXd Dynamics::calculateGravity(const Eigen::VectorXd & q) const
{
  return pinocchio::rnea(
    model_, data_, q,
    Eigen::VectorXd::Zero(model_.nv),
    Eigen::VectorXd::Zero(model_.nv));
}

Eigen::VectorXd Dynamics::calculateModelTorques(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v,
  const Eigen::VectorXd & a) const
{
  return pinocchio::rnea(model_, data_, q, v, a);
}

}  // namespace drag_teleop_controller