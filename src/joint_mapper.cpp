//
// JointMapper implementation.
//
#include "drag_teleop_controller/joint_mapper.hpp"

#include <cmath>
#include <stdexcept>

namespace drag_teleop_controller
{

JointMapper::JointMapper(const JointMappingConfig & config) : config_(config)
{
  if (config_.master_joints.size() != config_.slave_joints.size() ||
    config_.master_joints.size() != config_.offset.size() ||
    config_.master_joints.size() != config_.scale.size() ||
    config_.master_joints.size() != config_.sign.size())
  {
    throw std::invalid_argument(
      "JointMapper: master_joints/slave_joints/offset/scale/sign "
      "must have the same size");
  }
  if (config_.master_joints.empty())
  {
    throw std::invalid_argument("JointMapper: empty mapping config");
  }
}

bool JointMapper::map(
  const std::map<std::string, double> & master_values,
  std::vector<double> & out) const
{
  std::vector<double> result(config_.master_joints.size());
  for (size_t i = 0; i < config_.master_joints.size(); ++i)
  {
    const auto found = master_values.find(config_.master_joints[i]);
    if (found == master_values.end())
    {
      return false;
    }
    result[i] =
      static_cast<double>(config_.sign[i]) * config_.scale[i] * found->second +
      config_.offset[i];
  }
  out = std::move(result);
  return true;
}

bool JointMapper::inverse_map(
  const std::map<std::string, double> & slave_values,
  std::vector<double> & out) const
{
  std::vector<double> result(config_.master_joints.size());
  for (size_t i = 0; i < config_.master_joints.size(); ++i)
  {
    const auto found = slave_values.find(config_.slave_joints[i]);
    if (found == slave_values.end())
    {
      return false;
    }
    const double denom =
      static_cast<double>(config_.sign[i]) * config_.scale[i];
    if (!std::isfinite(denom) || denom == 0.0)
    {
      return false;
    }
    result[i] = (found->second - config_.offset[i]) / denom;
  }
  out = std::move(result);
  return true;
}

bool JointMapper::map_effort(
  const std::vector<double> & slave_effort,
  std::vector<double> & out) const
{
  // 输入可以是 12 维（仅臂关节）或 14 维（含夹爪），按索引一一映射
  if (slave_effort.size() > config_.master_joints.size())
  {
    return false;
  }
  std::vector<double> result(slave_effort.size());
  for (size_t i = 0; i < slave_effort.size(); ++i)
  {
    result[i] =
      static_cast<double>(config_.sign[i]) * config_.scale[i] * slave_effort[i];
  }
  out = std::move(result);
  return true;
}

}  // namespace drag_teleop_controller