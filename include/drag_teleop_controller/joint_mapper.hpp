//
// Joint mapping for drag teleop (master <-> slave).
//
// Pure algorithm, no ROS dependency. The mapping rules are index-aligned:
//   master_joints[i] <-> slave_joints[i]
// with
//   q_slave_ref[i] = sign[i] * scale[i] * q_master[i] + offset[i]
//   q_master_ref[i] = (q_slave[i] - offset[i]) / (sign[i] * scale[i])
//
// The mapper covers 14 joints (12 arm joints + 2 grippers). Teleop states are
// mapped BEFORE publishing: the master publishes forward-mapped states (slave
// joint names), the slave publishes inverse-mapped states (master joint
// names), so the receiving side can use them directly.
//
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace drag_teleop_controller
{

/// 关节映射配置（mapper 段，14 关节：12 臂 + 2 夹爪）。
struct JointMappingConfig
{
  std::vector<std::string> master_joints;
  std::vector<std::string> slave_joints;
  std::vector<double> offset;
  std::vector<double> scale;
  std::vector<int64_t> sign;
};

/// 纯关节映射算法（无 ROS 依赖，可独立单元测试）。
class JointMapper
{
public:
  explicit JointMapper(const JointMappingConfig & config);

  /// 正映射：master 关节值（按 master 关节名索引）→ slave 参考系。
  /// 输出按 slave_joints 顺序。任一 master 关节缺失返回 false。
  bool map(
    const std::map<std::string, double> & master_values,
    std::vector<double> & out) const;

  /// 逆映射：slave 关节值（按 slave 关节名索引）→ master 参考系。
  /// 输出按 master_joints 顺序。任一 slave 关节缺失或分母非法返回 false。
  bool inverse_map(
    const std::map<std::string, double> & slave_values,
    std::vector<double> & out) const;

  /// 力矩映射（功率守恒）：τ_master[i] = sign[i] * scale[i] * τ_slave[i]。
  /// 输入按 slave_joints 顺序，输出按 master_joints 顺序。
  bool map_effort(
    const std::vector<double> & slave_effort,
    std::vector<double> & out) const;

  const std::vector<std::string> & master_joints() const
  {
    return config_.master_joints;
  }
  const std::vector<std::string> & slave_joints() const
  {
    return config_.slave_joints;
  }
  size_t size() const { return config_.master_joints.size(); }

private:
  JointMappingConfig config_;
};

}  // namespace drag_teleop_controller