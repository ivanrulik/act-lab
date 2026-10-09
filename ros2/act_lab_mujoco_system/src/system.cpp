#include "system.hpp"
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
namespace act_lab {
Snapshot &snapshot() {
  static Snapshot value;
  return value;
}
hardware_interface::CallbackReturn
MujocoSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
          hardware_interface::CallbackReturn::SUCCESS ||
      info.joints.size() != 6)
    return hardware_interface::CallbackReturn::ERROR;
  const std::array<std::string, 6> names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  for (size_t i = 0; i < 6; ++i)
    if (info.joints[i].name != names[i] ||
        info.joints[i].command_interfaces.size() != 1 ||
        info.joints[i].command_interfaces[0].name != "effort" ||
        info.joints[i].state_interfaces.size() != 2 ||
        info.joints[i].state_interfaces[0].name != "position" ||
        info.joints[i].state_interfaces[1].name != "velocity")
      return hardware_interface::CallbackReturn::ERROR;
  q_ = snapshot().q;
  dq_ = snapshot().dq;
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
MujocoSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < 6; ++i) {
    result.emplace_back(info_.joints[i].name, "position", &q_[i]);
    result.emplace_back(info_.joints[i].name, "velocity", &dq_[i]);
  }
  return result;
}
std::vector<hardware_interface::CommandInterface>
MujocoSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < 6; ++i)
    result.emplace_back(info_.joints[i].name, "effort", &effort_[i]);
  return result;
}
hardware_interface::return_type MujocoSystem::read(const rclcpp::Time &,
                                                   const rclcpp::Duration &) {
  for (size_t i = 0; i < 6; ++i)
    if (!std::isfinite(snapshot().q[i]) || !std::isfinite(snapshot().dq[i]))
      return hardware_interface::return_type::ERROR;
  q_ = snapshot().q;
  dq_ = snapshot().dq;
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoSystem::write(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  snapshot().effort = effort_;
  return hardware_interface::return_type::OK;
}
} // namespace act_lab
PLUGINLIB_EXPORT_CLASS(act_lab::MujocoSystem,
                       hardware_interface::SystemInterface)
