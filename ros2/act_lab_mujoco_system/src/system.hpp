#pragma once
#include <array>
#include <hardware_interface/system_interface.hpp>
namespace act_lab {
struct Snapshot {
  std::array<double, 6> q{}, dq{}, effort{};
};
Snapshot &snapshot();
class MujocoSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  std::array<double, 6> q_{}, dq_{}, effort_{};
};
} // namespace act_lab
