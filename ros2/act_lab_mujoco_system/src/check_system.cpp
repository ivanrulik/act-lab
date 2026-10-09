#include "system.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value) {
  if (!value)
    throw std::runtime_error("interface assertion failed");
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    hardware_interface::HardwareInfo info;
    info.name = "ACTLabSimulation";
    info.type = "system";
    for (const auto &name :
         {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      hardware_interface::InterfaceInfo command;
      command.name = "effort";
      joint.command_interfaces.push_back(command);
      for (const auto &state : {"position", "velocity"}) {
        hardware_interface::InterfaceInfo entry;
        entry.name = state;
        joint.state_interfaces.push_back(entry);
      }
      info.joints.push_back(joint);
    }
    auto make = [&] { return std::make_unique<act_lab::MujocoSystem>(); };
    act_lab::snapshot().q = {1, 2, 3, 4, 5, 6};
    auto system = make();
    require(system->on_init(info) ==
            hardware_interface::CallbackReturn::SUCCESS);
    auto states = system->export_state_interfaces();
    auto commands = system->export_command_interfaces();
    require(states.size() == 12 && commands.size() == 6);
    for (size_t i = 0; i < 6; ++i) {
      require(states[2 * i].get_value() == double(i + 1));
      require(commands[i].get_name() == info.joints[i].name + "/effort");
    }
    const auto time = rclcpp::Time(int64_t(1000000000), RCL_ROS_TIME);
    const auto dt = rclcpp::Duration::from_seconds(.002);
    require(system->read(time, dt) == hardware_interface::return_type::OK);
    act_lab::snapshot().q[0] = std::numeric_limits<double>::quiet_NaN();
    require(system->read(time, dt) == hardware_interface::return_type::ERROR);
    act_lab::snapshot().q[0] = 1;
    auto swapped = info;
    std::swap(swapped.joints[0], swapped.joints[1]);
    require(make()->on_init(swapped) ==
            hardware_interface::CallbackReturn::ERROR);
    auto missing = info;
    missing.joints.pop_back();
    require(make()->on_init(missing) ==
            hardware_interface::CallbackReturn::ERROR);
    auto wrong = info;
    wrong.joints[0].command_interfaces[0].name = "position";
    require(make()->on_init(wrong) ==
            hardware_interface::CallbackReturn::ERROR);
    auto feedback = info;
    feedback.joints[0].state_interfaces.pop_back();
    require(make()->on_init(feedback) ==
            hardware_interface::CallbackReturn::ERROR);
    std::cout << "canonical ordering, missing joints, wrong interface and "
                 "nonfinite feedback passed\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
