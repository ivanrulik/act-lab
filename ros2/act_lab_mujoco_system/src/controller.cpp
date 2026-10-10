#include "system.hpp"
#include <atomic>
#include <chrono>
#include <controller_manager/controller_manager.hpp>
#include <fstream>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
using Json = nlohmann::json;
using namespace std::chrono_literals;
struct ManagerCallbackPause {
  std::shared_ptr<rclcpp::Executor> executor;
  std::shared_ptr<rclcpp::Node> manager;
  std::mutex &execution;
  ManagerCallbackPause(std::shared_ptr<rclcpp::Executor> exec,
                       std::shared_ptr<rclcpp::Node> node, std::mutex &mutex)
      : executor(std::move(exec)), manager(std::move(node)), execution(mutex) {
    std::lock_guard<std::mutex> lock(execution);
    executor->remove_node(manager);
  }
  ~ManagerCallbackPause() {
    std::lock_guard<std::mutex> lock(execution);
    executor->add_node(manager);
  }
};
std::string read_file(const std::string &path) {
  std::ifstream f(path);
  if (!f)
    throw std::runtime_error("missing " + path);
  return {std::istreambuf_iterator<char>(f), {}};
}
int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    std::string line;
    if (!std::getline(std::cin, line))
      throw std::runtime_error("missing initial snapshot");
    auto initial = Json::parse(line);
    if (initial.at("schema_version") != 1)
      throw std::runtime_error("invalid initial protocol version");
    act_lab::snapshot().q = initial.at("q").get<std::array<double, 6>>();
    auto urdf = read_file("/opt/crisp/ur5e.urdf");
    auto names = initial.at("joints");
    std::string hardware = "<ros2_control name=\"ACTLabSimulation\" "
                           "type=\"system\"><hardware><plugin>act_lab/"
                           "MujocoSystem</plugin></hardware>";
    for (const auto &name : names)
      hardware +=
          "<joint name=\"" + name.get<std::string>() +
          "\"><command_interface name=\"effort\"/><state_interface "
          "name=\"position\"/><state_interface name=\"velocity\"/></joint>";
    hardware += "</ros2_control>";
    urdf.insert(urdf.rfind("</robot>"), hardware);
    auto executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
    source->declare_parameter("robot_description", urdf);
    executor->add_node(source);
    const auto conflict_path =
        "/tmp/act_lab_conflict_" + std::to_string(getpid()) + ".yaml";
    auto conflict_config =
        read_file("/workspace/configs/ros2/crisp-simulation.yaml");
    conflict_config.replace(0, 6, "conflict:");
    std::ofstream(conflict_path) << conflict_config;
    auto options = controller_manager::get_cm_node_options();
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("update_rate", 500),
         rclcpp::Parameter(
             "crisp.params_file",
             std::string("/workspace/configs/ros2/crisp-simulation.yaml")),
         rclcpp::Parameter("conflict.params_file", conflict_path)});
    auto manager = std::make_shared<controller_manager::ControllerManager>(
        executor, urdf, true, "controller_manager", "", options);
    executor->add_node(manager);
    auto transport =
        std::make_shared<rclcpp::Node>("act_lab_controller_transport");
    executor->add_node(transport);
    auto pub = transport->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/act_lab/internal/crisp_target", rclcpp::QoS(1));
    std::atomic<size_t> delivered{0}, cycles{0};
    std::atomic<bool> done{false};
    auto sub = transport->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/act_lab/internal/crisp_target", rclcpp::QoS(1),
        [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr) { ++delivered; });
    std::mutex execution;
    std::thread callbacks([&] {
      while (!done) {
        {
          std::lock_guard<std::mutex> lock(execution);
          executor->spin_some(100us);
          ++cycles;
        }
        std::this_thread::sleep_for(100us);
      }
    });
    auto cleanup = [&] {
      done = true;
      callbacks.join();
    };
    try {
      auto controller = manager->load_controller(
          "crisp", "crisp_controllers/CartesianController");
      if (!controller)
        throw std::runtime_error("plugin loading failed");
      auto update = [&](int64_t ns) {
        std::lock_guard<std::mutex> lock(execution);
        auto time = rclcpp::Time(ns + 1000000000, RCL_ROS_TIME);
        auto dt = rclcpp::Duration::from_seconds(.002);
        manager->read(time, dt);
        if (manager->update(time, dt) != controller_interface::return_type::OK)
          throw std::runtime_error("controller update failed");
        manager->write(time, dt);
      };
      auto switch_mode = [&](bool activate, int64_t ns) {
        ManagerCallbackPause pause(executor, manager, execution);
        auto result = std::async(std::launch::async, [&] {
          return manager->switch_controller(
              activate ? std::vector<std::string>{"crisp"}
                       : std::vector<std::string>{},
              activate ? std::vector<std::string>{}
                       : std::vector<std::string>{"crisp"},
              2, true, rclcpp::Duration::from_seconds(5));
        });
        auto end = std::chrono::steady_clock::now() + 6s;
        while (result.wait_for(0ms) != std::future_status::ready) {
          if (std::chrono::steady_clock::now() > end)
            std::_Exit(2);
          update(ns);
          std::this_thread::sleep_for(1ms);
        }
        if (result.get() != controller_interface::return_type::OK)
          throw std::runtime_error("exclusive interface switch failed");
      };
      auto lifecycle =
          [&](const std::function<controller_interface::return_type()>
                  &operation,
              int64_t ns) {
            // Manager lifecycle/list handoffs need manual updates to advance.
            // Suspend only manager callbacks that may wait on its list locks.
            // CRISP and the model parameter service must keep spinning.
            ManagerCallbackPause pause(executor, manager, execution);
            auto result = std::async(std::launch::async, operation);
            auto end = std::chrono::steady_clock::now() + 15s;
            while (result.wait_for(0ms) != std::future_status::ready) {
              if (std::chrono::steady_clock::now() > end)
                std::_Exit(2);
              update(ns);
              std::this_thread::sleep_for(1ms);
            }
            return result.get();
          };
      bool active = false;
      bool ownership_checked = false;
      std::cout << Json({{"schema_version", 1},
                         {"ready", true},
                         {"pid", getpid()}})
                       .dump()
                << std::endl;
      while (std::getline(std::cin, line)) {
        const auto request = Json::parse(line);
        if (request.at("schema_version") != 1)
          throw std::runtime_error("invalid controller protocol version");
        if (request.value("shutdown", false))
          break;
        act_lab::snapshot().q = request.at("q").get<std::array<double, 6>>();
        act_lab::snapshot().dq = request.at("dq").get<std::array<double, 6>>();
        const int64_t ns = request.at("domain_ns");
        if (request.value("recover", false)) {
          if (active) {
            switch_mode(false, ns);
            active = false;
            if (lifecycle([&] { return manager->cleanup_controller("crisp"); },
                          ns) != controller_interface::return_type::OK)
              throw std::runtime_error("cleanup failed");
          }
          if (lifecycle([&] { return manager->configure_controller("crisp"); },
                        ns) != controller_interface::return_type::OK)
            throw std::runtime_error("configuration failed");
          std::cerr << "RECOVERY configured" << std::endl;
          update(ns);
          std::cerr << "RECOVERY updated" << std::endl;
          switch_mode(true, ns);
          std::cerr << "RECOVERY active" << std::endl;
          active = true;
          if (!ownership_checked) {
            decltype(controller) other;
            if (lifecycle(
                    [&] {
                      other = manager->load_controller(
                          "conflict", "crisp_controllers/CartesianController");
                      return other ? controller_interface::return_type::OK
                                   : controller_interface::return_type::ERROR;
                    },
                    ns) != controller_interface::return_type::OK ||
                lifecycle(
                    [&] { return manager->configure_controller("conflict"); },
                    ns) != controller_interface::return_type::OK)
              throw std::runtime_error("conflict test configuration failed");
            const auto result = lifecycle(
                [&] {
                  return manager->switch_controller(
                      {"conflict"}, {}, 2, true,
                      rclcpp::Duration::from_seconds(1));
                },
                ns);
            if (result != controller_interface::return_type::ERROR)
              throw std::runtime_error(
                  "second controller acquired owned effort interfaces");
            if (lifecycle(
                    [&] { return manager->cleanup_controller("conflict"); },
                    ns) != controller_interface::return_type::OK ||
                lifecycle(
                    [&] { return manager->unload_controller("conflict"); },
                    ns) != controller_interface::return_type::OK)
              throw std::runtime_error("conflict test cleanup failed");
            ownership_checked = true;
            std::cerr << "Exclusive effort ownership verified against "
                         "competing CRISP instance"
                      << std::endl;
          }
        }
        if (request.contains("target")) {
          const auto target = request.at("target");
          geometry_msgs::msg::PoseStamped msg;
          msg.header.frame_id = "world";
          msg.header.stamp =
              rclcpp::Time(target.at("source_ns").get<int64_t>() + 1000000000);
          auto p = target.at("position"), q = target.at("quaternion_wxyz");
          msg.pose.position.x = p[0];
          msg.pose.position.y = p[1];
          msg.pose.position.z = p[2];
          msg.pose.orientation.w = q[0];
          msg.pose.orientation.x = q[1];
          msg.pose.orientation.y = q[2];
          msg.pose.orientation.z = q[3];
          auto end = std::chrono::steady_clock::now() + 15s;
          while (pub->get_subscription_count() < 2) {
            if (std::chrono::steady_clock::now() > end)
              throw std::runtime_error("DDS discovery timeout");
            std::this_thread::sleep_for(1ms);
          }
          auto count = delivered.load();
          pub->publish(msg);
          if (!pub->wait_for_all_acked(1s))
            throw std::runtime_error(
                "bounded CRISP DDS acknowledgment timeout");
          while (delivered == count) {
            if (std::chrono::steady_clock::now() > end)
              throw std::runtime_error("DDS delivery timeout");
            std::this_thread::sleep_for(100us);
          }
          auto previous = cycles.load();
          while (cycles <= previous + 1) {
            if (std::chrono::steady_clock::now() > end)
              throw std::runtime_error("bounded callback completion timeout");
            std::this_thread::sleep_for(100us);
          }
        }
        update(ns);
        std::cout << Json({{"schema_version", 1},
                           {"effort", act_lab::snapshot().effort},
                           {"generation", request.at("generation")},
                           {"tick", request.at("tick")},
                           {"episode", request.at("episode")},
                           {"reactivated", request.value("recover", false)}})
                         .dump()
                  << std::endl;
      }
      if (active)
        switch_mode(false, 0);
      cleanup();
      controller.reset();
      manager.reset();
      std::remove(conflict_path.c_str());
      rclcpp::shutdown();
      return 0;
    } catch (...) {
      cleanup();
      throw;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
