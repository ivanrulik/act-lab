#include "gate.hpp"
#include <chrono>
#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <thread>
#include <vector>
#include <csignal>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

extern char **environ;
using Clock = std::chrono::steady_clock;
const std::vector<std::string> joints = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
const std::array<double,6> home = {-.035074,-1.756617,-.595185,-2.360379,1.570771,4.677315};

std::string read_file(const std::string& path) {
  std::ifstream file(path); if (!file) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>(file), {}};
}
void wait_until(const std::function<bool()>& ready, const std::function<void()>& spin) {
  const auto end = Clock::now() + std::chrono::seconds(15);
  while (!ready()) {
    if (Clock::now() >= end) throw std::runtime_error("bounded DDS discovery/delivery timeout");
    spin(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
geometry_msgs::msg::PoseStamped pose_message(const Json& data) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = data.value("frame_id", "world");
  const int64_t ns = data.value("source_timestamp_ns", int64_t{0}) + 1000000000;
  msg.header.stamp.sec = ns / 1000000000; msg.header.stamp.nanosec = ns % 1000000000;
  const auto p = data.at("position"), q = data.at("quaternion_wxyz");
  auto numeric = [](const Json& v) { return v.is_null() ? std::numeric_limits<double>::quiet_NaN() : v.get<double>(); };
  msg.pose.position.x = numeric(p[0]); msg.pose.position.y = numeric(p[1]); msg.pose.position.z = numeric(p[2]);
  msg.pose.orientation.w = numeric(q[0]); msg.pose.orientation.x = numeric(q[1]); msg.pose.orientation.y = numeric(q[2]); msg.pose.orientation.z = numeric(q[3]);
  return msg;
}
int publisher() {
  auto node = std::make_shared<rclcpp::Node>("crisp_bench_publisher");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/crisp_target_pose", rclcpp::QoS(1));
  std::string line;
  while (std::getline(std::cin, line)) {
    const auto request = Json::parse(line); if (request.contains("stop")) break;
    wait_until([&] { return pub->get_subscription_count() >= 2; }, [&] {rclcpp::spin_some(node);});
    pub->publish(pose_message(request)); std::cout << "{\"published\":true}\n" << std::flush;
  }
  return 0;
}
class Producer {
 public:
  pid_t pid = -1; FILE* input = nullptr; FILE* output = nullptr;
  Producer() {
    int in[2],out[2]; if (pipe(in) || pipe(out)) throw std::runtime_error("pipe failed");
    posix_spawn_file_actions_t actions; posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions,in[0],STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions,out[1],STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions,in[1]); posix_spawn_file_actions_addclose(&actions,out[0]);
    char self[]="/proc/self/exe", arg[]="publisher"; char* argv[]={self,arg,nullptr};
    const auto error=posix_spawn(&pid,self,&actions,nullptr,argv,environ);
    posix_spawn_file_actions_destroy(&actions); close(in[0]); close(out[1]);
    if (error) throw std::runtime_error("publisher spawn failed");
    input=fdopen(in[1],"w"); output=fdopen(out[0],"r");
  }
  void publish(const Json& msg) {
    const auto line=msg.dump()+"\n"; fputs(line.c_str(),input); fflush(input);
    char ack[512]; if (!fgets(ack,sizeof(ack),output)) throw std::runtime_error("publisher failed");
    if (!Json::parse(ack).at("published").get<bool>()) throw std::runtime_error("publisher acknowledgement failed");
  }
  void kill_publisher() {
    if (pid>0) { kill(pid,SIGKILL); int status; waitpid(pid,&status,0); pid=-1; }
  }
  ~Producer() {
    if (pid>0) { fputs("{\"stop\":true}\n",input); fflush(input); int status; waitpid(pid,&status,0); }
    if(input) fclose(input); if(output) fclose(output);
  }
};
class Bench {
 public:
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader{"controller_interface","controller_interface::ControllerInterface"};
  std::shared_ptr<controller_interface::ControllerInterface> controller;
  std::array<double,6> positions=home, velocities{}, raw{};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
  pinocchio::Model model; std::unique_ptr<pinocchio::Data> data;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr observer, description_node;
  std::thread callbacks;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr probe;
  std::atomic<size_t> messages{0}; std::string urdf=read_file("/opt/crisp/ur5e.urdf");
  bool osc; int rate; std::string variant;
  Bench(bool operational, int hz, const std::string& fixture="home", const std::string& config="baseline")
    : osc(operational),rate(hz),variant(config) {
    if (fixture=="nearby") for(size_t i=0;i<6;++i) positions[i]+=(i%2 ? -.02 : .02);
    if (fixture=="singular") positions[4]=1e-6;
    if(variant=="friction" || variant=="coriolis" || variant=="moving") velocities.fill(.1);
    pinocchio::urdf::buildModelFromXML(urdf,model); data=std::make_unique<pinocchio::Data>(model);
    if (model.nv!=6 || model.nq!=6) throw std::runtime_error("UR model not six revolute joints");
    observer=std::make_shared<rclcpp::Node>("crisp_bench_observer");
    probe=observer->create_subscription<geometry_msgs::msg::PoseStamped>("/crisp_target_pose",rclcpp::QoS(1),[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr){++messages;});
    executor.add_node(observer);
    description_node=std::make_shared<rclcpp::Node>("robot_state_publisher");
    description_node->declare_parameter("robot_description",urdf);
    executor.add_node(description_node);
    callbacks=std::thread([this]{executor.spin();});
    try {configure();} catch(...) {executor.cancel();callbacks.join();throw;}

  }
  Json measured_pose() {
    Eigen::Map<Eigen::Matrix<double,6,1>> q(positions.data());
    pinocchio::forwardKinematics(model,*data,q); pinocchio::updateFramePlacements(model,*data);
    const auto transform=data->oMf[model.getFrameId("tool0")]; Eigen::Quaterniond rotation(transform.rotation());
    return {{"position", {transform.translation()[0],transform.translation()[1],transform.translation()[2]}},
            {"quaternion_wxyz", {rotation.w(),rotation.x(),rotation.y(),rotation.z()}},
            {"frame_id","world"},{"source_timestamp_ns",0}};
  }
  void configure() {
    if (controller) { executor.remove_node(controller->get_node()->get_node_base_interface()); controller->release_interfaces(); controller.reset(); }
    raw.fill(0.0); command_handles.clear(); state_handles.clear();
    controller=loader.createSharedInstance("crisp_controllers/CartesianController");
    std::vector<rclcpp::Parameter> params={
      {"joints",joints},{"end_effector_frame",std::string(variant=="invalid_frame" ? "absent" : "tool0")},
      {"base_frame",std::string("world")},{"use_sim_time",true},{"use_operational_space",osc},
      {"topics.target_pose",std::string("/crisp_target_pose")},
      {"use_gravity_compensation",variant=="gravity"},{"use_friction",variant=="friction"},
      {"use_coriolis_compensation",variant=="coriolis"},
      {"friction.fp1",std::vector<double>(6,variant=="friction" ? 1.0 : 0.0)},
      {"friction.fp2",std::vector<double>(6,variant=="friction" ? 2.0 : 0.0)},
      {"friction.fp3",std::vector<double>(6,0.0)},
      {"noise.add_random_noise",false},{"variable_stiffness.enabled",false},
      {"nullspace.stiffness",0.0},{"nullspace.projector_type",std::string("none")},
      {"joint_limit_repulsion.enabled",false},{"joint_limit_repulsion.max_torque",0.0},{"max_delta_tau",100.0/rate},
      {"filter.target_pose",(variant=="smoothing" || variant=="responsive_probe") ? 0.1 : 1.0},
      {"filter.q",1.0},{"filter.dq",1.0},{"filter.q_ref",1.0},{"filter.output_torque",1.0}};
    if(variant=="invalid_joint") {auto bad=joints;bad[5]="absent";params[0]=rclcpp::Parameter("joints",bad);}
    rclcpp::NodeOptions options; options.parameter_overrides(params);
    if(controller->init("crisp_bench",urdf,rate,"",options)!=controller_interface::return_type::OK) throw std::runtime_error("controller init failed");
    executor.add_node(controller->get_node()->get_node_base_interface());
    const auto state=controller->configure().id();
    if(variant=="invalid_frame" || variant=="invalid_joint") {
      if(state==2) throw std::runtime_error("invalid configuration accepted");
      return;
    }
    if(state!=2) throw std::runtime_error("controller configure failed");
    auto declared=controller->command_interface_configuration().names;
    auto expected=joints;for(auto& name:expected) name+="/effort";
    if(declared!=expected) throw std::runtime_error("effort interface ordering mismatch");
    expected.clear();for(auto suffix:{"/position","/velocity"}) for(auto name:joints) expected.push_back(name+suffix);
    if(controller->state_interface_configuration().names!=expected) throw std::runtime_error("state interface ordering mismatch");
    for(size_t i=0;i<6;++i) command_handles.emplace_back(joints[i],"effort",&raw[i]);
    for(size_t i=0;i<6;++i) state_handles.emplace_back(joints[i],"position",&positions[i]);
    for(size_t i=0;i<6;++i) state_handles.emplace_back(joints[i],"velocity",&velocities[i]);
    std::vector<hardware_interface::LoanedCommandInterface> cmds;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for(auto& handle:command_handles) cmds.emplace_back(handle);
    for(auto& handle:state_handles) states.emplace_back(handle);
    controller->assign_interfaces(std::move(cmds),std::move(states));
    if(controller->get_node()->activate().id()!=3) throw std::runtime_error("controller activation failed");
  }
  void receive(Producer& producer, const Json& pose) {
    const auto before=messages.load(); producer.publish(pose);
    wait_until([&]{return messages>before;},[&]{});
    // Both subscriptions are serviced by the same executor. Drain callbacks
    // before stepping; no callback executes on the control update thread.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  void update(int64_t domain_ns=0) {
    if(controller->update(rclcpp::Time(domain_ns+1000000000,RCL_ROS_TIME),rclcpp::Duration::from_seconds(1.0/rate))!=controller_interface::return_type::OK) throw std::runtime_error("update failed");
  }
  bool finite() const {for(auto value:raw) if(!std::isfinite(value)) return false;return true;}
  Json settle(int updates=200) {for(int i=0;i<updates;++i) update();return raw;}
  ~Bench() { executor.cancel();if(callbacks.joinable()) callbacks.join(); if(controller) {controller->get_node()->deactivate();controller->release_interfaces();} }
};
Json native(bool osc,const std::string& fixture,const std::string& variant) {
  Bench bench(osc,500,fixture,variant);
  if(variant=="invalid_joint" || variant=="invalid_frame") return {{"configuration_rejected",true}};
  Producer producer; const auto producer_pid=producer.pid;
  Json result={{"fixture",fixture},{"mode",osc?"operational_space":"impedance"},{"activation_effort",bench.settle()},{"measured_pose",bench.measured_pose()}};
  if(variant=="gravity") {
    Eigen::Map<Eigen::Matrix<double,6,1>> q(bench.positions.data());
    auto expected=pinocchio::computeGeneralizedGravity(bench.model,*bench.data,q);
    result["gravity_expected"]=std::vector<double>(expected.data(),expected.data()+6);result["effort"]=bench.settle(500);return result;
  }
  if(variant=="friction" || variant=="coriolis" || variant=="moving") {
    bench.velocities.fill(.1);
    Eigen::Map<Eigen::Matrix<double,6,1>> q(bench.positions.data()), dq(bench.velocities.data());
    Eigen::Matrix<double,6,1> expected=Eigen::Matrix<double,6,1>::Zero();
    if(variant=="friction") expected.setConstant(1.0/(1.0+std::exp(-.2))-.5);
    if(variant=="coriolis") {pinocchio::computeCoriolisMatrix(bench.model,*bench.data,q,dq);expected=bench.data->C*dq;}
    result["expected_compensation"]=std::vector<double>(expected.data(),expected.data()+6);
    result["effort"]=bench.settle(500);result["finite"]=bench.finite();return result;
  }
  auto target=bench.measured_pose();target["position"][0]=target["position"][0].get<double>()+.005;
  // A nontrivial quaternion checks WXYZ->XYZW mapping through actual messages.
  if(variant=="nonfinite") target["position"][0]=nullptr;
  if(variant=="non_unit") for(auto& v:target["quaternion_wxyz"]) v=v.get<double>()*2.0;
  if(variant=="rotation") {
    target["position"]=bench.measured_pose()["position"];
    auto q=target["quaternion_wxyz"]; Eigen::Quaterniond rotation(q[0].get<double>(),q[1].get<double>(),q[2].get<double>(),q[3].get<double>());
    rotation=Eigen::Quaterniond(Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ()))*rotation;
    target["quaternion_wxyz"]={rotation.w(),rotation.x(),rotation.y(),rotation.z()};
  }
  bench.receive(producer,target);result["first_target_effort"]=bench.settle(1);result["fresh_effort"]=bench.settle();result["finite"]=bench.finite();
  if(variant=="nonfinite" || variant=="non_unit") return result;
  if(variant=="smoothing" || variant=="rotation") return result;
  target["source_timestamp_ns"]=-100000000;bench.receive(producer,target);result["stale_effort"]=bench.settle();
  target["source_timestamp_ns"]=100000000;bench.receive(producer,target);result["future_effort"]=bench.settle();
  target["frame_id"]="camera";bench.receive(producer,target);result["unsupported_frame_effort"]=bench.settle();
  producer.kill_publisher();for(int i=0;i<100;++i)bench.update(100000000+i*2000000);
  result["publisher_loss_effort"]=bench.raw;
  bench.controller->get_node()->deactivate();result["deactivated_effort_buffer"]=bench.raw;
  result["producer_pid"]=producer_pid;result["consumer_pid"]=getpid();return result;
}
Json benchmark(bool osc,int rate,const std::string& fixture) {
  Bench bench(osc,rate,fixture);for(int i=0;i<1000;++i)bench.update();
  std::vector<double> samples;samples.reserve(10000);int overruns=0;
  for(int i=0;i<10000;++i){auto start=Clock::now();bench.update();double ns=std::chrono::duration<double,std::nano>(Clock::now()-start).count();samples.push_back(ns);if(ns>=1e9/rate)++overruns;}
  if(!bench.finite())throw std::runtime_error("nonfinite benchmark effort");
  std::sort(samples.begin(),samples.end());
  return {{"mode",osc?"operational_space":"impedance"},{"max_delta_tau_nm",100.0/rate},{"rate_hz",rate},{"fixture",fixture},{"median_ns",samples[5000]},{"p95_ns",samples[9500]},{"p99_ns",samples[9900]},{"max_ns",samples.back()},{"budget_overruns",overruns},{"samples",10000},{"warmup",1000}};
}
Json guarded(bool osc,const std::string& path) {
  Bench bench(osc,500);Producer producer;EffortGate gate;Json rows=Json::array();
  std::ifstream trace(path);if(!trace)throw std::runtime_error("missing guard trace");std::string line;
  while(std::getline(trace,line)) {
    const auto event=Json::parse(line);const std::string kind=event.at("kind");const int64_t steady=event.at("steady_ns");
    if(kind=="clock")gate.clock(event.at("domain_ns"),event.at("episode_id"),steady);
    else if(kind=="authorize")gate.authorize(event.at("command"),steady);
    else if(kind=="fault")gate.reject(event.at("reason"));
    else if(kind=="step") {
      const bool enabled=gate.allowed(steady);
      bool reactivated=false;
      if(enabled && gate.recovery_required){bench.configure();reactivated=true;gate.recovery_required=false;bench.receive(producer,gate.intent);}
      else if(enabled && event.value("send_target",false))bench.receive(producer,gate.intent);
      bench.update(event.at("domain_ns"));
      const auto previous=gate.previous; auto raw=bench.raw;
      if(event.value("inject_large",false))raw.fill(100.0);
      if(event.value("inject_nonfinite",false))raw[0]=std::numeric_limits<double>::quiet_NaN();
      const auto output=gate.apply(raw,steady,.002);
      const bool expected=event.at("expected_enabled");
      if(expected != (gate.reason=="authorized"))throw std::runtime_error("guard case mismatch: "+event.at("case").get<std::string>()+" "+gate.reason);
      if(!expected)for(double value:output)if(value!=0.0)throw std::runtime_error("fault did not inhibit mock effort");
      for(size_t i=0;i<6;++i)if(!std::isfinite(output[i]) || std::abs(output[i])>5.0)throw std::runtime_error("guard ceiling failed");
      if(expected) for(size_t i=0;i<6;++i) if(std::abs(output[i]-previous[i])>0.200000001) throw std::runtime_error("guard slew failed");
      rows.push_back({{"case",event.at("case")},{"raw_effort",bench.raw},{"gate_input",raw},{"guarded_effort",output},{"reason",gate.reason},{"enabled",expected},{"lifecycle_reactivated",reactivated}});
    } else throw std::runtime_error("unknown trace event");
  }
  return rows;
}
int main(int argc,char** argv) {
  try {
    rclcpp::init(argc,argv);
    if(argc>1 && std::string(argv[1])=="publisher") {int result=publisher();rclcpp::shutdown();return result;}
    if(argc<3)throw std::runtime_error("usage: bench native|benchmark|guarded|describe mode [args]");
    const std::string task=argv[1];const bool osc=std::string(argv[2])=="operational_space";Json result;
    if(task=="describe"){Bench bench(osc,500);result["pose"]=bench.measured_pose();
      const auto names=bench.controller->get_node()->list_parameters({},100).names;
      for(const auto& name:names)result["parameters"][name]=bench.controller->get_node()->get_parameter(name).value_to_string();
      result["command_interfaces"]=bench.controller->command_interface_configuration().names;
      result["state_interfaces"]=bench.controller->state_interface_configuration().names;
    }
    else if(task=="native"){if(argc!=5)throw std::runtime_error("native args");result=native(osc,argv[3],argv[4]);}
    else if(task=="benchmark"){if(argc!=5)throw std::runtime_error("benchmark args");result=benchmark(osc,std::stoi(argv[3]),argv[4]);}
    else if(task=="guarded"){if(argc!=4)throw std::runtime_error("guarded args");result=guarded(osc,argv[3]);}
    else throw std::runtime_error("unknown task");
    std::cout<<result.dump()<<std::endl;rclcpp::shutdown();return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;return 1;}
}
