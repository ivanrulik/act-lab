#pragma once
// Test-only mock effort authorization. This is NOT a hardware stop/hold policy.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

using Json = nlohmann::json;
constexpr int64_t watchdog_ns = 100000000;

class EffortGate {
 public:
  std::string reason = "startup";
  bool recovery_required = true;
  std::array<double, 6> previous{};
  Json intent;

  void reject(const std::string& value) {
    active_ = false; reason = value; recovery_required = true; previous.fill(0.0);
  }
  void clock(int64_t domain, const std::string& episode, int64_t steady) {
    check_steady(steady);
    if (episode != episode_) {
      episode_ = episode; has_sequence_ = false; initialized_ = false; reset_required_ = false;
      reject("episode_reset");
    }
    if (reset_required_) { reject("new_episode_required"); return; }
    if (initialized_ && domain < domain_) {
      reset_required_ = true; reject("clock_reversed"); return;
    }
    if (initialized_ && steady - progress_ >= watchdog_ns) reject("clock_paused");
    if (!initialized_ || domain > domain_) progress_ = steady;
    domain_ = domain; initialized_ = true;
  }
  void authorize(const Json& command, int64_t steady) {
    check_steady(steady);
    try {
    if (!initialized_ || reset_required_) { reject("clock_not_ready"); return; }
    if (command.at("episode_id") != episode_) { reject("wrong_episode"); return; }
    const auto& encoded_sequence=command.at("sequence");
    if(!encoded_sequence.is_number_integer() ||
       (!encoded_sequence.is_number_unsigned() && encoded_sequence.get<int64_t>()<0)) {
      reject("invalid_sequence");return;
    }
    const auto seq = encoded_sequence.get<uint64_t>();
    if (has_sequence_ && seq <= sequence_) { reject("out_of_order"); return; }
    sequence_ = seq; has_sequence_ = true;
    if (!command.at("enabled").get<bool>()) { reject("disabled"); return; }
    if (command.at("frame_id") != "world") { reject("invalid_frame"); return; }
    source_ = command.at("source_timestamp_ns").get<int64_t>();
    if (source_ < 0 || source_ > domain_) { reject("invalid_timestamp"); return; }
    if (command.at("position").size()!=3 || command.at("quaternion_wxyz").size()!=4) { reject("invalid_shape"); return; }
    for (const auto& v : command.at("position")) {
      if (!v.is_number() || !std::isfinite(v.get<double>())) { reject("invalid_numeric"); return; }
    }
    double quaternion_squared_norm = 0.0;
    for (const auto& v : command.at("quaternion_wxyz")) {
      if (!v.is_number() || !std::isfinite(v.get<double>())) { reject("invalid_numeric"); return; }
      quaternion_squared_norm += v.get<double>() * v.get<double>();
    }
    if (std::abs(std::sqrt(quaternion_squared_norm) - 1.0) > 1e-3) {
      reject("invalid_quaternion"); return;
    }
    receipt_ = command.at("receipt_steady_ns").get<int64_t>();
    if(receipt_<0 || receipt_>steady) {reject("invalid_receipt");return;}
    intent = command; active_ = true; reason = "authorized";
    } catch(const Json::exception&) {reject("malformed_envelope");}
  }
  bool allowed(int64_t steady) {
    check_steady(steady);
    if (!active_) return false;
    if (steady - progress_ >= watchdog_ns) { reject("clock_paused"); return false; }
    if (domain_ - source_ >= watchdog_ns) { reject("stale_command"); return false; }
    if (steady - receipt_ >= watchdog_ns) { reject("receipt_timeout"); return false; }
    return true;
  }
  std::array<double, 6> apply(const std::array<double, 6>& raw, int64_t steady, double dt) {
    if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("invalid gate period");
    if (!allowed(steady)) return previous;
    for (double value : raw) if (!std::isfinite(value)) { reject("nonfinite_effort"); return previous; }
    for (size_t i=0; i<6; ++i) {
      const double bounded = std::clamp(raw[i], -5.0, 5.0);
      previous[i] += std::clamp(bounded - previous[i], -100.0*dt, 100.0*dt);
    }
    return previous;
  }
 private:
  std::string episode_;
  int64_t domain_ = 0, source_ = 0, receipt_ = 0, progress_ = 0, last_steady_ = 0;
  uint64_t sequence_ = 0;
  bool has_sequence_ = false;
  bool active_ = false, initialized_ = false, reset_required_ = false;
  void check_steady(int64_t now) {
    if (now < last_steady_ || now < 0) { reject("steady_reversed"); throw std::runtime_error("steady time reversed"); }
    last_steady_ = now;
  }
};
