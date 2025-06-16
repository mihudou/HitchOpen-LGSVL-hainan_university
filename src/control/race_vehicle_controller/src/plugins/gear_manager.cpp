#include <memory>
#include <vector>
#include <algorithm>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "base_common/pubsub.hpp"
#include "race_vehicle_controller/plugin.hpp"

namespace race {

class Gear {
public:
  Gear(int8_t index, double min_rpm, double max_rpm)
    : index_(index), min_rpm_(min_rpm), max_rpm_(max_rpm) {}

  int8_t get_number() const { return index_; }
  double get_min_rpm() const { return min_rpm_; }
  double get_max_rpm() const { return max_rpm_; }

private:
  int8_t index_;
  double min_rpm_;
  double max_rpm_;
};

class Gearbox {
public:
  Gearbox(std::vector<Gear> gears, int8_t initial_gear) : gears_(std::move(gears)) {
    if (gears_.empty()) {
      throw std::logic_error("No gears provided!");
    }
    if (!set_gear(initial_gear)) {
      throw std::runtime_error("Initial gear invalid!");
    }
  }

  bool should_upshift(double rpm) const {
    return rpm > current_->get_max_rpm() && !is_top();
  }

  bool should_downshift(double rpm) const {
    return rpm < current_->get_min_rpm() && !is_bottom();
  }

  bool set_gear(int8_t gear_number) {
    auto it = std::find_if(gears_.begin(), gears_.end(),
      [gear_number](const Gear& g) { return g.get_number() == gear_number; });
    if (it != gears_.end()) {
      current_ = it;
      return true;
    }
    std::cerr << "Gearbox: Invalid gear set attempt: " << (int)gear_number << "\n";
    return false;
  }

  int8_t get_gear() const { return current_->get_number(); }

  int8_t next_gear() const {
    for (const auto& g : gears_) {
      if (g.get_number() > current_->get_number()) {
        return g.get_number();
      }
    }
    return current_->get_number();
  }

  int8_t prev_gear() const {
    for (auto it = gears_.rbegin(); it != gears_.rend(); ++it) {
      if (it->get_number() < current_->get_number()) {
        return it->get_number();
      }
    }
    return current_->get_number();
  }

private:
  bool is_top() const {
    return std::none_of(gears_.begin(), gears_.end(),
      [this](const Gear& g) { return g.get_number() > current_->get_number(); });
  }

  bool is_bottom() const {
    return std::none_of(gears_.begin(), gears_.end(),
      [this](const Gear& g) { return g.get_number() < current_->get_number(); });
  }

  std::vector<Gear> gears_;
  std::vector<Gear>::const_iterator current_;
};

enum class GearShiftStatus : uint8_t {
  UNINITIALIZED,
  AVAILABLE,
  LOCKEDOUT,
  UPSHIFTING,
  DOWNSHIFTING
};

struct ATManagerConfig {
  std::vector<int8_t> gear_numbers;
  std::vector<double> min_rpms;
  std::vector<double> max_rpms;
  int8_t start_gear;
  double gear_change_wait_sec;
};

class ATManager {
public:
  ATManager(rclcpp::Node& node, const ATManagerConfig& cfg)
    : node_(node), gearbox_(build_gears(cfg), cfg.start_gear),
      change_wait_sec_(cfg.gear_change_wait_sec) {
    gear_cmd_ = cfg.start_gear;
  }

  void update(double stamp, int8_t gear, uint8_t status, double rpm) {
    current_stamp_ = stamp;
    current_rpm_ = rpm;
    shift_status_ = static_cast<GearShiftStatus>(status);
    physical_gear_ = gear;
    step();
  }

  int8_t get_gear_cmd() const { return gear_cmd_; }

  bool is_shifting() const { return waiting_; }

private:
  std::vector<Gear> build_gears(const ATManagerConfig& cfg) {
    std::vector<Gear> out;
    for (size_t i = 0; i < cfg.gear_numbers.size(); ++i) {
      out.emplace_back(cfg.gear_numbers[i], cfg.min_rpms[i], cfg.max_rpms[i]);
    }
    return out;
  }

  void step() {
    constexpr double MAX_WAIT = 2.0;

    if (current_rpm_ < 500) return; // Engine not running

    if (waiting_) {
      if (current_stamp_ - last_shift_ > MAX_WAIT) {
        RCLCPP_WARN(node_.get_logger(), "Gear shift timeout; resetting");
        waiting_ = false;
      }
      return;
    }

    if (gearbox_.get_gear() != physical_gear_) {
      RCLCPP_WARN(node_.get_logger(), "Resync gear: ECU=%d", physical_gear_);
      gearbox_.set_gear(physical_gear_);
      gear_cmd_ = physical_gear_;
    }

    if (gearbox_.should_upshift(current_rpm_)) {
      gear_cmd_ = gearbox_.next_gear();
      last_shift_ = current_stamp_;
      waiting_ = true;
      RCLCPP_INFO(node_.get_logger(), "Upshift to %d at RPM %.1f", gear_cmd_, current_rpm_);
    } else if (gearbox_.should_downshift(current_rpm_)) {
      gear_cmd_ = gearbox_.prev_gear();
      last_shift_ = current_stamp_;
      waiting_ = true;
      RCLCPP_INFO(node_.get_logger(), "Downshift to %d at RPM %.1f", gear_cmd_, current_rpm_);
    }
  }

  rclcpp::Node& node_;
  Gearbox gearbox_;
  double change_wait_sec_;
  int8_t gear_cmd_;
  int8_t physical_gear_ = 0;
  double current_stamp_ = 0;
  double current_rpm_ = 0;
  GearShiftStatus shift_status_;
  bool waiting_ = false;
  double last_shift_ = 0;
};

class GearManagerPlugin : public RvcPlugin {
public:
  bool configure() override {
    auto nums = node().declare_parameter("gear_manager.gear_numbers", std::vector<int64_t>{});
    auto min = node().declare_parameter("gear_manager.min_rpms", std::vector<double>{});
    auto max = node().declare_parameter("gear_manager.max_rpms", std::vector<double>{});
    auto start = static_cast<int8_t>(node().declare_parameter("gear_manager.start_gear", 1));
    auto wait = node().declare_parameter("gear_manager.gear_change_wait_sec", 0.5);

    ATManagerConfig cfg{
      std::vector<int8_t>(nums.begin(), nums.end()),
      min, max, start, wait
    };
    manager_ = std::make_unique<ATManager>(node(), cfg);
    return true;
  }

  bool compute_control_command(VehicleControlCommand& out) override {
    if (has_report_) {
      out.gear_cmd = manager_->get_gear_cmd();
    } else {
      RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 1000, "Waiting for engine report...");
    }
    return true;
  }

  void on_engine_report_update() override {
    auto msg = const_state().engine_report;
    manager_->update(rclcpp::Time(msg->stamp).seconds(), msg->current_gear, msg->gear_shift_status, msg->engine_rpm);
    has_report_ = true;
  }

private:
  std::unique_ptr<ATManager> manager_;
  bool has_report_ = false;
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::GearManagerPlugin, race::RvcPlugin)

class ATManager
{
public:
  typedef std::unique_ptr<ATManager> UniquePtr;
  ATManager(rclcpp::Node & parent, const ATManagerConfig & config)
  : parent_(parent)
  {
    if (!(config.gear_numbers.size() == config.min_rpms.size() &&
      config.gear_numbers.size() == config.max_rpms.size()))
    {
      throw std::logic_error("Quantity mismatch: gear numbers, min rpms, and max rpms.");
    }
    auto gears = std::vector<Gear>();
    for (size_t i = 0; i < config.gear_numbers.size(); i++) {
      gears.push_back(Gear(config.gear_numbers[i], config.min_rpms[i], config.max_rpms[i]));
    }
    m_gearbox_ = Gearbox(gears, config.start_gear);
    m_gear_change_wait_sec_ = config.gear_change_wait_sec;
    gear_command_ = config.start_gear;
  }

  /**
   * @brief step the model when new gear information is available
   *
   * @param timestamp_sec timestamp in second
   * @param gear_number actual physical gear number
   * @param gear_shift_status gear shift status code from Raptor
   * @param rpm engine rpm
   */
  void update(
    const double & timestamp_sec, const int8_t & gear_number,
    const uint8_t & gear_shift_status, const double & rpm)
  {
    try {
      const auto status = static_cast<GearShiftStatus>(gear_shift_status);
      if (!(status >= GearShiftStatus::UNINITIALIZED && status <= GearShiftStatus::DOWNSHIFTING)) {
        throw 42;
      }
      m_gear_shift_status_ = status;
    } catch (...) {
      RCLCPP_ERROR(parent_.get_logger(), "Gearbox: invalid gearshift status.");
      return;
    }
    if (m_gearbox_.has_gear(gear_number)) {
      m_physical_gear_number_ = gear_number;
    }
    current_time_sec = timestamp_sec;
    current_rpm_ = rpm;
    step();
  }

  /**
   * @brief Get the desired gear command
   *
   * @return int8_t gear command
   */
  uint8_t get_gear_command() const
  {
    return gear_command_;
  }

  bool is_waiting_for_gear_shift() const
  {
    return waiting_for_shift_;
  }

  double get_last_gear_shift_time() const
  {
    return last_gearshift_time_sec;
  }

private:
  Gearbox m_gearbox_;
  int8_t m_physical_gear_number_ = 0;
  uint8_t m_physical_gear_shift_status = 0;
  double current_time_sec = -1.0;
  double current_rpm_ = 0.0;
  double last_gearshift_time_sec = -1.0;
  bool waiting_for_shift_ = false;
  GearShiftStatus m_gear_shift_status_;
  int8_t gear_command_ = 0;
  rclcpp::Node & parent_;

  double m_gear_change_wait_sec_ = 0.0;

  void step()
  {
    static constexpr auto ENGINE_STALL_RPM = 500U;
    const auto engine_running = current_rpm_ > ENGINE_STALL_RPM;
    if (!engine_running) {
      return;            // Don't do anything when the engine is not running
    }
    if (current_time_sec < 0.0) {
      return;            // Don't do anything when the first update has not come through
    }
    const auto may_shift = (m_gear_shift_status_ == GearShiftStatus::AVAILABLE ||
      m_gear_shift_status_ == GearShiftStatus::UNINITIALIZED ||
      m_gear_shift_status_ == GearShiftStatus::LOCKEDOUT);
    if (waiting_for_shift_) {
      if ((current_time_sec - last_gearshift_time_sec) < m_gear_change_wait_sec_) {
        return;               // Don't do anything if a gear shift is in progress
      }
      if (m_gear_shift_status_ == GearShiftStatus::DOWNSHIFTING ||
        m_gear_shift_status_ == GearShiftStatus::UPSHIFTING ||
        m_gear_shift_status_ == GearShiftStatus::UNINITIALIZED)
      {
        RCLCPP_WARN(
          parent_.get_logger(),
          "Gear shifting taking exceptionally long and still in progress.");
        last_gearshift_time_sec = current_time_sec;
        return;
      } else if (m_gear_shift_status_ == GearShiftStatus::LOCKEDOUT) {
        RCLCPP_WARN(parent_.get_logger(), "Gear shifting locked out!");
        last_gearshift_time_sec = current_time_sec;
        return;
      } else if (m_gear_shift_status_ == GearShiftStatus::AVAILABLE) {
        waiting_for_shift_ = false;
        if (m_physical_gear_number_ != m_gearbox_.get_current_gear_number()) {
          RCLCPP_WARN(parent_.get_logger(), "Gear shifting successful.");
          m_gearbox_.set_gear(m_physical_gear_number_);
        } else {
          RCLCPP_WARN(parent_.get_logger(), "Gear shifting failed. Gear did not change.");
          gear_command_ = m_physical_gear_number_;
        }
      }

    } else if (may_shift) {
      // When not waiting for gear shift, check if possible to gear shift.
      if (m_gearbox_.get_current_gear_number() != m_physical_gear_number_) {
        RCLCPP_WARN(
          parent_.get_logger(), "Gear number mismatch. Actual gear is %d. Forcing reset.",
          m_physical_gear_number_);
        m_gearbox_.set_gear(m_physical_gear_number_);
      } else if (m_gearbox_.should_downshift(current_rpm_)) {
        RCLCPP_WARN(parent_.get_logger(), "Commanding downshift");
        gear_command_ -= 1;  // TODO(haoru): lookup gear box to see what's the actual next gear
        last_gearshift_time_sec = current_time_sec;
        waiting_for_shift_ = true;
      } else if (m_gearbox_.should_upshift(current_rpm_)) {
        RCLCPP_WARN(parent_.get_logger(), "Commanding upshift");
        gear_command_ += 1;  // TODO(haoru): lookup gear box to see what's the actual next gear
        last_gearshift_time_sec = current_time_sec;
        waiting_for_shift_ = true;
      }
    }
  }
};

class GearManagerPlugin : public RvcPlugin
{
public:
  bool configure() override
  {
    auto gear_numbers_param = node().declare_parameter(
      "gear_manager.gear_numbers",
      std::vector<int64_t>{});

    auto config = ATManagerConfig{
      std::vector<int8_t>(gear_numbers_param.begin(), gear_numbers_param.end()),
      node().declare_parameter("gear_manager.min_rpms", std::vector<double>{}),
      node().declare_parameter("gear_manager.max_rpms", std::vector<double>{}),
      static_cast<int8_t>(node().declare_parameter("gear_manager.start_gear", 1)),
      node().declare_parameter("gear_manager.gear_change_wait_sec", 0.5)
    };
    at_manager_ = std::make_unique<ATManager>(node(), config);
    return true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (has_engine_report_) {
      output_cmd.gear_cmd = static_cast<uint8_t>(at_manager_->get_gear_command());
    } else {
      RCLCPP_INFO_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 1000, "AT Manager Plugin: waiting for engine report.");
    }
    state().telemetry->gear_command = output_cmd.gear_cmd;
    state().telemetry->is_in_gearshift_cooldown = at_manager_->is_waiting_for_gear_shift();
    state().telemetry->last_gearshift_time_sec = at_manager_->get_last_gear_shift_time();
    return true;
  }

  void on_engine_report_update() override
  {
    const auto msg = const_state().engine_report;
    at_manager_->update(
      rclcpp::Time(
        msg->stamp).seconds(), msg->current_gear, msg->gear_shift_status, msg->engine_rpm);
    has_engine_report_ = true;
  }

private:
  ATManager::UniquePtr at_manager_{};
  bool has_engine_report_{false};
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::GearManagerPlugin, race::RvcPlugin)
