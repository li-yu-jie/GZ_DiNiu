#include <gpiod.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include <poll.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace {
constexpr int kCodeCount = 6;
const char *kForwardCodes[kCodeCount] = {"100", "110", "010", "011", "001", "101"};
}  // namespace

class SteerRateClosedLoopNode : public rclcpp::Node {
public:
  SteerRateClosedLoopNode() : Node("steer_closed_loop"), running_(true) {
    // Hardware mapping and topic names are parameterized so field tuning can be done
    // from launch/CLI without recompiling.
    chip_path_ = declare_parameter<std::string>("chip_path", "/dev/gpiochip0");
    hu_offset_ = declare_parameter<int>("hu_gpio", 5);
    hv_offset_ = declare_parameter<int>("hv_gpio", 6);
    hw_offset_ = declare_parameter<int>("hw_gpio", 13);

    dir_offset_ = declare_parameter<int>("dir_gpio", 16);
    limit_left_offset_ = declare_parameter<int>("limit_left_gpio", 24);
    limit_right_offset_ = declare_parameter<int>("limit_right_gpio", 25);
    limit_active_level_ = declare_parameter<int>("limit_active_level", 1);
    positive_dir_hits_left_limit_ = declare_parameter<bool>("positive_dir_hits_left_limit", true);
    limit_use_bias_ = declare_parameter<bool>("limit_use_bias", true);
    limit_bias_ = declare_parameter<std::string>("limit_bias", "pull_down");
    limit_startup_grace_s_ = declare_parameter<double>("limit_startup_grace_s", 0.5);

    pwm_chip_path_ = declare_parameter<std::string>("pwm_chip_path", "/sys/class/pwm/pwmchip0/");
    pwm_channel_ = declare_parameter<int>("pwm_channel", 1);
    pwm_period_ns_ = declare_parameter<long>("pwm_period_ns", 100000);

    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "steer_target_rate_deg_s");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "steer_rate_deg_s");

    encoder_counts_per_rev_ = declare_parameter<int>("encoder_counts_per_rev", 5770);
    invert_encoder_ = declare_parameter<bool>("invert_encoder", true);
    event_debounce_us_ = declare_parameter<int>("event_debounce_us", 100);

    control_hz_ = declare_parameter<double>("control_hz", 50.0);
    kp_ = declare_parameter<double>("kp", 3.0);
    ki_ = declare_parameter<double>("ki", 0.5);
    kd_ = declare_parameter<double>("kd", 0.0);
    i_max_ = declare_parameter<double>("i_max", 400.0);
    rate_deadband_deg_s_ = declare_parameter<double>("rate_deadband_deg_s", 0.2);
    max_pwm_percent_ = declare_parameter<double>("max_pwm_percent", 100.0);
    max_pwm_step_ = declare_parameter<double>("max_pwm_step", 2.0);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.3);
    debug_hz_ = declare_parameter<double>("debug_hz", 2.0);

    validate_parameters();
    deg_per_step_ = 360.0 / static_cast<double>(encoder_counts_per_rev_);

    open_pwm();
    open_gpio();

    cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      cmd_topic_, 10, std::bind(&SteerRateClosedLoopNode::on_cmd, this, std::placeholders::_1));
    feedback_pub_ = create_publisher<std_msgs::msg::Float64>(feedback_topic_, 10);

    encoder_thread_ = std::thread([this]() { this->encoder_loop(); });

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SteerRateClosedLoopNode::control_step, this));

    last_control_time_ = now();
    last_rate_time_ = last_control_time_;
    last_rate_step_ = read_signed_steps();
    last_cmd_time_ = std::chrono::steady_clock::now();
    startup_time_ = std::chrono::steady_clock::now();
    next_debug_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "steer_closed_loop(rate) started. cmd_topic=%s feedback_topic=%s",
      cmd_topic_.c_str(), feedback_topic_.c_str());
  }

  ~SteerRateClosedLoopNode() override {
    running_.store(false, std::memory_order_relaxed);
    if (encoder_thread_.joinable()) {
      encoder_thread_.join();
    }
    stop_motor();
    close_gpio();
    close_pwm();
  }

private:
  void validate_parameters() {
    if (encoder_counts_per_rev_ <= 0) {
      throw std::runtime_error("encoder_counts_per_rev must be > 0");
    }
    if (control_hz_ <= 0.0) {
      throw std::runtime_error("control_hz must be > 0");
    }
    if (max_pwm_percent_ <= 0.0 || max_pwm_percent_ > 100.0) {
      throw std::runtime_error("max_pwm_percent must be in (0, 100]");
    }
    if (max_pwm_step_ <= 0.0) {
      throw std::runtime_error("max_pwm_step must be > 0");
    }
    if (cmd_timeout_s_ < 0.0) {
      throw std::runtime_error("cmd_timeout_s must be >= 0");
    }
    if (pwm_period_ns_ <= 0) {
      throw std::runtime_error("pwm_period_ns must be > 0");
    }
  }

  static double clamp(double v, double lo, double hi) {
    if (v < lo) {
      return lo;
    }
    if (v > hi) {
      return hi;
    }
    return v;
  }

  static int code_index(const char *code) {
    for (int i = 0; i < kCodeCount; ++i) {
      if (std::strcmp(code, kForwardCodes[i]) == 0) {
        return i;
      }
    }
    return -1;
  }

  void on_cmd(const std_msgs::msg::Float64::SharedPtr msg) {
    target_rate_deg_s_.store(msg->data, std::memory_order_relaxed);
    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  void control_step() {
    // Main velocity loop: estimate rate -> PID -> direction + PWM.
    const auto tnow = now();
    const double dt = (tnow - last_control_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_control_time_ = tnow;

    const double measured_rate = compute_rate_deg_s(tnow);
    publish_rate(measured_rate);

    double target_rate = target_rate_deg_s_.load(std::memory_order_relaxed);
    // Failsafe: if commands stop arriving, automatically decay target to zero.
    if (cmd_timeout_s_ > 0.0) {
      const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_time_).count();
      if (age > cmd_timeout_s_) {
        target_rate = 0.0;
      }
    }

    double error = target_rate - measured_rate;
    if (std::abs(error) < rate_deadband_deg_s_) {
      error = 0.0;
    }

    integral_ = clamp(integral_ + error * dt, -i_max_, i_max_);
    const double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    const double raw = kp_ * error + ki_ * integral_ + kd_ * derivative;
    double out = clamp(raw, -max_pwm_percent_, max_pwm_percent_);

    const double delta = out - prev_output_;
    if (delta > max_pwm_step_) {
      out = prev_output_ + max_pwm_step_;
    } else if (delta < -max_pwm_step_) {
      out = prev_output_ - max_pwm_step_;
    }
    prev_output_ = out;

    // Limit safety: prevent driving deeper into an active end-stop.
    if (out > 0.0 && positive_limit_active()) {
      out = 0.0;
      integral_ = 0.0;
    }
    if (out < 0.0 && negative_limit_active()) {
      out = 0.0;
      integral_ = 0.0;
    }

    set_direction(out >= 0.0);
    set_pwm(std::abs(out));

    if (std::chrono::steady_clock::now() >= next_debug_time_) {
      RCLCPP_INFO(
        get_logger(),
        "rate=%.2f target=%.2f err=%.2f out%%=%.2f L=%d R=%d",
        measured_rate, target_rate, error, out,
        static_cast<int>(limit_left_active()), static_cast<int>(limit_right_active()));
      const auto period = std::chrono::duration<double>(1.0 / debug_hz_);
      next_debug_time_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    }
  }

  double compute_rate_deg_s(const rclcpp::Time &tnow) {
    // Rate is derived from hall-step delta over dt.
    const long current_step = read_signed_steps();
    const double dt = (tnow - last_rate_time_).seconds();
    if (dt <= 0.0) {
      return 0.0;
    }
    const long delta = current_step - last_rate_step_;
    last_rate_step_ = current_step;
    last_rate_time_ = tnow;
    return (static_cast<double>(delta) * deg_per_step_) / dt;
  }

  void publish_rate(double rate_deg_s) {
    std_msgs::msg::Float64 msg;
    msg.data = rate_deg_s;
    feedback_pub_->publish(msg);
  }

  long read_signed_steps() const {
    const long steps = total_step_.load(std::memory_order_relaxed);
    return invert_encoder_ ? -steps : steps;
  }

  bool limit_left_active() const {
    if (in_limit_startup_grace()) {
      return false;
    }
    if (!limit_left_line_) {
      return false;
    }
    const int v = gpiod_line_get_value(limit_left_line_);
    return v == limit_active_level_;
  }

  bool limit_right_active() const {
    if (in_limit_startup_grace()) {
      return false;
    }
    if (!limit_right_line_) {
      return false;
    }
    const int v = gpiod_line_get_value(limit_right_line_);
    return v == limit_active_level_;
  }

  bool positive_limit_active() const {
    return positive_dir_hits_left_limit_ ? limit_left_active() : limit_right_active();
  }

  bool negative_limit_active() const {
    return positive_dir_hits_left_limit_ ? limit_right_active() : limit_left_active();
  }

  bool in_limit_startup_grace() const {
    // Ignore limit switches briefly after startup to avoid power-on glitches.
    if (limit_startup_grace_s_ <= 0.0) {
      return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - startup_time_).count();
    return age < limit_startup_grace_s_;
  }

  void encoder_loop() {
    // Dedicated thread for hall event polling to keep control loop timing stable.
    pollfd fds[3];
    fds[0].fd = gpiod_line_event_get_fd(hu_line_);
    fds[1].fd = gpiod_line_event_get_fd(hv_line_);
    fds[2].fd = gpiod_line_event_get_fd(hw_line_);
    fds[0].events = POLLIN;
    fds[1].events = POLLIN;
    fds[2].events = POLLIN;

    if (fds[0].fd < 0 || fds[1].fd < 0 || fds[2].fd < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "event fd invalid: hu=%d hv=%d hw=%d",
        fds[0].fd, fds[1].fd, fds[2].fd);
      return;
    }

    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      const int ret = poll(fds, 3, 20);
      if (ret == 0) {
        continue;
      }
      if (ret < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "poll failed: errno=%d", errno);
        continue;
      }
      if (fds[0].revents & POLLIN) {
        process_event(hu_line_, last_evt_us_hu_, hu_level_);
      }
      if (fds[1].revents & POLLIN) {
        process_event(hv_line_, last_evt_us_hv_, hv_level_);
      }
      if (fds[2].revents & POLLIN) {
        process_event(hw_line_, last_evt_us_hw_, hw_level_);
      }
    }
  }

  void process_event(gpiod_line *line, unsigned long long &last_evt_us, int &level) {
    gpiod_line_event ev{};
    if (gpiod_line_event_read(line, &ev) < 0) {
      return;
    }

    const auto now_us = static_cast<unsigned long long>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    // Simple software debounce on each hall channel.
    if (last_evt_us != 0 && now_us - last_evt_us < static_cast<unsigned long long>(event_debounce_us_)) {
      return;
    }
    last_evt_us = now_us;

    if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
      level = 1;
    } else if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
      level = 0;
    }

    char code[4];
    std::snprintf(code, sizeof(code), "%d%d%d", hu_level_, hv_level_, hw_level_);
    const int curr = code_index(code);
    if (curr < 0) {
      return;
    }
    if (last_code_idx_ < 0) {
      last_code_idx_ = curr;
      return;
    }
    if (curr == last_code_idx_) {
      return;
    }

    // Valid 6-step commutation sequence only; jump codes are ignored.
    if ((last_code_idx_ + 1) % kCodeCount == curr) {
      total_step_.fetch_add(1, std::memory_order_relaxed);
    } else if ((last_code_idx_ - 1 + kCodeCount) % kCodeCount == curr) {
      total_step_.fetch_sub(1, std::memory_order_relaxed);
    }
    last_code_idx_ = curr;
  }

  bool write_file(const std::string &path, const std::string &value) {
    std::ofstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "open file failed: %s", path.c_str());
      return false;
    }
    f << value;
    return true;
  }

  void open_pwm() {
    // sysfs PWM setup: export -> period -> duty=0 -> enable.
    (void)write_file(pwm_chip_path_ + "export", std::to_string(pwm_channel_));
    if (!write_file(
        pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/period",
        std::to_string(pwm_period_ns_))) {
      throw std::runtime_error("set pwm period failed");
    }
    if (!write_file(
        pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/duty_cycle", "0")) {
      throw std::runtime_error("set pwm duty failed");
    }
    if (!write_file(
        pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/enable", "1")) {
      throw std::runtime_error("enable pwm failed");
    }
  }

  void close_pwm() {
    (void)write_file(pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/enable", "0");
    (void)write_file(pwm_chip_path_ + "unexport", std::to_string(pwm_channel_));
  }

  void set_pwm(double percent) {
    percent = clamp(percent, 0.0, 100.0);
    const long duty = static_cast<long>(std::llround(
      static_cast<double>(pwm_period_ns_) * (percent / 100.0)));
    (void)write_file(
      pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/duty_cycle",
      std::to_string(duty));
  }

  void stop_motor() {
    set_pwm(0.0);
  }

  void open_gpio() {
    chip_ = gpiod_chip_open(chip_path_.c_str());
    if (!chip_) {
      throw std::runtime_error("open gpio chip failed: " + chip_path_);
    }

    dir_line_ = gpiod_chip_get_line(chip_, dir_offset_);
    hu_line_ = gpiod_chip_get_line(chip_, hu_offset_);
    hv_line_ = gpiod_chip_get_line(chip_, hv_offset_);
    hw_line_ = gpiod_chip_get_line(chip_, hw_offset_);
    limit_left_line_ = gpiod_chip_get_line(chip_, limit_left_offset_);
    limit_right_line_ = gpiod_chip_get_line(chip_, limit_right_offset_);
    if (!dir_line_ || !hu_line_ || !hv_line_ || !hw_line_ || !limit_left_line_ || !limit_right_line_) {
      throw std::runtime_error("get gpio line failed");
    }

    if (gpiod_line_request_output(dir_line_, "steer-dir", 0) != 0) {
      throw std::runtime_error("request dir output failed");
    }
    if (gpiod_line_request_both_edges_events(hu_line_, "steer-hu") != 0 ||
        gpiod_line_request_both_edges_events(hv_line_, "steer-hv") != 0 ||
        gpiod_line_request_both_edges_events(hw_line_, "steer-hw") != 0) {
      throw std::runtime_error("request hall events failed");
    }
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    // Prefer explicit pull bias on limit inputs to avoid floating states.
    if (limit_use_bias_) {
      gpiod_line_request_config cfg_l = {};
      gpiod_line_request_config cfg_r = {};
      cfg_l.consumer = "steer-limit-l";
      cfg_r.consumer = "steer-limit-r";
      cfg_l.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
      cfg_r.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
      if (limit_bias_ == "pull_up") {
        cfg_l.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
        cfg_r.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
      } else if (limit_bias_ == "pull_down") {
        cfg_l.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
        cfg_r.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
      } else {
        cfg_l.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
        cfg_r.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
      }
      if (gpiod_line_request(limit_left_line_, &cfg_l, 0) != 0 ||
          gpiod_line_request(limit_right_line_, &cfg_r, 0) != 0) {
        throw std::runtime_error("request limit input with bias failed");
      }
    } else
#endif
    {
      if (gpiod_line_request_input(limit_left_line_, "steer-limit-l") != 0 ||
          gpiod_line_request_input(limit_right_line_, "steer-limit-r") != 0) {
        throw std::runtime_error("request limit input failed");
      }
    }

    int v = gpiod_line_get_value(hu_line_);
    hu_level_ = (v < 0) ? 0 : v;
    v = gpiod_line_get_value(hv_line_);
    hv_level_ = (v < 0) ? 0 : v;
    v = gpiod_line_get_value(hw_line_);
    hw_level_ = (v < 0) ? 0 : v;

    char code[4];
    std::snprintf(code, sizeof(code), "%d%d%d", hu_level_, hv_level_, hw_level_);
    last_code_idx_ = code_index(code);
  }

  void close_gpio() {
    if (dir_line_) {
      gpiod_line_release(dir_line_);
      dir_line_ = nullptr;
    }
    if (hu_line_) {
      gpiod_line_release(hu_line_);
      hu_line_ = nullptr;
    }
    if (hv_line_) {
      gpiod_line_release(hv_line_);
      hv_line_ = nullptr;
    }
    if (hw_line_) {
      gpiod_line_release(hw_line_);
      hw_line_ = nullptr;
    }
    if (limit_left_line_) {
      gpiod_line_release(limit_left_line_);
      limit_left_line_ = nullptr;
    }
    if (limit_right_line_) {
      gpiod_line_release(limit_right_line_);
      limit_right_line_ = nullptr;
    }
    if (chip_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  void set_direction(bool forward) {
    if (!dir_line_) {
      return;
    }
    (void)gpiod_line_set_value(dir_line_, forward ? 1 : 0);
  }

private:
  std::string chip_path_;
  std::string pwm_chip_path_;
  std::string cmd_topic_;
  std::string feedback_topic_;

  int hu_offset_{};
  int hv_offset_{};
  int hw_offset_{};
  int dir_offset_{};
  int limit_left_offset_{};
  int limit_right_offset_{};
  int limit_active_level_{};
  bool positive_dir_hits_left_limit_{true};
  bool limit_use_bias_{true};
  std::string limit_bias_{"pull_down"};
  double limit_startup_grace_s_{0.5};

  int pwm_channel_{};
  long pwm_period_ns_{};

  int encoder_counts_per_rev_{};
  bool invert_encoder_{};
  int event_debounce_us_{};

  double control_hz_{};
  double kp_{};
  double ki_{};
  double kd_{};
  double i_max_{};
  double rate_deadband_deg_s_{};
  double max_pwm_percent_{};
  double max_pwm_step_{};
  double cmd_timeout_s_{};
  double debug_hz_{};

  double deg_per_step_{};
  double integral_{0.0};
  double prev_error_{0.0};
  double prev_output_{0.0};

  rclcpp::Time last_control_time_;
  rclcpp::Time last_rate_time_;
  long last_rate_step_{0};
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::chrono::steady_clock::time_point startup_time_;
  std::chrono::steady_clock::time_point next_debug_time_;

  std::atomic<bool> running_;
  std::atomic<double> target_rate_deg_s_{0.0};
  std::atomic<long> total_step_{0};

  int hu_level_{0};
  int hv_level_{0};
  int hw_level_{0};
  int last_code_idx_{-1};
  unsigned long long last_evt_us_hu_{0};
  unsigned long long last_evt_us_hv_{0};
  unsigned long long last_evt_us_hw_{0};

  gpiod_chip *chip_{nullptr};
  gpiod_line *dir_line_{nullptr};
  gpiod_line *hu_line_{nullptr};
  gpiod_line *hv_line_{nullptr};
  gpiod_line *hw_line_{nullptr};
  gpiod_line *limit_left_line_{nullptr};
  gpiod_line *limit_right_line_{nullptr};

  std::thread encoder_thread_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SteerRateClosedLoopNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "steer_closed_loop startup error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
