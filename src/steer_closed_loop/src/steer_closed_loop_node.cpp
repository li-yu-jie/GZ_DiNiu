#include <gpiod.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include <poll.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace {
// 霍尔三相有效码表（六步换相序列）。
// 通过当前码与上一次码的相邻关系判断“正转一步/反转一步”。
constexpr int kCodeCount = 6;
const char *kForwardCodes[kCodeCount] = {"100", "110", "010", "011", "001", "101"};
}  // namespace

class SteerPositionClosedLoopNode : public rclcpp::Node {
  // 启动状态机：
  // 1) 找左限位（用于建立机械零点映射）
  // 2) 从左限位回到 0°
  // 3) 进入正常闭环控制
  enum class StartupState {
    kSeekingLeftLimit,
    kReturningToZero,
    kDone,
  };

public:
  SteerPositionClosedLoopNode() : Node("steer_closed_loop"), running_(true) {
    // ---------------- 硬件与 IO 参数 ----------------
    // 注意：这里的 GPIO 编号是 gpiod line offset（在树莓派场景通常可按 BCM 理解）。
    chip_path_ = declare_parameter<std::string>("chip_path", "/dev/gpiochip0");
    hu_offset_ = declare_parameter<int>("hu_gpio", 5);
    hv_offset_ = declare_parameter<int>("hv_gpio", 6);
    hw_offset_ = declare_parameter<int>("hw_gpio", 13);

    dir_offset_ = declare_parameter<int>("dir_gpio", 16);
    invert_dir_ = declare_parameter<bool>("invert_dir", true);
    brake_offset_ = declare_parameter<int>("brake_gpio", 12);
    // Brake is active-low by default: output GND means braking.
    brake_active_level_ = declare_parameter<int>("brake_active_level", 0);
    limit_left_offset_ = declare_parameter<int>("limit_left_gpio", 24);
    limit_right_offset_ = declare_parameter<int>("limit_right_gpio", 25);
    limit_active_level_ = declare_parameter<int>("limit_active_level", 1);
    positive_dir_hits_left_limit_ = declare_parameter<bool>("positive_dir_hits_left_limit", true);
    limit_use_bias_ = declare_parameter<bool>("limit_use_bias", true);
    limit_bias_ = declare_parameter<std::string>("limit_bias", "pull_down");
    limit_startup_grace_s_ = declare_parameter<double>("limit_startup_grace_s", 0.5);

    // ---------------- PWM 参数 ----------------
    pwm_chip_path_ = declare_parameter<std::string>("pwm_chip_path", "/sys/class/pwm/pwmchip0/");
    pwm_channel_ = declare_parameter<int>("pwm_channel", 1);
    pwm_period_ns_ = declare_parameter<long>("pwm_period_ns", 100000);

    // ---------------- ROS 话题 ----------------
    // cmd_topic: 目标角度（度）
    // feedback_topic: 实际角度（度）
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "target_steer");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "steer_position");

    // ---------------- 编码器换算参数 ----------------
    // 标定关系：-45°..+45° 约等于 2300 counts => 一圈约 9200 counts。
    encoder_counts_per_rev_ = declare_parameter<int>("encoder_counts_per_rev", 9200);
    invert_encoder_ = declare_parameter<bool>("invert_encoder", true);
    event_debounce_us_ = declare_parameter<int>("event_debounce_us", 100);

    // ---------------- 位置环 PID 与运动约束 ----------------
    control_hz_ = declare_parameter<double>("control_hz", 50.0);
    kp_ = declare_parameter<double>("kp", 9.5);
    ki_ = declare_parameter<double>("ki", 0.0);
    kd_ = declare_parameter<double>("kd", 0.0);
    i_max_ = declare_parameter<double>("i_max", 50.0);
    position_deadband_deg_ = declare_parameter<double>("position_deadband_deg", 0.5);
    max_pwm_percent_ = declare_parameter<double>("max_pwm_percent", 90.0);
    max_pwm_step_ = declare_parameter<double>("max_pwm_step", 12.0);
    min_effective_pwm_percent_ = declare_parameter<double>("min_effective_pwm_percent", 18.0);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.0);
    debug_hz_ = declare_parameter<double>("debug_hz", 2.0);

    // ---------------- 启动自标定参数 ----------------
    // 节点启动后默认先找左限位，将左限位定义为 left_limit_deg，再回到 0°。
    startup_auto_home_ = declare_parameter<bool>("startup_auto_home", true);
    left_limit_deg_ = declare_parameter<double>("left_limit_deg", 45.0);
    right_limit_deg_ = declare_parameter<double>("right_limit_deg", -45.0);
    home_seek_pwm_percent_ = declare_parameter<double>("home_seek_pwm_percent", 65.0);
    home_zero_tolerance_deg_ = declare_parameter<double>("home_zero_tolerance_deg", 1.0);
    // 额外零点修正（度），用于机械装配误差补偿。
    zero_offset_deg_ = declare_parameter<double>("zero_offset_deg", 0.0);

    validate_parameters();
    // 编码器步数 -> 角度系数（度/步）。
    deg_per_step_ = 360.0 / static_cast<double>(encoder_counts_per_rev_);

    // 初始化硬件，失败会直接抛异常并退出节点。
    open_pwm();
    open_gpio();

    // 订阅目标角、发布反馈角。
    cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      cmd_topic_, 10, std::bind(&SteerPositionClosedLoopNode::on_cmd, this, std::placeholders::_1));
    feedback_pub_ = create_publisher<std_msgs::msg::Float64>(feedback_topic_, 10);

    // 编码器事件线程：独立于控制线程，避免阻塞控制周期。
    encoder_thread_ = std::thread([this]() { this->encoder_loop(); });

    // 固定周期位置环控制。
    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SteerPositionClosedLoopNode::control_step, this));

    last_control_time_ = now();
    last_cmd_time_ = std::chrono::steady_clock::now();
    startup_time_ = std::chrono::steady_clock::now();
    next_debug_time_ = std::chrono::steady_clock::now();
    startup_state_ = startup_auto_home_ ? StartupState::kSeekingLeftLimit : StartupState::kDone;

    RCLCPP_INFO(
      get_logger(),
      "steer_closed_loop(position) started. cmd_topic=%s feedback_topic=%s invert_dir=%d",
      cmd_topic_.c_str(), feedback_topic_.c_str(), static_cast<int>(invert_dir_));
  }

  ~SteerPositionClosedLoopNode() override {
    // 析构顺序：停线程 -> 停电机 -> 释放 GPIO/PWM
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
    // 参数合法性检查：将配置错误尽早在启动阶段暴露。
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
    if (min_effective_pwm_percent_ < 0.0 || min_effective_pwm_percent_ > max_pwm_percent_) {
      throw std::runtime_error("min_effective_pwm_percent must be in [0, max_pwm_percent]");
    }
    if (brake_active_level_ != 0 && brake_active_level_ != 1) {
      throw std::runtime_error("brake_active_level must be 0 or 1");
    }
    if (position_deadband_deg_ < 0.0) {
      throw std::runtime_error("position_deadband_deg must be >= 0");
    }
    if (cmd_timeout_s_ < 0.0) {
      throw std::runtime_error("cmd_timeout_s must be >= 0");
    }
    if (pwm_period_ns_ <= 0) {
      throw std::runtime_error("pwm_period_ns must be > 0");
    }
    if (left_limit_deg_ <= right_limit_deg_) {
      throw std::runtime_error("left_limit_deg must be > right_limit_deg");
    }
    if (home_seek_pwm_percent_ <= 0.0 || home_seek_pwm_percent_ > max_pwm_percent_) {
      throw std::runtime_error("home_seek_pwm_percent must be in (0, max_pwm_percent]");
    }
    if (home_zero_tolerance_deg_ < 0.0) {
      throw std::runtime_error("home_zero_tolerance_deg must be >= 0");
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
    // 将三相霍尔码映射到序号，用于判断旋转方向与步进增减。
    for (int i = 0; i < kCodeCount; ++i) {
      if (std::strcmp(code, kForwardCodes[i]) == 0) {
        return i;
      }
    }
    return -1;
  }

  void on_cmd(const std_msgs::msg::Float64::SharedPtr msg) {
    // 按需求：回零完成前忽略外部角度命令，避免启动自标定被打断。
    if (startup_state_ != StartupState::kDone) {
      return;
    }
    target_position_deg_.store(
      clamp(msg->data, right_limit_deg_, left_limit_deg_),
      std::memory_order_relaxed);
    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  void control_step() {
    // 控制主循环：
    // 1) 发布当前反馈角
    // 2) 执行启动状态机（找左限位/回零）
    // 3) 正常状态执行位置 PID
    const auto tnow = now();
    const double dt = (tnow - last_control_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_control_time_ = tnow;

    const double measured_position = compute_position_deg();
    publish_position(measured_position);

    if (startup_state_ == StartupState::kSeekingLeftLimit) {
      // 阶段A：寻找左限位
      if (limit_left_active()) {
        // 触发左限位后，建立“编码器原始角 -> 物理角度”映射：
        // 使当前 raw_position 对应 left_limit_deg（默认 +45°）
        const double raw_position = compute_raw_position_deg();
        zero_offset_deg_ = left_limit_deg_ - raw_position;
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_output_ = 0.0;
        stop_motor();
        target_position_deg_.store(0.0, std::memory_order_relaxed);
        startup_state_ = StartupState::kReturningToZero;
        last_cmd_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(
          get_logger(),
          "left limit latched, calibrated: left=%.2f deg zero_offset=%.3f, returning to 0 deg",
          left_limit_deg_, zero_offset_deg_);
      } else {
        // 以固定 PWM 往“左限位方向”运动，直到触发限位。
        double seek_out = positive_dir_hits_left_limit_ ? home_seek_pwm_percent_ : -home_seek_pwm_percent_;
        if ((seek_out > 0.0 && positive_limit_active()) || (seek_out < 0.0 && negative_limit_active())) {
          seek_out = 0.0;
        }
        set_brake(std::abs(seek_out) < 1e-6);
        if (std::abs(seek_out) >= 1e-6) {
          set_direction(seek_out >= 0.0);
        }
        set_pwm(std::abs(seek_out));
        log_debug(measured_position, left_limit_deg_, left_limit_deg_ - measured_position, seek_out);
      }
      return;
    }

    double target_position = 0.0;
    if (startup_state_ == StartupState::kReturningToZero) {
      // 阶段B：从左限位回到 0°
      target_position = 0.0;
    } else {
      target_position = target_position_deg_.load(std::memory_order_relaxed);
      // 可选超时回中：当外部命令停止太久，目标回 0°。
      if (cmd_timeout_s_ > 0.0) {
        const auto age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_time_).count();
        if (age > cmd_timeout_s_) {
          target_position = 0.0;
        }
      }
    }
    target_position = clamp(target_position, right_limit_deg_, left_limit_deg_);
    run_position_pid(target_position, measured_position, dt);

    if (startup_state_ == StartupState::kReturningToZero) {
      // 回零判定：误差进入容差后，切换为正常运行态。
      const double home_error = target_position - measured_position;
      if (std::abs(home_error) <= home_zero_tolerance_deg_) {
        startup_state_ = StartupState::kDone;
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_output_ = 0.0;
        stop_motor();
        RCLCPP_INFO(get_logger(), "startup homing done: reached 0 deg");
      }
    }
  }

  double compute_position_deg() const {
    // 实际反馈角 = 编码器原始角 + 零点偏置
    return compute_raw_position_deg() + zero_offset_deg_;
  }

  double compute_raw_position_deg() const {
    const long step = read_signed_steps();
    return static_cast<double>(step) * deg_per_step_;
  }

  void publish_position(double position_deg) {
    std_msgs::msg::Float64 msg;
    msg.data = position_deg;
    feedback_pub_->publish(msg);
  }

  double run_position_pid(double target_position, double measured_position, double dt) {
    // 标准位置式 PID：
    // error = target - measured
    // out = kp*e + ki*∫e + kd*de/dt
    double error = target_position - measured_position;
    if (std::abs(error) < position_deadband_deg_) {
      error = 0.0;
    }

    // 先计算候选积分并限幅（防止积分无限增长）。
    const double integral_candidate = clamp(integral_ + error * dt, -i_max_, i_max_);
    const double derivative = (error - prev_error_) / dt;

    const double raw_candidate = kp_ * error + ki_ * integral_candidate + kd_ * derivative;
    const double saturated_candidate = clamp(raw_candidate, -max_pwm_percent_, max_pwm_percent_);

    // 条件积分抗饱和：只有在未饱和或“有助于脱离饱和”时才更新积分。
    if (
      raw_candidate == saturated_candidate ||
      (saturated_candidate >= max_pwm_percent_ && error < 0.0) ||
      (saturated_candidate <= -max_pwm_percent_ && error > 0.0)) {
      integral_ = integral_candidate;
    }

    const double raw_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    double out = clamp(raw_output, -max_pwm_percent_, max_pwm_percent_);

    // 输出斜率限制：抑制突变，减少机械冲击与电流尖峰。
    const double delta = out - prev_output_;
    if (delta > max_pwm_step_) {
      out = prev_output_ + max_pwm_step_;
    } else if (delta < -max_pwm_step_) {
      out = prev_output_ - max_pwm_step_;
    }
    prev_output_ = out;
    prev_error_ = error;

    // 限位保护：若继续朝触发方向运动，强制输出归零并清积分。
    if (out > 0.0 && positive_limit_active()) {
      out = 0.0;
      integral_ = 0.0;
    }
    if (out < 0.0 && negative_limit_active()) {
      out = 0.0;
      integral_ = 0.0;
    }

    // 最小有效 PWM：避免因静摩擦导致“有误差但不动”。
    if (std::abs(out) > 1e-6 && std::abs(out) < min_effective_pwm_percent_ && std::abs(error) > position_deadband_deg_) {
      out = (out > 0.0) ? min_effective_pwm_percent_ : -min_effective_pwm_percent_;
    }

    // 输出为零时抱闸；非零时松闸并按方向输出 PWM。
    set_brake(std::abs(out) < 1e-6);
    if (std::abs(out) >= 1e-6) {
      set_direction(out >= 0.0);
    }
    set_pwm(std::abs(out));
    log_debug(measured_position, target_position, error, out);
    return out;
  }

  void log_debug(double measured_position, double target_position, double error, double out) {
    if (std::chrono::steady_clock::now() >= next_debug_time_) {
      RCLCPP_INFO(
        get_logger(),
        "state=%s pos=%.2f target=%.2f err=%.2f out%%=%.2f L=%d R=%d B=%d",
        startup_state_name(), measured_position, target_position, error, out,
        static_cast<int>(limit_left_active()), static_cast<int>(limit_right_active()),
        static_cast<int>(brake_engaged_));
      const auto period = std::chrono::duration<double>(1.0 / debug_hz_);
      next_debug_time_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    }
  }

  const char *startup_state_name() const {
    switch (startup_state_) {
      case StartupState::kSeekingLeftLimit:
        return "seeking_left";
      case StartupState::kReturningToZero:
        return "returning_zero";
      case StartupState::kDone:
        return "ready";
      default:
        return "unknown";
    }
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
    // 启动初期限位毛刺屏蔽，防止上电瞬态误触发。
    if (limit_startup_grace_s_ <= 0.0) {
      return false;
    }
    const auto age =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - startup_time_).count();
    return age < limit_startup_grace_s_;
  }

  void encoder_loop() {
    // 事件线程：监听霍尔边沿并更新步数累计。
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
    // 每通道软件去抖。
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

    // 只接受“相邻一步”的合法序列，跳码直接忽略（抗干扰）。
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
    // sysfs PWM 初始化：export -> period -> duty=0 -> enable
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
    set_brake(true);
  }

  void open_gpio() {
    // 打开 GPIO 芯片并申请方向、刹车、霍尔、限位引脚。
    chip_ = gpiod_chip_open(chip_path_.c_str());
    if (!chip_) {
      throw std::runtime_error("open gpio chip failed: " + chip_path_);
    }

    dir_line_ = gpiod_chip_get_line(chip_, dir_offset_);
    brake_line_ = gpiod_chip_get_line(chip_, brake_offset_);
    hu_line_ = gpiod_chip_get_line(chip_, hu_offset_);
    hv_line_ = gpiod_chip_get_line(chip_, hv_offset_);
    hw_line_ = gpiod_chip_get_line(chip_, hw_offset_);
    limit_left_line_ = gpiod_chip_get_line(chip_, limit_left_offset_);
    limit_right_line_ = gpiod_chip_get_line(chip_, limit_right_offset_);
    if (!dir_line_ || !brake_line_ || !hu_line_ || !hv_line_ || !hw_line_ || !limit_left_line_ || !limit_right_line_) {
      throw std::runtime_error("get gpio line failed");
    }

    if (gpiod_line_request_output(dir_line_, "steer-dir", 0) != 0) {
      throw std::runtime_error("request dir output failed");
    }
    // 刹车默认上电即抱闸（active level）。
    if (gpiod_line_request_output(brake_line_, "steer-brake", brake_active_level_) != 0) {
      throw std::runtime_error("request brake output failed");
    }
    brake_engaged_ = true;
    if (gpiod_line_request_both_edges_events(hu_line_, "steer-hu") != 0 ||
        gpiod_line_request_both_edges_events(hv_line_, "steer-hv") != 0 ||
        gpiod_line_request_both_edges_events(hw_line_, "steer-hw") != 0) {
      throw std::runtime_error("request hall events failed");
    }
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    // 若库支持，优先使用硬件上下拉以降低输入漂移风险。
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
    if (brake_line_) {
      gpiod_line_release(brake_line_);
      brake_line_ = nullptr;
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
    int level = forward ? 1 : 0;
    if (invert_dir_) {
      level = 1 - level;
    }
    (void)gpiod_line_set_value(dir_line_, level);
  }

  void set_brake(bool engaged) {
    // 根据 brake_active_level_ 适配高有效/低有效刹车电路。
    if (!brake_line_) {
      return;
    }
    brake_engaged_ = engaged;
    const int level = engaged ? brake_active_level_ : (1 - brake_active_level_);
    (void)gpiod_line_set_value(brake_line_, level);
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
  bool invert_dir_{true};
  int brake_offset_{};
  int brake_active_level_{0};
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
  double position_deadband_deg_{};
  double max_pwm_percent_{};
  double max_pwm_step_{};
  double min_effective_pwm_percent_{};
  double cmd_timeout_s_{};
  double debug_hz_{};
  bool startup_auto_home_{true};
  double left_limit_deg_{45.0};
  double right_limit_deg_{-45.0};
  double home_seek_pwm_percent_{35.0};
  double home_zero_tolerance_deg_{1.0};
  double zero_offset_deg_{};

  double deg_per_step_{};
  double integral_{0.0};
  double prev_error_{0.0};
  double prev_output_{0.0};

  rclcpp::Time last_control_time_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::chrono::steady_clock::time_point startup_time_;
  std::chrono::steady_clock::time_point next_debug_time_;
  StartupState startup_state_{StartupState::kDone};

  std::atomic<bool> running_;
  std::atomic<double> target_position_deg_{0.0};
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
  gpiod_line *brake_line_{nullptr};
  gpiod_line *hu_line_{nullptr};
  gpiod_line *hv_line_{nullptr};
  gpiod_line *hw_line_{nullptr};
  gpiod_line *limit_left_line_{nullptr};
  gpiod_line *limit_right_line_{nullptr};
  bool brake_engaged_{true};

  std::thread encoder_thread_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SteerPositionClosedLoopNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "steer_closed_loop startup error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
