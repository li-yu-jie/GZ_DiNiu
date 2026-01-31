#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gpiod.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <string>
#include <thread>

class EncoderVelNode : public rclcpp::Node {
public:
  EncoderVelNode()
  : Node("encoder_vel_node"), running_(true), count_(0) {
    chip_name_ = this->declare_parameter<std::string>("chip", "/dev/gpiochip0");
    gpio_a_ = this->declare_parameter<int>("gpio_a", 21);
    gpio_b_ = this->declare_parameter<int>("gpio_b", 20);
    counts_per_rev_ = this->declare_parameter<int>("counts_per_rev", 600);
    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.165);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 50.0);
    topic_ = this->declare_parameter<std::string>("topic", "linear_velocity");

    if (counts_per_rev_ <= 0) {
      throw std::runtime_error("counts_per_rev must be > 0");
    }
    if (wheel_radius_m_ <= 0.0) {
      throw std::runtime_error("wheel_radius_m must be > 0");
    }
    if (publish_hz_ <= 0.0) {
      throw std::runtime_error("publish_hz must be > 0");
    }

    publisher_ = this->create_publisher<std_msgs::msg::Float64>(topic_, 10);

    open_gpio();

    event_thread_ = std::thread([this]() { this->event_loop(); });

    last_time_ = this->get_clock()->now();
    last_count_ = count_.load();

    auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EncoderVelNode::publish_velocity, this));
  }

  ~EncoderVelNode() override {
    running_.store(false);
    if (event_thread_.joinable()) {
      event_thread_.join();
    }
    close_gpio();
  }

private:
  void open_gpio() {
    chip_ = gpiod_chip_open(chip_name_.c_str());
    if (!chip_) {
      throw std::runtime_error("gpiod_chip_open failed for " + chip_name_);
    }

    line_a_ = gpiod_chip_get_line(chip_, gpio_a_);
    line_b_ = gpiod_chip_get_line(chip_, gpio_b_);
    if (!line_a_ || !line_b_) {
      close_gpio();
      throw std::runtime_error("gpiod_chip_get_line failed");
    }

    if (gpiod_line_request_input(line_b_, "enc_b") < 0) {
      close_gpio();
      throw std::runtime_error("gpiod_line_request_input failed for B");
    }

    if (gpiod_line_request_rising_edge_events(line_a_, "enc_a_rise") < 0) {
      close_gpio();
      throw std::runtime_error("gpiod_line_request_rising_edge_events failed for A");
    }
  }

  void close_gpio() {
    if (line_a_) {
      gpiod_line_release(line_a_);
      line_a_ = nullptr;
    }
    if (line_b_) {
      gpiod_line_release(line_b_);
      line_b_ = nullptr;
    }
    if (chip_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  void event_loop() {
    while (running_.load()) {
      timespec timeout{};
      timeout.tv_sec = 0;
      timeout.tv_nsec = 100 * 1000 * 1000; // 100ms
      int ret = gpiod_line_event_wait(line_a_, &timeout);
      if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "event_wait failed");
        break;
      }
      if (ret == 0) {
        continue;
      }

      gpiod_line_event ev;
      if (gpiod_line_event_read(line_a_, &ev) < 0) {
        RCLCPP_ERROR(this->get_logger(), "event_read failed");
        break;
      }

      int b = gpiod_line_get_value(line_b_);
      if (b < 0) {
        RCLCPP_ERROR(this->get_logger(), "get_value B failed");
        break;
      }

      if (b == 0) {
        count_.fetch_add(1);
      } else {
        count_.fetch_sub(1);
      }
    }
  }

  void publish_velocity() {
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    long long current_count = count_.load();
    long long delta = current_count - last_count_;

    double revs = static_cast<double>(delta) / static_cast<double>(counts_per_rev_);
    double distance = revs * (2.0 * M_PI * wheel_radius_m_);
    double velocity = distance / dt;

    std_msgs::msg::Float64 msg;
    msg.data = velocity;
    publisher_->publish(msg);

    last_time_ = now;
    last_count_ = current_count;
  }

  std::string chip_name_;
  int gpio_a_{};
  int gpio_b_{};
  int counts_per_rev_{};
  double wheel_radius_m_{};
  double publish_hz_{};
  std::string topic_;

  gpiod_chip* chip_ = nullptr;
  gpiod_line* line_a_ = nullptr;
  gpiod_line* line_b_ = nullptr;

  std::atomic<bool> running_;
  std::atomic<long long> count_;
  std::thread event_thread_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_time_;
  long long last_count_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EncoderVelNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Startup error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
