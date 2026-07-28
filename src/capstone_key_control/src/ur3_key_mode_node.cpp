// ROS 2 migration of the input boundary used by iCIRLab phri_real.cpp:
// one terminal-reader thread and one shared process_key() path for local and
// remote keys.  Robot control is intentionally left to the planner/executor.

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cctype>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class Ur3KeyModeNode : public rclcpp::Node
{
public:
  Ur3KeyModeNode()
  : Node("ur3_key_mode_node")
  {
    mode_publisher_ = create_publisher<std_msgs::msg::Int16>(
      "/capstone_planning/planner_mode", rclcpp::QoS(1).reliable());
    goal_publisher_ = create_publisher<std_msgs::msg::String>(
      "/capstone_planning/named_goal_request", rclcpp::QoS(1).reliable());
    remote_key_subscription_ = create_subscription<std_msgs::msg::String>(
      "/capstone_planning/remote_key", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (!message->data.empty()) {
          process_key(message->data.front());
        }
      });
    cancel_client_ = create_client<std_srvs::srv::Trigger>(
      "/trajectory_executor/cancel");

    print_help();
    if (isatty(STDIN_FILENO) != 0) {
      input_thread_ = std::thread(&Ur3KeyModeNode::mode_change_reader, this);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "stdin is not a TTY; use /capstone_planning/remote_key instead.");
    }
  }

  ~Ur3KeyModeNode() override
  {
    stop_requested_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  class RawTerminal
  {
public:
    RawTerminal()
    {
      valid_ = tcgetattr(STDIN_FILENO, &saved_) == 0;
      if (!valid_) {
        return;
      }
      termios raw = saved_;
      raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      valid_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }

    ~RawTerminal()
    {
      if (valid_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
      }
    }

    bool valid() const {return valid_;}

private:
    termios saved_{};
    bool valid_{false};
  };

  void mode_change_reader()
  {
    RawTerminal terminal;
    if (!terminal.valid()) {
      RCLCPP_ERROR(get_logger(), "Could not enter raw keyboard mode.");
      return;
    }

    while (rclcpp::ok() && !stop_requested_.load()) {
      pollfd descriptor{};
      descriptor.fd = STDIN_FILENO;
      descriptor.events = POLLIN;
      const int result = poll(&descriptor, 1, 100);
      if (result <= 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }
      char key = '\0';
      if (read(STDIN_FILENO, &key, 1) == 1) {
        process_key(key);
      }
    }
  }

  void process_key(char raw_key)
  {
    const char key = static_cast<char>(
      std::tolower(static_cast<unsigned char>(raw_key)));
    switch (key) {
      case '1':
        publish_mode(1, "MPlib");
        break;
      case '2':
        publish_mode(2, "MoveIt 2");
        break;
      case 'h':
        publish_goal("home");
        break;
      case 's':
        publish_goal("scan");
        break;
      case 'l':
        publish_goal("low_pick");
        break;
      case 'f':
        request_cancel();
        break;
      case 'p':
        {
          const int mode = active_mode_.load();
          RCLCPP_INFO(
            get_logger(), "Current planner mode: %d (%s)",
            mode, mode == 1 ? "MPlib" : "MoveIt 2");
          break;
        }
      case '?':
        print_help();
        break;
      case 'q':
        RCLCPP_INFO(get_logger(), "Keyboard node shutdown requested.");
        stop_requested_.store(true);
        rclcpp::shutdown();
        break;
      case '\n':
      case '\r':
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown key '%c'; press ? for help.", key);
        break;
    }
  }

  void publish_mode(int mode, const char * label)
  {
    active_mode_.store(mode);
    std_msgs::msg::Int16 message;
    message.data = static_cast<int16_t>(mode);
    mode_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Planner %d selected: %s.", mode, label);
  }

  void publish_goal(const std::string & name)
  {
    // Re-publish the selected mode with each goal so late-starting planner
    // nodes cannot interpret a command using a stale/default mode.
    const int mode = active_mode_.load();
    publish_mode(mode, mode == 1 ? "MPlib" : "MoveIt 2");
    std_msgs::msg::String message;
    // Include the mode in the command itself. DDS ordering is guaranteed per
    // topic, not across the separate mode and goal topics.
    message.data = std::to_string(mode) + ":" + name;
    goal_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Named goal requested: %s.", name.c_str());
  }

  void request_cancel()
  {
    if (!cancel_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "trajectory_executor cancel service is not ready.");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    cancel_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        }
      });
  }

  void print_help()
  {
    RCLCPP_INFO(
      get_logger(),
      "\nUR3 key mode\n"
      "  1: MPlib planner     2: MoveIt 2 planner\n"
      "  h: Home             s: Scan             l: Low-pick\n"
      "  f: Cancel controller goal               p: Print mode\n"
      "  ?: Help             q: Quit the keyboard/session");
  }

  std::atomic_bool stop_requested_{false};
  std::atomic_int active_mode_{1};
  std::thread input_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    remote_key_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ur3KeyModeNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
