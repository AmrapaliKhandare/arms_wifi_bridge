// ROS 2 Leader Node - Communicates via TCP, NO libtrossen dependency!

#include <chrono>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tcp_client.hpp"

using namespace std::chrono_literals;

class LeaderBridge : public rclcpp::Node
{
public:
  LeaderBridge()
  : Node("leader_bridge"),
    follower_ready_(false),
    received_first_effort_(false),
    connection_lost_(false),
    teleoperation_active_(false),
    force_feedback_gain_(0.1),
    connection_timeout_(100ms),
    teleoperation_time_(20s)
  {
    // Declare parameters
    this->declare_parameter<std::string>("tcp_host", "localhost");
    this->declare_parameter<int>("tcp_port", 5001);
    
    std::string tcp_host = this->get_parameter("tcp_host").as_string();
    int tcp_port = this->get_parameter("tcp_port").as_int();

    // QoS settings
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Connect to arm control server via TCP
    RCLCPP_INFO(this->get_logger(), "Connecting to arm server at %s:%d...", 
                tcp_host.c_str(), tcp_port);
    
    tcp_client_ = std::make_unique<TCPClient>(tcp_host, tcp_port);
    
    if (!tcp_client_->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to arm server!");
      throw std::runtime_error("TCP connection failed");
    }
    
    RCLCPP_INFO(this->get_logger(), "Connected to arm server successfully");

    // Get number of joints
    auto pos = tcp_client_->get_positions();
    num_joints_ = pos.size();
    
    tau_shared_.resize(num_joints_, 0.0);
    tau_local_.resize(num_joints_, 0.0);
    tau_cmd_.resize(num_joints_, 0.0);

    // Create publishers
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/leader/joint_states", qos);
    pub_ready_ = this->create_publisher<std_msgs::msg::Bool>(
      "/leader/ready", qos);

    // Create subscribers
    sub_ready_ = this->create_subscription<std_msgs::msg::Bool>(
      "/follower/ready", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        this->follower_ready_callback(msg);
      });

    sub_efforts_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/follower/external_efforts", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        this->effort_callback(msg);
      });

    // Prepare joint state message
    js_msg_.name.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      js_msg_.name[i] = "J" + std::to_string(i + 1);
    }
    js_msg_.position.resize(num_joints_);
    js_msg_.velocity.resize(num_joints_);

    // Create timers
    ready_timer_ = this->create_wall_timer(
      100ms, [this]() { this->publish_ready(); });

    sync_timer_ = this->create_wall_timer(
      10ms, [this]() { this->synchronization_loop(); });

    teleoperation_timer_ = nullptr;

    last_effort_time_ = this->now();
    sync_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Leader bridge initialized with %zu joints", num_joints_);
  }

  ~LeaderBridge()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down leader bridge...");
    if (teleoperation_active_) {
      emergency_stop();
    }
    return_to_home_and_sleep();
  }

private:
  // TCP client instead of direct hardware access
  std::unique_ptr<TCPClient> tcp_client_;
  size_t num_joints_;

  // Publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ready_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_efforts_;

  // Timers
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::TimerBase::SharedPtr teleoperation_timer_;

  // Synchronization
  std::atomic<bool> follower_ready_;
  std::atomic<bool> received_first_effort_;
  rclcpp::Time last_effort_time_;
  rclcpp::Time sync_start_time_;
  rclcpp::Time teleoperation_start_time_;

  // Teleoperation state
  bool teleoperation_active_;
  bool connection_lost_;
  
  // Control parameters
  const double force_feedback_gain_;
  const std::chrono::milliseconds connection_timeout_;
  const std::chrono::seconds teleoperation_time_;

  // Data buffers
  std::vector<double> tau_shared_;
  std::vector<double> tau_local_;
  std::vector<double> tau_cmd_;
  std::mutex tau_mtx_;
  sensor_msgs::msg::JointState js_msg_;

  void follower_ready_callback(std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !follower_ready_.load()) {
      follower_ready_.store(true);
      RCLCPP_INFO(this->get_logger(), "Received follower ready signal!");
    }
  }

  void effort_callback(sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(tau_mtx_);
    const size_t m = std::min(num_joints_, msg->effort.size());
    std::copy_n(msg->effort.begin(), m, tau_shared_.begin());
    if (m < num_joints_) {
      std::fill(tau_shared_.begin() + m, tau_shared_.end(), 0.0);
    }
    received_first_effort_.store(true);
    last_effort_time_ = this->now();
  }

  void publish_ready() {
    if (!follower_ready_.load() || !received_first_effort_.load()) {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      pub_ready_->publish(msg);
    }
  }

  void synchronization_loop() {
    if (!follower_ready_.load()) {
      return;
    }

    if (!received_first_effort_.load()) {
      try {
        auto q = tcp_client_->get_positions();
        auto qd = tcp_client_->get_velocities();
        std::copy(q.begin(), q.end(), js_msg_.position.begin());
        std::copy(qd.begin(), qd.end(), js_msg_.velocity.begin());
        js_msg_.header.stamp = this->now();
        pub_joint_states_->publish(js_msg_);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "TCP error: %s", e.what());
      }

      if ((this->now() - sync_start_time_).seconds() > 5.0) {
        RCLCPP_WARN(this->get_logger(), "No effort data after 5s. Proceeding anyway...");
        received_first_effort_.store(true);
      }
      return;
    }

    if (follower_ready_.load() && received_first_effort_.load() && !teleoperation_active_) {
      start_teleoperation();
    }
  }

  void start_teleoperation() {
    sync_timer_->cancel();
    ready_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), 
      "Data exchange established. Starting teleoperation in 1 second...");
    
    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(this->get_logger(), "Switching leader to external-effort mode...");
    try {
      tcp_client_->set_mode_effort();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set mode: %s", e.what());
      return;
    }
    
    last_effort_time_ = this->now();
    teleoperation_start_time_ = this->now();
    teleoperation_active_ = true;

    RCLCPP_INFO(this->get_logger(), 
      "Teleoperation started for %ld seconds with k = %.2f",
      teleoperation_time_.count(), force_feedback_gain_);

    teleoperation_timer_ = this->create_wall_timer(
      1ms, [this]() { this->teleoperation_loop(); });
  }

  void teleoperation_loop() {
    auto elapsed = this->now() - teleoperation_start_time_;
    if (elapsed.seconds() >= static_cast<double>(teleoperation_time_.count())) {
      RCLCPP_INFO(this->get_logger(), "Teleoperation completed normally.");
      teleoperation_timer_->cancel();
      teleoperation_active_ = false;
      return_to_home_and_sleep();
      return;
    }

    auto time_since_last = this->now() - last_effort_time_;
    if (time_since_last.seconds() > (connection_timeout_.count() / 1000.0)) {
      RCLCPP_ERROR(this->get_logger(), "CONNECTION LOST TO FOLLOWER!");
      connection_lost_ = true;
      teleoperation_timer_->cancel();
      teleoperation_active_ = false;
      emergency_stop();
      return_to_home_and_sleep();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(tau_mtx_);
      tau_local_ = tau_shared_;
    }

    for (size_t i = 0; i < num_joints_; ++i) {
      tau_cmd_[i] = -force_feedback_gain_ * tau_local_[i];
    }

    try {
      tcp_client_->set_efforts(tau_cmd_);
      auto q = tcp_client_->get_positions();
      auto qd = tcp_client_->get_velocities();
      std::copy(q.begin(), q.end(), js_msg_.position.begin());
      std::copy(qd.begin(), qd.end(), js_msg_.velocity.begin());
      js_msg_.header.stamp = this->now();
      pub_joint_states_->publish(js_msg_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "TCP error in control loop: %s", e.what());
    }
  }

  void emergency_stop() {
    RCLCPP_INFO(this->get_logger(), "=== EMERGENCY STOP ===");
    try {
      tcp_client_->set_mode_position();
      std::this_thread::sleep_for(500ms);
    } catch (...) {}
  }

  void return_to_home_and_sleep() {
    RCLCPP_INFO(this->get_logger(), "Moving to home and sleep...");
    try {
      tcp_client_->move_home();
      tcp_client_->move_sleep();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in shutdown: %s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LeaderBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}