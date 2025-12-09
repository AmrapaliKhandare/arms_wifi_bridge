// ROS 2 Follower Node - Communicates via TCP, NO libtrossen dependency!

#include <chrono>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tcp_client.hpp"

using namespace std::chrono_literals;

class FollowerBridge : public rclcpp::Node
{
public:
  FollowerBridge()
  : Node("follower_bridge"),
    leader_ready_(false),
    received_first_state_(false),
    connection_lost_(false),
    teleoperation_active_(false),
    connection_timeout_(100ms),
    teleoperation_time_(20s)
  {
    // Declare parameters
    this->declare_parameter<std::string>("tcp_host", "localhost");
    this->declare_parameter<int>("tcp_port", 5002);
    
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
    
    q_shared_.resize(num_joints_, 0.0);
    qd_shared_.resize(num_joints_, 0.0);
    q_local_.resize(num_joints_, 0.0);
    qd_local_.resize(num_joints_, 0.0);

    // Create publishers
    pub_efforts_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/follower/external_efforts", qos);
    pub_ready_ = this->create_publisher<std_msgs::msg::Bool>(
      "/follower/ready", qos);

    // Create subscribers
    sub_ready_ = this->create_subscription<std_msgs::msg::Bool>(
      "/leader/ready", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        this->leader_ready_callback(msg);
      });

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        this->joint_state_callback(msg);
      });

    // Prepare effort message
    effort_msg_.name.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      effort_msg_.name[i] = "J" + std::to_string(i + 1);
    }
    effort_msg_.effort.resize(num_joints_);

    // Create timers
    ready_timer_ = this->create_wall_timer(
      100ms, [this]() { this->publish_ready(); });

    sync_timer_ = this->create_wall_timer(
      10ms, [this]() { this->synchronization_loop(); });

    teleoperation_timer_ = nullptr;

    last_state_time_ = this->now();
    sync_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Follower bridge initialized with %zu joints", num_joints_);
  }

  ~FollowerBridge()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down follower bridge...");
    
    // Cancel all timers before cleanup to prevent deadlock
    if (teleoperation_timer_) {
      teleoperation_timer_->cancel();
    }
    if (sync_timer_) {
      sync_timer_->cancel();
    }
    if (ready_timer_) {
      ready_timer_->cancel();
    }
    
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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_efforts_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ready_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

  // Timers
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::TimerBase::SharedPtr teleoperation_timer_;

  // Synchronization
  std::atomic<bool> leader_ready_;
  std::atomic<bool> received_first_state_;
  rclcpp::Time last_state_time_;
  rclcpp::Time sync_start_time_;
  rclcpp::Time teleoperation_start_time_;

  // Teleoperation state
  bool teleoperation_active_;
  bool connection_lost_;
  
  // Control parameters
  const std::chrono::milliseconds connection_timeout_;
  const std::chrono::seconds teleoperation_time_;

  // Data buffers
  std::vector<double> q_shared_;
  std::vector<double> qd_shared_;
  std::vector<double> q_local_;
  std::vector<double> qd_local_;
  std::mutex q_mtx_;
  sensor_msgs::msg::JointState effort_msg_;

  void leader_ready_callback(std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !leader_ready_.load()) {
      leader_ready_.store(true);
      RCLCPP_INFO(this->get_logger(), "Received leader ready signal!");
    }
  }

  void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(q_mtx_);
    
    // SAFETY: Validate incoming data size
    if (msg->position.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty position data");
      return;
    }
    
    const size_t mp = std::min(num_joints_, msg->position.size());
    const size_t mv = std::min(num_joints_, msg->velocity.size());
    
    std::copy_n(msg->position.begin(), mp, q_shared_.begin());
    if (mp < num_joints_) {
      std::fill(q_shared_.begin() + mp, q_shared_.end(), 0.0);
    }
    
    std::copy_n(msg->velocity.begin(), mv, qd_shared_.begin());
    if (mv < num_joints_) {
      std::fill(qd_shared_.begin() + mv, qd_shared_.end(), 0.0);
    }
    
    received_first_state_.store(true);
    last_state_time_ = this->now();
  }

  void publish_ready() {
    if (!leader_ready_.load() || !received_first_state_.load()) {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      pub_ready_->publish(msg);
    }
  }

  void synchronization_loop() {
    if (!leader_ready_.load()) {
      return;
    }

    if (!received_first_state_.load()) {
      try {
        auto tau = tcp_client_->get_efforts();
        const size_t m = std::min(num_joints_, tau.size());
        std::copy_n(tau.begin(), m, effort_msg_.effort.begin());
        if (m < num_joints_) {
          std::fill(effort_msg_.effort.begin() + m, effort_msg_.effort.end(), 0.0);
        }
        effort_msg_.header.stamp = this->now();
        pub_efforts_->publish(effort_msg_);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "TCP error: %s", e.what());
      }

      if ((this->now() - sync_start_time_).seconds() > 5.0) {
        RCLCPP_WARN(this->get_logger(), "No joint state data after 5s. Proceeding anyway...");
        received_first_state_.store(true);
      }
      return;
    }

    if (leader_ready_.load() && received_first_state_.load() && !teleoperation_active_) {
      start_teleoperation();
    }
  }

  void start_teleoperation() {
    sync_timer_->cancel();
    ready_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), 
      "Data exchange established. Starting teleoperation in 1 second...");
    
    std::this_thread::sleep_for(1s);

    last_state_time_ = this->now();
    teleoperation_start_time_ = this->now();
    teleoperation_active_ = true;

    RCLCPP_INFO(this->get_logger(), 
      "Tracking leader for %ld seconds...", 
      teleoperation_time_.count());

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

    auto time_since_last = this->now() - last_state_time_;
    if (time_since_last.seconds() > (connection_timeout_.count() / 1000.0)) {
      RCLCPP_ERROR(this->get_logger(), "CONNECTION LOST TO LEADER!");
      connection_lost_ = true;
      teleoperation_timer_->cancel();
      teleoperation_active_ = false;
      emergency_stop();
      return_to_home_and_sleep();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(q_mtx_);
      q_local_ = q_shared_;
      qd_local_ = qd_shared_;
    }

    try {
      tcp_client_->set_positions(q_local_);
      
      auto tau = tcp_client_->get_efforts();
      const size_t m = std::min(num_joints_, tau.size());
      std::copy_n(tau.begin(), m, effort_msg_.effort.begin());
      if (m < num_joints_) {
        std::fill(effort_msg_.effort.begin() + m, effort_msg_.effort.end(), 0.0);
      }
      effort_msg_.header.stamp = this->now();
      pub_efforts_->publish(effort_msg_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "TCP error in control loop: %s", e.what());
      // CRITICAL FIX: Stop on TCP failure
      connection_lost_ = true;
      teleoperation_timer_->cancel();
      teleoperation_active_ = false;
      emergency_stop();
      return_to_home_and_sleep();
      return;
    }
  }

  void emergency_stop() {
    RCLCPP_INFO(this->get_logger(), "=== EMERGENCY STOP ===");
    try {
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
  auto node = std::make_shared<FollowerBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}