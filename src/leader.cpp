// Copyright RRL NYU
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Purpose:
// Leader-side teleoperation bridge with force feedback and connection monitoring.
// Runs on the PC connected to the *leader* arm. Receives external efforts
// from the follower over ROS 2 and applies -k * efforts to the leader.
// Publishes the leader joint states for the follower to track.
// Monitors connection health and performs emergency stop if connection lost.
//
// Network topics:
//   Subscribes: /follower/external_efforts   (sensor_msgs/JointState.effort)
//              /follower/ready               (std_msgs/Bool)
//   Publishes : /leader/joint_states         (sensor_msgs/JointState position/velocity)
//              /leader/ready                 (std_msgs/Bool)

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "libtrossen_arm/trossen_arm.hpp"

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;

  // --- ROS 2 initialization --------------------------------------------------
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("leader_bridge");

  // QoS: keep only the latest sample; drop rather than retry; no persistence
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
               .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  std::cout << "Initializing the leader driver..." << std::endl;
  trossen_arm::TrossenArmDriver driver_leader;

  std::cout << "Configuring the leader driver..." << std::endl;
  driver_leader.configure(
    trossen_arm::Model::wxai_v0,
    trossen_arm::StandardEndEffector::wxai_v0_leader,
    "192.168.1.2",
    false
  );

  const int num_joints = driver_leader.get_num_joints();

  std::cout << "Moving leader to home position..." << std::endl;
  driver_leader.set_all_modes(trossen_arm::Mode::position);
  {
    std::vector<double> home {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0, 0.0};
    home.resize(num_joints, 0.0);
    driver_leader.set_all_positions(home, 2.0f, true);
  }

  // --- Synchronization flags -------------------------------------------------
  std::atomic<bool> follower_ready{false};
  std::atomic<bool> received_first_effort{false};

  // --- Connection monitoring -------------------------------------------------
  std::atomic<std::chrono::steady_clock::time_point> last_effort_time{std::chrono::steady_clock::now()};
  const auto connection_timeout = 100ms;  // Consider connection lost after 100ms

  // --- ROS 2 comms: latest follower efforts in, leader states out -----------
  std::vector<double> tau_shared(num_joints, 0.0);
  std::mutex tau_mtx;

  // Subscriber: follower ready signal
  auto sub_ready = node->create_subscription<std_msgs::msg::Bool>(
    "/follower/ready", qos,
    [&](std_msgs::msg::Bool::SharedPtr msg){
      if (msg->data) {
        follower_ready.store(true);
        std::cout << "Received follower ready signal!" << std::endl;
      }
    });

  // Subscriber: follower external efforts
  auto sub_tau = node->create_subscription<sensor_msgs::msg::JointState>(
    "/follower/external_efforts", qos,
    [&](sensor_msgs::msg::JointState::SharedPtr msg){
      std::lock_guard<std::mutex> lk(tau_mtx);
      const size_t m = std::min(static_cast<size_t>(num_joints), msg->effort.size());
      std::copy_n(msg->effort.begin(), m, tau_shared.begin());
      if (m < static_cast<size_t>(num_joints)) {
        std::fill(tau_shared.begin() + m, tau_shared.end(), 0.0);
      }
      received_first_effort.store(true);
      last_effort_time.store(std::chrono::steady_clock::now());  // Update timestamp
    });

  // Publisher: leader joint states
  auto pub_js = node->create_publisher<sensor_msgs::msg::JointState>("/leader/joint_states", qos);

  // Publisher: leader ready signal
  auto pub_ready = node->create_publisher<std_msgs::msg::Bool>("/leader/ready", qos);

  // Preallocated joint state message
  sensor_msgs::msg::JointState js;
  js.name.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) js.name[i] = "J" + std::to_string(i+1);
  js.position.resize(num_joints);
  js.velocity.resize(num_joints);

  // Spin ROS callbacks on a separate thread
  std::thread spin_thr([&]{ rclcpp::spin(node); });

  // --- Wait for follower to be ready -----------------------------------------
  std::cout << "Leader is ready. Waiting for follower..." << std::endl;
  
  // Publish our ready signal periodically
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  
  while (rclcpp::ok() && !follower_ready.load()) {
    pub_ready->publish(ready_msg);
    std::this_thread::sleep_for(100ms);
  }

  if (!rclcpp::ok()) {
    std::cout << "Shutdown requested before synchronization complete." << std::endl;
    rclcpp::shutdown();
    spin_thr.join();
    return 1;
  }

  std::cout << "Both systems ready. Waiting for initial data exchange..." << std::endl;
  
  // Wait for first effort message from follower
  auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !received_first_effort.load()) {
    // Keep publishing leader state to help follower
    auto q  = driver_leader.get_all_positions();
    auto qd = driver_leader.get_all_velocities();
    std::copy(q.begin(),  q.end(),  js.position.begin());
    std::copy(qd.begin(), qd.end(), js.velocity.begin());
    js.header.stamp = node->now();
    pub_js->publish(js);
    
    std::this_thread::sleep_for(10ms);
    
    // Timeout after 5 seconds
    if (std::chrono::steady_clock::now() - wait_start > 5s) {
      std::cout << "Warning: No effort data received from follower after 5s. Proceeding anyway..." << std::endl;
      break;
    }
  }

  std::cout << "Data exchange established. Switching to teleoperation mode in 1 second..." << std::endl;
  std::this_thread::sleep_for(1s);

  std::cout << "Switching leader to external-effort mode..." << std::endl;
  driver_leader.set_all_modes(trossen_arm::Mode::external_effort);

  std::cout << "Leader now in external-effort mode. Starting teleoperation..." << std::endl;

  // Reset connection monitoring timer
  last_effort_time.store(std::chrono::steady_clock::now());

  // --- Teleoperation loop ----------------------------------------------------
  const double force_feedback_gain = 0.1;  // conservative, safe default
  const auto   teleoperation_time  = 20s;

  std::cout << "Starting teleoperation for " 
            << std::chrono::duration_cast<std::chrono::seconds>(teleoperation_time).count()
            << " seconds with k = " << force_feedback_gain << " ..." << std::endl;
  std::cout << "Connection monitoring active (timeout: " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(connection_timeout).count() 
            << "ms)" << std::endl;

  auto start_time = std::chrono::steady_clock::now();
  auto end_time   = start_time + teleoperation_time;

  std::vector<double> tau_local(num_joints, 0.0);  // snapshot buffer
  std::vector<double> tau_cmd(num_joints, 0.0);    // reusable command buffer

  bool connection_lost = false;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
    // Check connection health
    auto time_since_last_effort = std::chrono::steady_clock::now() - last_effort_time.load();
    if (time_since_last_effort > connection_timeout) {
      std::cerr << "\n!!! CONNECTION LOST TO FOLLOWER !!!" << std::endl;
      std::cerr << "Last effort message received " 
                << std::chrono::duration_cast<std::chrono::milliseconds>(time_since_last_effort).count() 
                << "ms ago" << std::endl;
      std::cerr << "Initiating emergency stop procedure..." << std::endl;
      connection_lost = true;
      break;
    }

    // Take a snapshot of latest follower efforts
    {
      std::lock_guard<std::mutex> lk(tau_mtx);
      tau_local = tau_shared;  // copy num_joints doubles
    }

    // Map follower efforts to leader torques (negated, scaled)
    for (int i = 0; i < num_joints; ++i) {
      tau_cmd[i] = -force_feedback_gain * tau_local[i];
    }

    // Apply to leader (non-blocking, immediate)
    driver_leader.set_all_external_efforts(tau_cmd, 0.0f, false);

    // Publish leader positions/velocities for the follower
    auto q  = driver_leader.get_all_positions();
    auto qd = driver_leader.get_all_velocities();
    std::copy(q.begin(),  q.end(),  js.position.begin());
    std::copy(qd.begin(), qd.end(), js.velocity.begin());
    js.header.stamp = node->now();
    pub_js->publish(js);

    std::this_thread::sleep_for(1ms);
  }

  // --- Emergency response or normal shutdown ---------------------------------
  if (connection_lost) {
    std::cout << "\n=== EMERGENCY STOP SEQUENCE ===" << std::endl;
    
    // Switch to position mode immediately
    std::cout << "Switching leader to position mode..." << std::endl;
    driver_leader.set_all_modes(trossen_arm::Mode::position);
    
    // Hold current position briefly to stabilize
    std::cout << "Stabilizing at current position..." << std::endl;
    auto current_pos = driver_leader.get_all_positions();
    driver_leader.set_all_positions(current_pos, 0.5f, true);
    
    std::cout << "Emergency hold complete." << std::endl;
  } else {
    std::cout << "Teleoperation completed normally." << std::endl;
  }

  // --- Return to home & sleep -----------------------------------------------
  std::cout << "Moving leader to home..." << std::endl;
  driver_leader.set_all_modes(trossen_arm::Mode::position);
  {
    std::vector<double> home {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0, 0.0};
    home.resize(num_joints, 0.0);
    driver_leader.set_all_positions(home, 2.0f, true);
  }

  std::cout << "Moving leader to sleep position..." << std::endl;
  driver_leader.set_all_positions(std::vector<double>(num_joints, 0.0), 2.0f, true);

  // --- Shutdown --------------------------------------------------------------
  std::cout << "Leader shutdown complete." << std::endl;
  rclcpp::shutdown();
  spin_thr.join();
  return connection_lost ? 2 : 0;  // Return error code if connection was lost
}