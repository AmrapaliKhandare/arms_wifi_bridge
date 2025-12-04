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
// Follower-side teleoperation bridge with connection monitoring.
// Runs on the PC connected to the *follower* arm. Subscribes to the leader's
// joint states over ROS 2 and commands the follower in POSITION mode to track.
// Publishes the follower's external efforts back to the leader.
// Monitors connection health and performs emergency stop if connection lost.
//
// Network topics:
//   Subscribes: /leader/joint_states         (sensor_msgs/JointState position/velocity)
//              /leader/ready                 (std_msgs/Bool)
//   Publishes : /follower/external_efforts   (sensor_msgs/JointState.effort)
//              /follower/ready               (std_msgs/Bool)

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "libtrossen_arm/trossen_arm.hpp"

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;

  // --- ROS 2 initialization --------------------------------------------------
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("follower_bridge");

  // QoS: keep only the latest sample; drop rather than retry; no persistence
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
               .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  std::cout << "Initializing the follower driver..." << std::endl;
  trossen_arm::TrossenArmDriver driver_follower;

  std::cout << "Configuring the follower driver..." << std::endl;
  driver_follower.configure(
    trossen_arm::Model::wxai_v0,
    trossen_arm::StandardEndEffector::wxai_v0_follower,
    "192.168.1.3",
    false
  );

  const int num_joints = driver_follower.get_num_joints();

  std::cout << "Moving follower to home position..." << std::endl;
  driver_follower.set_all_modes(trossen_arm::Mode::position);
  {
    std::vector<double> home {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0, 0.0};
    home.resize(num_joints, 0.0);
    driver_follower.set_all_positions(home, 2.0f, true);
  }

  // Ensure we're in position mode for teleoperation
  driver_follower.set_all_modes(trossen_arm::Mode::position);

  // --- Synchronization flags -------------------------------------------------
  std::atomic<bool> leader_ready{false};
  std::atomic<bool> received_first_state{false};

  // --- Connection monitoring -------------------------------------------------
  std::atomic<std::chrono::steady_clock::time_point> last_state_time{std::chrono::steady_clock::now()};
  const auto connection_timeout = 100ms;  // Consider connection lost after 100ms

  // --- ROS 2 comms: leader states in, follower efforts out ------------------
  // Shared latest leader positions/velocities
  std::vector<double> q_shared(num_joints, 0.0);
  std::vector<double> qd_shared(num_joints, 0.0);
  std::mutex q_mtx;

  // Subscriber: leader ready signal
  auto sub_ready = node->create_subscription<std_msgs::msg::Bool>(
    "/leader/ready", qos,
    [&](std_msgs::msg::Bool::SharedPtr msg){
      if (msg->data) {
        leader_ready.store(true);
        std::cout << "Received leader ready signal!" << std::endl;
      }
    });

  // Subscriber: leader joint states
  auto sub_js = node->create_subscription<sensor_msgs::msg::JointState>(
    "/leader/joint_states", qos,
    [&](sensor_msgs::msg::JointState::SharedPtr msg){
      std::lock_guard<std::mutex> lk(q_mtx);
      const size_t mp = std::min(static_cast<size_t>(num_joints), msg->position.size());
      const size_t mv = std::min(static_cast<size_t>(num_joints), msg->velocity.size());
      std::copy_n(msg->position.begin(), mp, q_shared.begin());
      if (mp < static_cast<size_t>(num_joints))
        std::fill(q_shared.begin() + mp, q_shared.end(), 0.0);
      std::copy_n(msg->velocity.begin(), mv, qd_shared.begin());
      if (mv < static_cast<size_t>(num_joints))
        std::fill(qd_shared.begin() + mv, qd_shared.end(), 0.0);
      received_first_state.store(true);
      last_state_time.store(std::chrono::steady_clock::now());  // Update timestamp
    });

  // Publisher: follower external efforts
  auto pub_tau = node->create_publisher<sensor_msgs::msg::JointState>(
    "/follower/external_efforts", qos);

  // Publisher: follower ready signal
  auto pub_ready = node->create_publisher<std_msgs::msg::Bool>("/follower/ready", qos);

  // Preallocated outgoing message
  sensor_msgs::msg::JointState eff;
  eff.name.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) eff.name[i] = "J" + std::to_string(i+1);
  eff.effort.resize(num_joints);

  // Spin ROS callbacks on a separate thread
  std::thread spin_thr([&]{ rclcpp::spin(node); });

  // --- Wait for leader to be ready -------------------------------------------
  std::cout << "Follower is ready. Waiting for leader..." << std::endl;
  
  // Publish our ready signal periodically
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  
  while (rclcpp::ok() && !leader_ready.load()) {
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
  
  // Wait for first joint state message from leader
  auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !received_first_state.load()) {
    // Keep publishing effort data to help leader
    auto tau = driver_follower.get_all_external_efforts();
    const size_t m = std::min(static_cast<size_t>(num_joints), tau.size());
    std::copy_n(tau.begin(), m, eff.effort.begin());
    if (m < static_cast<size_t>(num_joints))
      std::fill(eff.effort.begin() + m, eff.effort.end(), 0.0);
    eff.header.stamp = node->now();
    pub_tau->publish(eff);
    
    std::this_thread::sleep_for(10ms);
    
    // Timeout after 5 seconds
    if (std::chrono::steady_clock::now() - wait_start > 5s) {
      std::cout << "Warning: No joint state data received from leader after 5s. Proceeding anyway..." << std::endl;
      break;
    }
  }

  std::cout << "Data exchange established. Starting teleoperation in 1 second..." << std::endl;
  std::this_thread::sleep_for(1s);

  // Reset connection monitoring timer
  last_state_time.store(std::chrono::steady_clock::now());

  // --- Teleoperation loop ----------------------------------------------------
  const auto teleoperation_time = 20s;

  std::cout << "Tracking leader for "
            << std::chrono::duration_cast<std::chrono::seconds>(teleoperation_time).count()
            << " seconds..." << std::endl;
  std::cout << "Connection monitoring active (timeout: " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(connection_timeout).count() 
            << "ms)" << std::endl;

  auto start_time = std::chrono::steady_clock::now();
  auto end_time   = start_time + teleoperation_time;

  std::vector<double> q_local(num_joints, 0.0);
  std::vector<double> qd_local(num_joints, 0.0);

  bool connection_lost = false;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
    // Check connection health
    auto time_since_last_state = std::chrono::steady_clock::now() - last_state_time.load();
    if (time_since_last_state > connection_timeout) {
      std::cerr << "\n!!! CONNECTION LOST TO LEADER !!!" << std::endl;
      std::cerr << "Last joint state message received " 
                << std::chrono::duration_cast<std::chrono::milliseconds>(time_since_last_state).count() 
                << "ms ago" << std::endl;
      std::cerr << "Initiating emergency stop procedure..." << std::endl;
      connection_lost = true;
      break;
    }

    // Snapshot latest leader states
    {
      std::lock_guard<std::mutex> lk(q_mtx);
      q_local  = q_shared;
      qd_local = qd_shared;
    }

    // Track leader: position mode (non-blocking, with feed-forward velocities)
    driver_follower.set_all_positions(q_local, 0.0f, false, qd_local);

    // Publish follower external efforts for leader haptic feedback
    auto tau = driver_follower.get_all_external_efforts();
    const size_t m = std::min(static_cast<size_t>(num_joints), tau.size());
    std::copy_n(tau.begin(), m, eff.effort.begin());
    if (m < static_cast<size_t>(num_joints))
      std::fill(eff.effort.begin() + m, eff.effort.end(), 0.0);
    eff.header.stamp = node->now();
    pub_tau->publish(eff);

    std::this_thread::sleep_for(1ms);
  }

  // --- Emergency response or normal shutdown ---------------------------------
  if (connection_lost) {
    std::cout << "\n=== EMERGENCY STOP SEQUENCE ===" << std::endl;
    
    // Already in position mode, hold current position to stabilize
    std::cout << "Stabilizing at current position..." << std::endl;
    auto current_pos = driver_follower.get_all_positions();
    driver_follower.set_all_positions(current_pos, 0.5f, true);
    
    std::cout << "Emergency hold complete." << std::endl;
  } else {
    std::cout << "Teleoperation completed normally." << std::endl;
  }

  // --- Return to home & sleep -----------------------------------------------
  std::cout << "Moving follower to home..." << std::endl;
  {
    std::vector<double> home {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0, 0.0};
    home.resize(num_joints, 0.0);
    driver_follower.set_all_positions(home, 2.0f, true);
  }

  std::cout << "Moving follower to sleep position..." << std::endl;
  driver_follower.set_all_positions(std::vector<double>(num_joints, 0.0), 2.0f, true);

  // --- Shutdown --------------------------------------------------------------
  std::cout << "Follower shutdown complete." << std::endl;
  rclcpp::shutdown();
  spin_thr.join();
  return connection_lost ? 2 : 0;  // Return error code if connection was lost
}