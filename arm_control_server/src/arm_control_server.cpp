// Pure C++ server - NO ROS dependencies
// This talks directly to libtrossen_arm

#include <iostream>
#include <thread>
#include <atomic>
#include <cstring>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "libtrossen_arm/trossen_arm.hpp"
#include "arm_protocol.hpp"

class ArmControlServer {
public:
  ArmControlServer(int port, const std::string& arm_ip, bool is_leader)
    : port_(port), arm_ip_(arm_ip), is_leader_(is_leader), running_(false)
  {
  }

  bool initialize() {
    std::cout << "[Server] Initializing arm at " << arm_ip_ 
              << (is_leader_ ? " (LEADER)" : " (FOLLOWER)") << std::endl;

    auto effector = is_leader_ 
      ? trossen_arm::StandardEndEffector::wxai_v0_leader
      : trossen_arm::StandardEndEffector::wxai_v0_follower;

    try {
      driver_.configure(
        trossen_arm::Model::wxai_v0,
        effector,
        arm_ip_,
        false
      );
      std::cout << "[Server] Arm configured successfully" << std::endl;
      
      // Move to home on startup
      move_home();
      
      return true;
    } catch (const std::exception& e) {
      std::cerr << "[Server] Failed to initialize arm: " << e.what() << std::endl;
      return false;
    }
  }

  void start() {
    running_ = true;

    // Create socket
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
      std::cerr << "[Server] Failed to create socket" << std::endl;
      return;
    }

    // Allow port reuse
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      std::cerr << "[Server] Failed to bind to port " << port_ << std::endl;
      close(server_sock);
      return;
    }

    // Listen
    if (listen(server_sock, 5) < 0) {
      std::cerr << "[Server] Failed to listen" << std::endl;
      close(server_sock);
      return;
    }

    std::cout << "[Server] Listening on port " << port_ << std::endl;

    // Accept connections
    while (running_) {
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      
      int client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &client_len);
      if (client_sock < 0) {
        if (running_) {
          std::cerr << "[Server] Failed to accept connection" << std::endl;
        }
        continue;
      }

      std::cout << "[Server] Client connected" << std::endl;

      // Handle client in new thread
      std::thread([this, client_sock]() {
        this->handle_client(client_sock);
      }).detach();
    }

    close(server_sock);
  }

  void stop() {
    running_ = false;
    std::cout << "[Server] Stopping..." << std::endl;
  }

private:
  int port_;
  std::string arm_ip_;
  bool is_leader_;
  std::atomic<bool> running_;
  trossen_arm::TrossenArmDriver driver_;

  void handle_client(int client_sock) {
    std::string buffer;
    char recv_buffer[4096];

    while (running_) {
      ssize_t received = recv(client_sock, recv_buffer, sizeof(recv_buffer) - 1, 0);
      
      if (received <= 0) {
        std::cout << "[Server] Client disconnected" << std::endl;
        break;
      }

      recv_buffer[received] = '\0';
      buffer += recv_buffer;

      // Process complete messages (delimited by newline)
      size_t newline_pos;
      while ((newline_pos = buffer.find('\n')) != std::string::npos) {
        std::string message = buffer.substr(0, newline_pos + 1);
        buffer.erase(0, newline_pos + 1);

        // Process message
        auto response = process_message(message);
        
        // Send response
        std::string response_str = response.serialize();
        send(client_sock, response_str.c_str(), response_str.length(), 0);
      }
    }

    close(client_sock);
  }

  arm_protocol::Response process_message(const std::string& msg_str) {
    arm_protocol::Response response;
    
    auto msg = arm_protocol::Message::deserialize(msg_str);
    if (!msg.success) {
      response.success = false;
      response.error = "Invalid message format";
      return response;
    }

    try {
      switch (msg.command) {
        case arm_protocol::Command::GET_POSITIONS:
          response.data = driver_.get_all_positions();
          response.success = true;
          break;

        case arm_protocol::Command::GET_VELOCITIES:
          response.data = driver_.get_all_velocities();
          response.success = true;
          break;

        case arm_protocol::Command::GET_EFFORTS:
          response.data = driver_.get_all_external_efforts();
          response.success = true;
          break;

        case arm_protocol::Command::SET_MODE_POSITION:
          driver_.set_all_modes(trossen_arm::Mode::position);
          response.success = true;
          break;

        case arm_protocol::Command::SET_MODE_EFFORT:
          driver_.set_all_modes(trossen_arm::Mode::external_effort);
          response.success = true;
          break;

        case arm_protocol::Command::SET_POSITIONS:
          driver_.set_all_positions(msg.data, 0.0f, false);
          response.success = true;
          break;

        case arm_protocol::Command::SET_EFFORTS:
          driver_.set_all_external_efforts(msg.data, 0.0f, false);
          response.success = true;
          break;

        case arm_protocol::Command::MOVE_HOME:
          move_home();
          response.success = true;
          break;

        case arm_protocol::Command::MOVE_SLEEP:
          move_sleep();
          response.success = true;
          break;

        case arm_protocol::Command::SHUTDOWN:
          response.success = true;
          running_ = false;
          break;

        default:
          response.success = false;
          response.error = "Unknown command";
      }
    } catch (const std::exception& e) {
      response.success = false;
      response.error = std::string("Error: ") + e.what();
    }

    return response;
  }

  void move_home() {
    driver_.set_all_modes(trossen_arm::Mode::position);
    std::vector<double> home = {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0, 0.0};
    driver_.set_all_positions(home, 2.0f, true);
    std::cout << "[Server] Moved to home" << std::endl;
  }

  void move_sleep() {
    driver_.set_all_modes(trossen_arm::Mode::position);
    std::vector<double> sleep(driver_.get_num_joints(), 0.0);
    driver_.set_all_positions(sleep, 2.0f, true);
    std::cout << "[Server] Moved to sleep" << std::endl;
  }
};

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <port> <arm_ip> <leader|follower>" << std::endl;
    std::cerr << "Example: " << argv[0] << " 5001 192.168.1.2 leader" << std::endl;
    return 1;
  }

  int port = std::atoi(argv[1]);
  std::string arm_ip = argv[2];
  bool is_leader = (std::string(argv[3]) == "leader");

  std::cout << "=== Arm Control Server ===" << std::endl;
  std::cout << "Port: " << port << std::endl;
  std::cout << "Arm IP: " << arm_ip << std::endl;
  std::cout << "Role: " << (is_leader ? "LEADER" : "FOLLOWER") << std::endl;

  ArmControlServer server(port, arm_ip, is_leader);
  
  if (!server.initialize()) {
    std::cerr << "Failed to initialize server" << std::endl;
    return 1;
  }

  // Start server
  server.start();

  return 0;
}