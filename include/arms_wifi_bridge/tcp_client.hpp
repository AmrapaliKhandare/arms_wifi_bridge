#ifndef TCP_CLIENT_HPP_
#define TCP_CLIENT_HPP_

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "arm_protocol.hpp"

class TCPClient {
public:
  TCPClient(const std::string& host, int port)
    : host_(host), port_(port), sock_(-1), connected_(false)
  {
  }

  ~TCPClient() {
    disconnect();
  }

  bool connect() {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
      return false;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
      close(sock_);
      sock_ = -1;
      return false;
    }

    if (::connect(sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      close(sock_);
      sock_ = -1;
      return false;
    }

    connected_ = true;
    return true;
  }

  void disconnect() {
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
    connected_ = false;
  }

  bool is_connected() const {
    return connected_;
  }

  // Send request and receive response
  arm_protocol::Response send_request(const arm_protocol::Message& msg) {
    if (!connected_) {
      arm_protocol::Response resp;
      resp.success = false;
      resp.error = "Not connected";
      return resp;
    }

    // Serialize and send
    std::string msg_str = msg.serialize();
    ssize_t sent = send(sock_, msg_str.c_str(), msg_str.length(), 0);
    
    if (sent < 0) {
      arm_protocol::Response resp;
      resp.success = false;
      resp.error = "Send failed";
      connected_ = false;
      return resp;
    }

    // Receive response (read until newline)
    std::string response;
    char buffer[4096];
    
    while (true) {
      ssize_t received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
      
      if (received <= 0) {
        arm_protocol::Response resp;
        resp.success = false;
        resp.error = "Connection closed";
        connected_ = false;
        return resp;
      }

      buffer[received] = '\0';
      response += buffer;

      // Check for newline (end of message)
      if (response.find('\n') != std::string::npos) {
        break;
      }
    }

    return arm_protocol::Response::deserialize(response);
  }

  // Convenience methods
  std::vector<double> get_positions() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::GET_POSITIONS;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to get positions: " + resp.error);
    }
    return resp.data;
  }

  std::vector<double> get_velocities() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::GET_VELOCITIES;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to get velocities: " + resp.error);
    }
    return resp.data;
  }

  std::vector<double> get_efforts() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::GET_EFFORTS;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to get efforts: " + resp.error);
    }
    return resp.data;
  }

  void set_mode_position() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::SET_MODE_POSITION;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to set position mode: " + resp.error);
    }
  }

  void set_mode_effort() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::SET_MODE_EFFORT;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to set effort mode: " + resp.error);
    }
  }

  void set_positions(const std::vector<double>& positions) {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::SET_POSITIONS;
    msg.data = positions;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to set positions: " + resp.error);
    }
  }

  void set_efforts(const std::vector<double>& efforts) {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::SET_EFFORTS;
    msg.data = efforts;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to set efforts: " + resp.error);
    }
  }

  void move_home() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::MOVE_HOME;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to move home: " + resp.error);
    }
  }

  void move_sleep() {
    arm_protocol::Message msg;
    msg.command = arm_protocol::Command::MOVE_SLEEP;
    auto resp = send_request(msg);
    if (!resp.success) {
      throw std::runtime_error("Failed to move sleep: " + resp.error);
    }
  }

private:
  std::string host_;
  int port_;
  int sock_;
  bool connected_;
};

#endif  // TCP_CLIENT_HPP_