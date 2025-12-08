#ifndef ARM_PROTOCOL_HPP_
#define ARM_PROTOCOL_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace arm_protocol {

// Command types
enum class Command {
  GET_POSITIONS,
  GET_VELOCITIES,
  GET_EFFORTS,
  SET_MODE_POSITION,
  SET_MODE_EFFORT,
  SET_POSITIONS,
  SET_EFFORTS,
  MOVE_HOME,
  MOVE_SLEEP,
  SHUTDOWN
};

// Simple JSON-like message format (we'll keep it simple, no external JSON lib needed)
class Message {
public:
  Command command;
  std::vector<double> data;
  std::string error;
  bool success;

  // Serialize to string
  std::string serialize() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    
    ss << "{\"cmd\":\"" << command_to_string(command) << "\"";
    
    if (!data.empty()) {
      ss << ",\"data\":[";
      for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) ss << ",";
        ss << data[i];
      }
      ss << "]";
    }
    
    ss << "}\n";  // Newline as message delimiter
    return ss.str();
  }

  // Parse from string
  static Message deserialize(const std::string& str) {
    Message msg;
    msg.success = false;
    
    // Very simple parsing (for production, use a proper JSON library)
    size_t cmd_pos = str.find("\"cmd\":\"");
    if (cmd_pos == std::string::npos) {
      msg.error = "Invalid message format";
      return msg;
    }
    
    size_t cmd_start = cmd_pos + 7;
    size_t cmd_end = str.find("\"", cmd_start);
    std::string cmd_str = str.substr(cmd_start, cmd_end - cmd_start);
    
    msg.command = string_to_command(cmd_str);
    
    // Parse data array if present
    size_t data_pos = str.find("\"data\":[");
    if (data_pos != std::string::npos) {
      size_t data_start = data_pos + 8;
      size_t data_end = str.find("]", data_start);
      std::string data_str = str.substr(data_start, data_end - data_start);
      
      std::stringstream ss(data_str);
      double val;
      char comma;
      while (ss >> val) {
        msg.data.push_back(val);
        ss >> comma;  // consume comma
      }
    }
    
    msg.success = true;
    return msg;
  }

private:
  static std::string command_to_string(Command cmd) {
    switch (cmd) {
      case Command::GET_POSITIONS: return "GET_POSITIONS";
      case Command::GET_VELOCITIES: return "GET_VELOCITIES";
      case Command::GET_EFFORTS: return "GET_EFFORTS";
      case Command::SET_MODE_POSITION: return "SET_MODE_POSITION";
      case Command::SET_MODE_EFFORT: return "SET_MODE_EFFORT";
      case Command::SET_POSITIONS: return "SET_POSITIONS";
      case Command::SET_EFFORTS: return "SET_EFFORTS";
      case Command::MOVE_HOME: return "MOVE_HOME";
      case Command::MOVE_SLEEP: return "MOVE_SLEEP";
      case Command::SHUTDOWN: return "SHUTDOWN";
      default: return "UNKNOWN";
    }
  }

  static Command string_to_command(const std::string& str) {
    if (str == "GET_POSITIONS") return Command::GET_POSITIONS;
    if (str == "GET_VELOCITIES") return Command::GET_VELOCITIES;
    if (str == "GET_EFFORTS") return Command::GET_EFFORTS;
    if (str == "SET_MODE_POSITION") return Command::SET_MODE_POSITION;
    if (str == "SET_MODE_EFFORT") return Command::SET_MODE_EFFORT;
    if (str == "SET_POSITIONS") return Command::SET_POSITIONS;
    if (str == "SET_EFFORTS") return Command::SET_EFFORTS;
    if (str == "MOVE_HOME") return Command::MOVE_HOME;
    if (str == "MOVE_SLEEP") return Command::MOVE_SLEEP;
    if (str == "SHUTDOWN") return Command::SHUTDOWN;
    return Command::GET_POSITIONS;  // Default
  }
};

// Response message
class Response {
public:
  bool success;
  std::vector<double> data;
  std::string error;

  std::string serialize() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    
    ss << "{\"success\":" << (success ? "true" : "false");
    
    if (!error.empty()) {
      ss << ",\"error\":\"" << error << "\"";
    }
    
    if (!data.empty()) {
      ss << ",\"data\":[";
      for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) ss << ",";
        ss << data[i];
      }
      ss << "]";
    }
    
    ss << "}\n";
    return ss.str();
  }

  static Response deserialize(const std::string& str) {
    Response resp;
    
    // Parse success
    size_t success_pos = str.find("\"success\":");
    if (success_pos != std::string::npos) {
      resp.success = str.find("true", success_pos) != std::string::npos;
    } else {
      resp.success = false;
    }
    
    // Parse error
    size_t error_pos = str.find("\"error\":\"");
    if (error_pos != std::string::npos) {
      size_t error_start = error_pos + 9;
      size_t error_end = str.find("\"", error_start);
      resp.error = str.substr(error_start, error_end - error_start);
    }
    
    // Parse data
    size_t data_pos = str.find("\"data\":[");
    if (data_pos != std::string::npos) {
      size_t data_start = data_pos + 8;
      size_t data_end = str.find("]", data_start);
      std::string data_str = str.substr(data_start, data_end - data_start);
      
      std::stringstream ss(data_str);
      double val;
      char comma;
      while (ss >> val) {
        resp.data.push_back(val);
        ss >> comma;
      }
    }
    
    return resp;
  }
};

}  // namespace arm_protocol

#endif  // ARM_PROTOCOL_HPP_