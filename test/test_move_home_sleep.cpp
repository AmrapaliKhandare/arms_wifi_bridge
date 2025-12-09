
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "tcp_client.hpp"

int main(int argc, char** argv)
{
  // Default: local server on port 5001 (change if needed)
  std::string host = "127.0.0.1";
  int port = 5001;

  if (argc >= 2) {
    host = argv[1];
  }
  if (argc >= 3) {
    port = std::stoi(argv[2]);
  }

  std::cout << "=== Test: MOVE_HOME then MOVE_SLEEP on leader arm ===\n";
  std::cout << "Connecting to TCP server at " << host << ":" << port << "...\n";

  try {
    TCPClient client(host, port);

    
    if (!client.connect()) {
      std::cerr << "Failed to connect to server\n";
      return 1;
    }


    // Optional: check that we can talk to the server and read positions
    auto q0 = client.get_positions();
    std::cout << "Current joint positions (" << q0.size() << " joints):\n  ";
    for (double v : q0) {
      std::cout << v << "  ";
    }
    std::cout << "\n";

    std::cout << "\nPress ENTER to command HOME pose...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Sending MOVE_HOME...\n";
    client.move_home();
    std::cout << "MOVE_HOME command sent. Waiting 3 seconds for motion...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto q_home = client.get_positions();
    std::cout << "Joint positions after HOME:\n  ";
    for (double v : q_home) {
      std::cout << v << "  ";
    }
    std::cout << "\n";

    std::cout << "\nPress ENTER to command SLEEP pose...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Sending MOVE_SLEEP...\n";
    client.move_sleep();
    std::cout << "MOVE_SLEEP command sent. Waiting 3 seconds for motion...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto q_sleep = client.get_positions();
    std::cout << "Joint positions after SLEEP:\n  ";
    for (double v : q_sleep) {
      std::cout << v << "  ";
    }
    std::cout << "\n";

    std::cout << "=== Test complete ===\n";
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "Fatal error in test: " << e.what() << std::endl;
    return 1;
  }
}
