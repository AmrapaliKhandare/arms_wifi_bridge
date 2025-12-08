#include <iostream>
#include "libtrossen_arm/trossen_arm.hpp"

int main(int argc, char ** argv)
{
  std::cout << "plain_libtrossen_test: starting\n";

  // Minimal usage: construct and immediately destroy
  trossen_arm::TrossenArmDriver driver;

  std::cout << "plain_libtrossen_test: finished normally\n";
  return 0;
}
