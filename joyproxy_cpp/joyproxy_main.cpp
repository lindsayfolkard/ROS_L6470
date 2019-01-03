// I was following the composition package examples, however they seemed to be a
// bit too confusing for me, so I just went with manual composition as it seemed
// to be the easiest option

#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "joyproxy_node.hpp"

using namespace joyproxy;

int main(int argc, char * argv[])
{
  std::cout << "Run JoyProxy Node main!" << std::endl;
  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Add some nodes to the executor which provide work for the executor during its "spin" function.
  // An example of available work is executing a subscription callback, or a timer callback.
  auto node = std::make_shared<JoyProxy>();

  rclcpp::spin(node);
  
  rclcpp::shutdown();

  return 0;
}
