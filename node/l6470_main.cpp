// I was following the composition package examples, however they seemed to be a
// bit too confusing for me, so I just went with manual composition as it seemed
// to be the easiest option

#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "l6470_node.hpp"

int main(int argc, char * argv[])
{
  std::cout << "Run l6470_main for nodee" << std::endl;
  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);
  //rclcpp::WallRate loop_rate(0.1);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;

  // Let's parse the PowerStepConfig
  const std::string configFilePath = "Node.cfg";
  OverallCfg cfg(configFilePath);
  //std::cout

  // Add some nodes to the executor which provide work for the executor during its "spin" function.
  // An example of available work is executing a subscription callback, or a timer callback.
  //auto node = std::make_shared<l6470::L6470Node>(cfg);
  auto node = std::shared_ptr<l6470::L6470Node> (new l6470::L6470Node(cfg));
  exec.add_node(node);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  while(rclcpp::ok())
  {
      std::cout << "Spin some.. please sir" << std::endl;
      //rclcpp::spin_some(node);
      // loop_rate.sleep();
      exec.spin();
      std::cout << "Leave spin ... " << std::endl;
  }
  std::cout << "Leave spin ?? " << std::endl;
  std::cout << std::flush;
  
  rclcpp::shutdown();

  return 0;
}
