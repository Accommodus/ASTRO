

#include "distributed_satellite_sim/env_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvNode>());
  rclcpp::shutdown();
  return 0;
}
