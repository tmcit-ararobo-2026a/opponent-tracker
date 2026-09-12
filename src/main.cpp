#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "opponent_tracker/opponent_tracker_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<opponent_tracker::OpponentTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

