#include "experiment_coordinator.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ExperimentCoordinator>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

