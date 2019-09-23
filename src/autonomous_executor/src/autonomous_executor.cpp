#include <rclcpp/rclcpp.hpp>
#include "autonomous/autonomous_component.hpp"
#include "pcam/pcam_component.hpp"
#include "red_detect/red_detect_component.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc,argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto autonomous = std::make_shared<autonomous::Autonomous>();
  exec.add_node(autonomous);
  auto pcam = std::make_shared<pcam::Pcam>();
  exec.add_node(pcam);
  auto red_detect = std::make_shared<red_detect::RedDetect>();
  exec.add_node(red_detect);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}