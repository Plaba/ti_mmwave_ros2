#include "xwr_config/mmWaveCommSrv.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xwr_config::mmWaveCommSrv>());
  rclcpp::shutdown();
  return 0;
}