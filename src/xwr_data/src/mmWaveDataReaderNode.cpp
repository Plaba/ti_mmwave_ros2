#include "xwr_data/mmWaveDataReader.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xwr_data::DataUARTHandler>());
  rclcpp::shutdown();
  return 0;
}