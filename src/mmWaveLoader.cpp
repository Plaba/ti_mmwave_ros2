// Copyright 2021 TI/Zhang/ETHZ-ASL (?)

#include "rclcpp/rclcpp.hpp"

#include "ti_mmwave_ros2/DataHandlerClass.h"
#include "ti_mmwave_ros2/mmWaveCommSrv.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_global_arguments(true);

  auto mmWaveCommSrv = std::make_shared<ti_mmwave_ros2::mmWaveCommSrv>(options);
  exec.add_node(mmWaveCommSrv);

  auto mmWaveDataHdl = std::make_shared<ti_mmwave_ros2::DataUARTHandler>(options);

  options.allow_undeclared_parameters(true);
  
  exec.add_node(mmWaveDataHdl);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
