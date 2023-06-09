// Copyright 2021 TI/Zhang/ETHZ-ASL (?)

#include <stdio.h>

#include <cstdlib>
#include <fstream>
#include <regex>  // NOLINT => linter is not happy with regex (should use re2)

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include "ti_mmwave_ros2/ParameterSetter.h"
#include "ti_mmwave_ros2/srv/mm_wave_cli.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("mmWaveQuickConfig");
  auto req = std::make_shared<ti_mmwave_ros2::srv::MMWaveCLI::Request>();
  ti_mmwave_ros2::srv::MMWaveCLI::Response::SharedPtr resp;

  if (argc < 2)
  {
    RCLCPP_INFO(node->get_logger(), "mmWaveQuickConfig: usage: mmWaveQuickConfig /file_directory/params.cfg");
    return 1;
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "mmWaveQuickConfig: Configuring mmWave device using config file: " << argv[1]);
  }
  // ros::ServiceClient client = nh.serviceClient<ti_mmwave_ros2::srv::MMWaveCLI>("/mmWaveCLI");
  auto client = node->create_client<ti_mmwave_ros2::srv::MMWaveCLI>("/mmWaveCLI");
  std::ifstream myParams;
  ti_mmwave_ros2::ParameterSetter parser;
  // wait for service to become available
  // ros::service::waitForService("/mmWaveCLI", 10000);
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "Waiting for /mmWaveCLI service to be available...");
  }

  // wait 0.5 secs to avoid multi-sensor conflicts
  // ros::Duration(0.5).sleep();
  RCLCPP_INFO(node->get_logger(), "mmWaveQuickConfig: Waiting 1 second to avoid multi-sensor conflicts");
  rclcpp::sleep_for(1s);

  myParams.open(argv[1]);

  RCLCPP_INFO(node->get_logger(), "Opening file %s...", argv[1]);
  if (myParams.is_open())
  {
    while (std::getline(myParams, req->comm))
    {
      req->comm.erase(std::remove(req->comm.begin(), req->comm.end(), '\r'),
                             req->comm.end());
      // Ignore comment lines (first non-space char is '%') or blank lines
      if (!(std::regex_match(req->comm, std::regex("^\\s*%.*")) ||
            std::regex_match(req->comm, std::regex("^\\s*"))))
      {
        RCLCPP_INFO_STREAM(node->get_logger(), "mmWaveQuickConfig: Sending command: " << req->comm);
        auto result = client->async_send_request(req);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          resp = result.get();
          if (std::regex_search(resp->resp, std::regex("Done")))
          {
            RCLCPP_INFO(node->get_logger(), "mmWaveQuickConfig: Command successful (mmWave sensor responded with 'Done')");
            parser.ParamsParser(req, node);
          }
          else
          {
            RCLCPP_ERROR(node->get_logger(), "mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
            RCLCPP_ERROR_STREAM(node->get_logger(), "mmWaveQuickConfig: Response: " << resp->resp);
            return 1;
          }
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "mmWaveQuickConfig: Failed to call service mmWaveCLI");
          RCLCPP_ERROR_STREAM(node->get_logger(), "mmWaveQuickConfig: " << req->comm);
          return 1;
        }
      }
    }
    parser.CalParams(node);

    auto param_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    for(auto param: node->list_parameters({}, 1).names) {
      auto param_value = node->get_parameter(param).to_parameter_msg();
      param_req->parameters.push_back(param_value);

      RCLCPP_INFO_STREAM(node->get_logger(), "mmWaveQuickConfig: Setting parameter " << param_value.name );
    }

    auto set_param_client = node->create_client<rcl_interfaces::srv::SetParameters>("ti_data_handler/set_parameters");

    do {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(node->get_logger(), "Waiting for %s service to be available...", set_param_client->get_service_name());
    } while (!set_param_client->wait_for_service(1s));

    auto set_param_result = set_param_client->async_send_request(param_req);

    if (rclcpp::spin_until_future_complete(node, set_param_result) ==
          rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "mmWaveQuickConfig: Parameters set successfully");
    } else {
      RCLCPP_ERROR(node->get_logger(), "mmWaveQuickConfig: Failed to set parameters");
      return 1;
    }

    myParams.close();
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "mmWaveQuickConfig: Failed to open File " << argv[1]);
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "mmWaveQuickConfig: mmWaveQuickConfig will now terminate.");
  RCLCPP_INFO_STREAM(node->get_logger(), "mmWaveQuickConfig: Done configuring mmWave device using config file: " << argv[1]);

  return 0;
}
