// Copyright 2021 TI/Zhang/ETHZ-ASL (?)

#include "xwr_config/mmWaveCommSrv.h"

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

/*Include standard C/C++ headers*/
#include <string>

namespace xwr_config
{
mmWaveCommSrv::mmWaveCommSrv()
: rclcpp::Node("mmWaveCommSrv")
{
  onInit();
}

mmWaveCommSrv::mmWaveCommSrv(const rclcpp::NodeOptions& options)
: rclcpp::Node("mmWaveCommSrv", options)
{
  onInit();
}

void mmWaveCommSrv::onInit()
{
  mySerialPort = declare_parameter("command_port", mySerialPort);
  myBaudRate = declare_parameter("command_rate", myBaudRate);

  RCLCPP_INFO_STREAM(get_logger(), "mmWaveCommSrv: command_port = " << mySerialPort.c_str());
  RCLCPP_INFO_STREAM(get_logger(), "mmWaveCommSrv: command_rate = " << myBaudRate);

  /* Write one newline to get comms to known state, (rather hacky) */
  serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
  mySerialObject.setPort(mySerialPort.c_str());
  /*Open UART Port and error checking*/
  try
  {
    mySerialObject.open();
  }
  catch (std::exception &e1)
  {
    RCLCPP_INFO(get_logger(), "Failed to open Data serial port with error: %s", e1.what());
    RCLCPP_INFO(get_logger(), "Waiting 20 seconds before trying again...");
    try
    {
      // Wait 20 seconds and try to open serial port again
      rclcpp::sleep_for(std::chrono::seconds(20));
      mySerialObject.open();
    }
    catch (std::exception &e2)
    {
      RCLCPP_ERROR(get_logger(), "Failed second time to open Data serial port, error: %s", e1.what());
      RCLCPP_ERROR(get_logger(), "Port could not be opened. Port is \"%s\" and baud rate is %d",
                mySerialPort.c_str(), myBaudRate);
      pthread_exit(NULL);
    }
  }

  mySerialObject.write("\n");
  mySerialObject.close();

  commSrv = create_service<mmWaveCLI>("mmWaveCLI", 
    std::bind(&mmWaveCommSrv::commSrv_cb, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  RCLCPP_DEBUG(get_logger(),"mmWaveCommsrv: Finished onInit function");
}

bool mmWaveCommSrv::commSrv_cb(
  const std::shared_ptr<rmw_request_id_t> _request_header,
  std::shared_ptr<mmWaveCLI::Request> req, std::shared_ptr<mmWaveCLI::Response> res
)
{
  RCLCPP_DEBUG_STREAM(get_logger(),"mmWaveCommSrv: Port is " << mySerialPort.c_str() << " and baud rate is " << myBaudRate);

  /*Open Serial port and error check*/
  serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
  mySerialObject.setPort(mySerialPort.c_str());
  try
  {
    mySerialObject.open();
  }
  catch (std::exception &e1)
  {
    RCLCPP_INFO_STREAM(get_logger(), "mmWaveCommSrv: Failed to open User serial port with error: " << e1.what());
    RCLCPP_INFO_STREAM(get_logger(), "mmWaveCommSrv: Waiting 20 seconds before trying again...");
    try
    {
      // Wait 20 seconds and try to open serial port again
      rclcpp::sleep_for(std::chrono::seconds(20));
      mySerialObject.open();
      mySerialObject.write("\n");  // Flush the port
    }
    catch (std::exception &e2)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "mmWaveCommSrv: Failed second time to open User serial port, error: " << e2.what());
      RCLCPP_ERROR_STREAM(get_logger(),"mmWaveCommSrv: Port could not be opened. Port is " << mySerialPort.c_str()
                                                                               << " and baud rate is " << myBaudRate);
      return false;
    }
  }

  /*Read any previous pending response(s)*/
  while (mySerialObject.available() > 0)
  {
    mySerialObject.readline(res->resp, 1024, ":/>");
    RCLCPP_INFO_STREAM(get_logger(), "mmWaveCommSrv: Received (previous) response from sensor: " << res->resp.c_str());
    res->resp = "";
  }

  /*Send out command received from the client*/
  req->comm.append("\n");
  RCLCPP_DEBUG_STREAM(get_logger(), "mmWaveCommSrv: Sending command to sensor: " << req->comm);
  size_t bytesSent = mySerialObject.write(req->comm);
  RCLCPP_DEBUG_STREAM(get_logger(), "mmWaveCommSrv: Sent nb of bytes to sensor: " << bytesSent);

  /*Read output from mmwDemo*/
  mySerialObject.readline(res->resp, 1024, ":/>");
  RCLCPP_DEBUG_STREAM(get_logger(), "mmWaveCommSrv: Received response from sensor: " << res->resp.c_str());

  mySerialObject.close();

  return true;
}
}  // namespace xwr_config

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(xwr_config::mmWaveCommSrv)
