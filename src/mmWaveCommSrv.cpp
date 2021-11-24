#include "ti_mmwave_rospkg/mmWaveCommSrv.h"

namespace ti_mmwave_rospkg
{
PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveCommSrv, nodelet::Nodelet);

mmWaveCommSrv::mmWaveCommSrv()
{
}

void mmWaveCommSrv::onInit()
{
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  ros::NodeHandle private_nh2("~");  // follow namespace for multiple sensors

  private_nh2.getParam("command_port", mySerialPort);
  private_nh2.getParam("command_rate", myBaudRate);

  ROS_INFO_STREAM("mmWaveCommSrv: command_port = " << mySerialPort.c_str());
  ROS_INFO_STREAM("mmWaveCommSrv: command_rate = " << myBaudRate);

  /* Write one newline to get comms to known state, (rather hacky) */
  serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
  mySerialObject.setPort(mySerialPort.c_str());
  mySerialObject.open();
  mySerialObject.write("\n");
  mySerialObject.close();

  commSrv = private_nh.advertiseService("/mmWaveCLI", &mmWaveCommSrv::commSrv_cb, this);

  NODELET_DEBUG("mmWaveCommsrv: Finished onInit function");
}

bool mmWaveCommSrv::commSrv_cb(mmWaveCLI::Request &req, mmWaveCLI::Response &res)
{
  NODELET_DEBUG_STREAM("mmWaveCommSrv: Port is " << mySerialPort.c_str() << " and baud rate is " << myBaudRate);

  /*Open Serial port and error check*/
  serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
  mySerialObject.setPort(mySerialPort.c_str());
  try
  {
    mySerialObject.open();
  }
  catch (std::exception &e1)
  {
    ROS_INFO_STREAM("mmWaveCommSrv: Failed to open User serial port with error: " << e1.what());
    ROS_INFO_STREAM("mmWaveCommSrv: Waiting 20 seconds before trying again...");
    try
    {
      // Wait 20 seconds and try to open serial port again
      ros::Duration(20).sleep();
      mySerialObject.open();
      mySerialObject.write("\n");  // Flush the port
    }
    catch (std::exception &e2)
    {
      ROS_ERROR_STREAM("mmWaveCommSrv: Failed second time to open User serial port, error: " << e2.what());
      NODELET_ERROR_STREAM("mmWaveCommSrv: Port could not be opened. Port is " << mySerialPort.c_str()
                                                                               << " and baud rate is " << myBaudRate);
      return false;
    }
  }

  /*Read any previous pending response(s)*/
  while (mySerialObject.available() > 0)
  {
    mySerialObject.readline(res.resp, 1024, ":/>");
    ROS_INFO_STREAM("mmWaveCommSrv: Received (previous) response from sensor: " << res.resp.c_str());
    res.resp = "";
  }

  /*Send out command received from the client*/
  ROS_INFO_STREAM("mmWaveCommSrv: Sending command to sensor: " << req.comm.c_str());
  req.comm.append("\n");
  size_t bytesSent = mySerialObject.write(req.comm.c_str());
  ROS_INFO_STREAM("mmWaveCommSrv: Sent nb of bytes to sensor: " << bytesSent);

  /*Read output from mmwDemo*/
  mySerialObject.readline(res.resp, 1024, ":/>");
  ROS_INFO_STREAM("mmWaveCommSrv: Received response from sensor: " << res.resp.c_str());

  mySerialObject.close();

  return true;
}
}  // namespace ti_mmwave_rospkg
