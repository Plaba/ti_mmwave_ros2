/*
 * DataHandlerClass.h
 *
 * This file contains various defines used within this package.
 *
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ti_mmwave_ros2_DATAHANDLERCLASS_H
#define ti_mmwave_ros2_DATAHANDLERCLASS_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "ti_mmwave_ros2/msg/radar_scan.hpp"
#include "ti_mmwave_ros2/mmWave.h"
#include "ti_mmwave_ros2/point_types.h"
#define COUNT_SYNC_MAX 2

namespace ti_mmwave_ros2
{

class DataUARTHandler : public rclcpp::Node
{
public:
  /*Constructor*/
  // void DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
  explicit DataUARTHandler(const rclcpp::NodeOptions& options);

  /*User callable function to start the handler's internal threads*/
  void start(void);

  /*Helper functions to allow pthread compatability*/
  static void* readIncomingData_helper(void* context);

  static void* sortIncomingData_helper(void* context);

  static void* syncedBufferSwap_helper(void* context);

  /*Sorted mmwDemo Data structure*/
  mmwDataPacket mmwData;

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  bool setupParameters(void);
  rcl_interfaces::msg::SetParametersResult paramsCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::TimerBase::SharedPtr timer_;

  int nr;
  int nd;
  int ntx;
  float fs;
  float fc;
  float BW;
  float PRI;
  float tfr;
  float max_range;
  float vrange;
  float max_vel;
  float vvel;

  std::string frameID;
  /*Contains the name of the serial port*/
  std::string dataSerialPort;

  /*Contains the baud Rate*/
  int dataBaudRate;

  /*Contains the max_allowed_elevation_angle_deg (points with elevation angles
    outside +/- max_allowed_elevation_angle_deg will be removed)*/
  int maxAllowedElevationAngleDeg;

  /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles
    outside +/- max_allowed_azimuth_angle_deg will be removed)*/
  int maxAllowedAzimuthAngleDeg;

  /*Mutex protected variable which synchronizes threads*/
  int countSync;

  /*Read/Write Buffers*/
  std::vector<uint8_t> pingPongBuffers[2];

  /*Pointer to current data (sort)*/
  std::vector<uint8_t>* currentBufp;

  /*Pointer to new data (read)*/
  std::vector<uint8_t>* nextBufp;

  /*Mutex protecting the countSync variable */
  pthread_mutex_t countSync_mutex;

  /*Mutex protecting the nextBufp pointer*/
  pthread_mutex_t nextBufp_mutex;

  /*Mutex protecting the currentBufp pointer*/
  pthread_mutex_t currentBufp_mutex;

  /*Condition variable which blocks the Swap Thread until signaled*/
  pthread_cond_t countSync_max_cv;

  /*Condition variable which blocks the Read Thread until signaled*/
  pthread_cond_t read_go_cv;

  /*Condition variable which blocks the Sort Thread until signaled*/
  pthread_cond_t sort_go_cv;

  /* Indicator that the read thread is joined */

  pthread_t read_thread_id;
  pthread_t sort_thread_id;
  pthread_t swap_thread_id;

  volatile bool should_shutdown = false;

  void shutdownCallback();

  /*Swap Buffer Pointers Thread*/
  void* syncedBufferSwap(void);

  /*Checks if the magic word was found*/
  int isMagicWord(uint8_t last8Bytes[8]);

  /*Read incoming UART Data Thread*/
  void* readIncomingData(void);

  /*Sort incoming UART Data Thread*/
  void* sortIncomingData(void);

  void visualize(const ti_mmwave_ros2::msg::RadarScan& msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr DataUARTHandler_pub;

  const rclcpp::Context::SharedPtr context_;
};

}  // namespace ti_mmwave_ros2


#endif  // ti_mmwave_ros2_DATAHANDLERCLASS_H
