// Copyright 2021 TI/Zhang/ETHZ-ASL (?)

#include <pthread.h>

#include <vector>

#include "ti_mmwave_ros2/DataHandlerClass.h"

namespace ti_mmwave_ros2
{

DataUARTHandler::DataUARTHandler(const rclcpp::NodeOptions& options)
: rclcpp::Node("ti_data_handler", options)
, currentBufp(&pingPongBuffers[0])
, nextBufp(&pingPongBuffers[1])
{
  DataUARTHandler_pub = create_publisher<sensor_msgs::msg::PointCloud2>("radar_scan_pcl", 100);
  dataSerialPort = declare_parameter("data_port", dataSerialPort);
  dataBaudRate = declare_parameter("data_rate", dataBaudRate);
  frameID = declare_parameter("frame_id", frameID);

  declare_parameter<int>("max_allowed_elevation_angle_deg", 90);
  declare_parameter<int>("max_allowed_azimuth_angle_deg", 90);

  get_parameter("max_allowed_elevation_angle_deg", maxAllowedElevationAngleDeg);
  get_parameter("max_allowed_azimuth_angle_deg", maxAllowedAzimuthAngleDeg);

  declare_parameter<int>("/ti_mmwave/numAdcSamples",0);
  declare_parameter<int>("/ti_mmwave/numLoops",0);
  declare_parameter<int>("/ti_mmwave/num_TX",0);
  declare_parameter<double>("/ti_mmwave/f_s",0.0);
  declare_parameter<double>("/ti_mmwave/f_c",0.0);
  declare_parameter<double>("/ti_mmwave/BW",0.0);
  declare_parameter<double>("/ti_mmwave/PRI",0.0);
  declare_parameter<double>("/ti_mmwave/t_fr",0.0);
  declare_parameter<double>("/ti_mmwave/max_range",0.0);
  declare_parameter<double>("/ti_mmwave/range_resolution",0.0);
  declare_parameter<double>("/ti_mmwave/max_doppler_vel",0.0);
  declare_parameter<double>("/ti_mmwave/doppler_vel_resolution",0.0);
  timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DataUARTHandler::start, this));
  param_cb_handle_ = add_on_set_parameters_callback(std::bind(&DataUARTHandler::paramsCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult DataUARTHandler::paramsCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    RCLCPP_INFO(get_logger(), "recieved parameter: %s", parameters[0].get_name().c_str());
    result.successful = true;
    result.reason = "success";
    return result;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{
  int firstPacketReady = 0;
  uint8_t last8Bytes[8] = { 0 };

  /*Open UART Port and error checking*/
  serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
  mySerialObject.setPort(dataSerialPort);
  try
  {
    mySerialObject.open();
  }
  catch (std::exception &e1)
  {
    RCLCPP_INFO(get_logger(), "DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
    RCLCPP_INFO(get_logger(), "DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
    try
    {
      // Wait 20 seconds and try to open serial port again
      rclcpp::sleep_for(std::chrono::seconds(20));
      mySerialObject.open();
    }
    catch (std::exception &e2)
    {
      RCLCPP_ERROR(get_logger(), "DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
      RCLCPP_ERROR(get_logger(), "DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d",
                dataSerialPort.c_str(), dataBaudRate);
      pthread_exit(NULL);
    }
  }

  if (mySerialObject.isOpen())
  {
    RCLCPP_INFO(get_logger(), "DataUARTHandler Read Thread: Port is open");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "DataUARTHandler Read Thread: Port could not be opened");
  }

  /*Quick magicWord check to synchronize program with data Stream*/
  while (!isMagicWord(last8Bytes))
  {
    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);
  }

  /*Lock nextBufp before entering main loop*/
  pthread_mutex_lock(&nextBufp_mutex);

  while (rclcpp::ok())
  {
    /*Start reading UART data and writing to buffer while also checking for magicWord*/
    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);

    nextBufp->push_back(last8Bytes[7]);  // push byte onto buffer

    /*If a magicWord is found wait for sorting to finish and switch buffers*/
    if (isMagicWord(last8Bytes))
    {
      /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
      pthread_mutex_lock(&countSync_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      /*increment countSync*/
      countSync++;

      /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
      if (firstPacketReady == 0)
      {
        countSync++;
        firstPacketReady = 1;
      }

      /*Signal Swap Thread to run if countSync has reached its max value*/
      if (countSync == COUNT_SYNC_MAX)
      {
        pthread_cond_signal(&countSync_max_cv);
      }

      /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
      pthread_cond_wait(&read_go_cv, &countSync_mutex);

      /*Unlock countSync so that Swap Thread can use it*/
      pthread_mutex_unlock(&countSync_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      nextBufp->clear();
      memset(last8Bytes, 0, sizeof(last8Bytes));
    }
  }
  mySerialObject.close();
  pthread_exit(NULL);
}

int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
  int val = 0, i = 0, j = 0;

  for (i = 0; i < 8; i++)
  {
    if (last8Bytes[i] == magicWord[i])
    {
      j++;
    }
  }

  if (j == 8)
  {
    val = 1;
  }

  return val;
}

void *DataUARTHandler::syncedBufferSwap(void)
{
  while (rclcpp::ok())
  {
    pthread_mutex_lock(&countSync_mutex);

    while (countSync < COUNT_SYNC_MAX)
    {
      pthread_cond_wait(&countSync_max_cv, &countSync_mutex);

      pthread_mutex_lock(&currentBufp_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      std::vector<uint8_t> *tempBufp = currentBufp;

      this->currentBufp = this->nextBufp;

      this->nextBufp = tempBufp;

      pthread_mutex_unlock(&currentBufp_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      countSync = 0;

      pthread_cond_signal(&sort_go_cv);
      pthread_cond_signal(&read_go_cv);
    }
    pthread_mutex_unlock(&countSync_mutex);
  }
  pthread_exit(NULL);
}

void *DataUARTHandler::sortIncomingData(void)
{
  MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
  uint32_t tlvLen = 0;
  uint32_t tlvCount = 0;
  uint32_t headerSize = 8 * 4;  // same for 1843 and 6843
  unsigned int currentDatap = 0;
  SorterState sorterState = READ_HEADER;
  int i = 0;

  boost::shared_ptr<pcl::PointCloud<radar_pcl::PointXYZIVR>> RScan(new pcl::PointCloud<radar_pcl::PointXYZIVR>);
  ti_mmwave_ros2::msg::RadarScan radarscan;

  // wait for first packet to arrive
  pthread_mutex_lock(&countSync_mutex);
  pthread_cond_wait(&sort_go_cv, &countSync_mutex);
  pthread_mutex_unlock(&countSync_mutex);

  pthread_mutex_lock(&currentBufp_mutex);

  while (rclcpp::ok())
  {
    switch (sorterState)
    {
      case READ_HEADER:
        // init variables
        mmwData.numObjOut = 0;

        // make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord
        // since it was already removed)
        if (currentBufp->size() < 12)
        {
          sorterState = SWAP_BUFFERS;
          break;
        }

        // get version (4 bytes)
        memcpy(&mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
        currentDatap += (sizeof(mmwData.header.version));

        // get totalPacketLen (4 bytes)
        memcpy(&mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
        currentDatap += (sizeof(mmwData.header.totalPacketLen));

        // get platform (4 bytes)
        memcpy(&mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
        currentDatap += (sizeof(mmwData.header.platform));

        // fixed header size 8*4 (same for 1843 and 6843 => set above)
        if (currentBufp->size() < headerSize)
        {
          sorterState = SWAP_BUFFERS;
          break;
        }

        // get frameNumber (4 bytes)
        memcpy(&mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
        currentDatap += (sizeof(mmwData.header.frameNumber));

        // get timeCpuCycles (4 bytes)
        memcpy(&mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
        currentDatap += (sizeof(mmwData.header.timeCpuCycles));

        // get numDetectedObj (4 bytes)
        memcpy(&mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
        currentDatap += (sizeof(mmwData.header.numDetectedObj));

        // get numTLVs (4 bytes)
        memcpy(&mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
        currentDatap += (sizeof(mmwData.header.numTLVs));

        // get subFrameNumber (4 bytes)
        memcpy(&mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
        currentDatap += (sizeof(mmwData.header.subFrameNumber));

        // if packet lengths do not match, throw it away
        if (mmwData.header.totalPacketLen == currentBufp->size())
        {
          sorterState = CHECK_TLV_TYPE;
        }
        else
        {
          sorterState = SWAP_BUFFERS;
        }
        break;

      case READ_OBJ_STRUCT:
        // CHECK_TLV_TYPE code has already read tlvType and tlvLen

        i = 0;
        mmwData.numObjOut = mmwData.header.numDetectedObj;

        // Populate sequence
        RScan->header.seq = 0;

        // Get time
        RScan->header.stamp = now().nanoseconds() / 1e3;
        // or from: https://github.com/radar-lab/ti_mmwave_ros2/pull/27/files
        pcl_conversions::toPCL(now(), RScan->header.stamp);

        RScan->header.frame_id = frameID;
        RScan->height = 1;
        RScan->width = mmwData.numObjOut;
        RScan->is_dense = 1;
        RScan->points.resize(RScan->width * RScan->height);

        // Populate pointcloud
        while (i < mmwData.numObjOut)
        {
          // get object x-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut_cartes.x));
          currentDatap += (sizeof(mmwData.objOut_cartes.x));

          // get object y-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut_cartes.y));
          currentDatap += (sizeof(mmwData.objOut_cartes.y));

          // get object z-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut_cartes.z));
          currentDatap += (sizeof(mmwData.objOut_cartes.z));

          // get object velocity (m/s)
          memcpy(&mmwData.objOut_cartes.velocity, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut_cartes.velocity));
          currentDatap += (sizeof(mmwData.objOut_cartes.velocity));

          // Map mmWave sensor coordinates from TI to std ROS coordinate system (FLU)
          // TI seems to have different frames for the sensors :/ => make platform dependent
          if ((mmwData.header.platform & 0xFFFF) == 0x1843)
          {
            // 1843 has LH (!) coordinate frame => assign proper axes
            RScan->points[i].x = mmwData.objOut_cartes.y;
            RScan->points[i].y = mmwData.objOut_cartes.z;
            RScan->points[i].z = -mmwData.objOut_cartes.x;
          }
          else if ((mmwData.header.platform & 0xFFFF) == 0x6843)
          {
            // 6843 has RH coordinate frame => rotate around z by +90 degrees
            RScan->points[i].x = mmwData.objOut_cartes.y;
            RScan->points[i].y = -mmwData.objOut_cartes.x;
            RScan->points[i].z = mmwData.objOut_cartes.z;
          }
          else
          {
            RCLCPP_FATAL(get_logger(), "Device must either be '1843(AOP)' or '6843(AOP)'");
          }

          RScan->points[i].velocity = mmwData.objOut_cartes.velocity;
          RScan->points[i].range =
              sqrt(pow(RScan->points[i].x, 2) + pow(RScan->points[i].y, 2) + pow(RScan->points[i].z, 2));

          // Increase counter
          i++;
        }

        sorterState = CHECK_TLV_TYPE;

        break;

      case READ_SIDE_INFO:
        // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
        if (mmwData.numObjOut > 0)
        {
          for (i = 0; i < mmwData.numObjOut; i++)
          {
            // get snr (unit is 0.1 steps of dB)
            memcpy(&mmwData.sideInfo.snr, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.snr));
            currentDatap += (sizeof(mmwData.sideInfo.snr));

            // get noise (unit is 0.1 steps of dB)
            memcpy(&mmwData.sideInfo.noise, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.noise));
            currentDatap += (sizeof(mmwData.sideInfo.noise));

            // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
            RScan->points[i].intensity = mmwData.sideInfo.snr / 10.0;
          }
        }
        else
        {  // else just skip side info section if we have not already received and parsed detected obj list
          currentDatap += tlvLen;
        }

        sorterState = CHECK_TLV_TYPE;

        break;

      case READ_LOG_MAG_RANGE:
        sorterState = CHECK_TLV_TYPE;
        break;

      case READ_NOISE:
        currentDatap += tlvLen;
        sorterState = CHECK_TLV_TYPE;
        break;

      case READ_AZIMUTH:
        currentDatap += tlvLen;
        sorterState = CHECK_TLV_TYPE;
        break;

      case READ_DOPPLER:
        currentDatap += tlvLen;
        sorterState = CHECK_TLV_TYPE;
        break;

      case READ_STATS:
        currentDatap += tlvLen;
        sorterState = CHECK_TLV_TYPE;
        break;

      case CHECK_TLV_TYPE:
        if (tlvCount++ >= mmwData.header.numTLVs)  // Done parsing all received TLV sections
        {
          // Publish detected object pointcloud
          if (mmwData.numObjOut > 0)
          {
            auto ros_scan_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*RScan, *ros_scan_msg);
            DataUARTHandler_pub->publish(*ros_scan_msg);
          }

          sorterState = SWAP_BUFFERS;
        }
        else  // More TLV sections to parse
        {
          // get tlvType (32 bits) & remove from queue
          memcpy(&tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
          currentDatap += (sizeof(tlvType));

          // get tlvLen (32 bits) & remove from queue
          memcpy(&tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
          currentDatap += (sizeof(tlvLen));

          switch (tlvType)
          {
            case MMWDEMO_OUTPUT_MSG_NULL:
              break;

            case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
              sorterState = READ_OBJ_STRUCT;
              break;

            case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
              sorterState = READ_LOG_MAG_RANGE;
              break;

            case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
              sorterState = READ_NOISE;
              break;

            case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
              sorterState = READ_AZIMUTH;
              break;

            case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
              sorterState = READ_DOPPLER;
              break;

            case MMWDEMO_OUTPUT_MSG_STATS:
              sorterState = READ_STATS;
              break;

            case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
              sorterState = READ_SIDE_INFO;
              break;

            case MMWDEMO_OUTPUT_MSG_MAX:
              sorterState = READ_HEADER;
              break;

            default:
              break;
          }
        }

        break;

      case SWAP_BUFFERS:

        pthread_mutex_lock(&countSync_mutex);
        pthread_mutex_unlock(&currentBufp_mutex);

        countSync++;

        if (countSync == COUNT_SYNC_MAX)
        {
          pthread_cond_signal(&countSync_max_cv);
        }

        pthread_cond_wait(&sort_go_cv, &countSync_mutex);

        pthread_mutex_unlock(&countSync_mutex);
        pthread_mutex_lock(&currentBufp_mutex);

        currentDatap = 0;
        tlvCount = 0;

        sorterState = READ_HEADER;

        break;

      default:
        break;
    }
  }

  pthread_exit(NULL);
}

bool DataUARTHandler::setupParameters(void)
{
  get_parameter("data_port", dataSerialPort);
  get_parameter("data_rate", dataBaudRate);
  get_parameter("frame_id", frameID);
  get_parameter_or("max_allowed_elevation_angle_deg", maxAllowedElevationAngleDeg, 90);
  get_parameter_or("max_allowed_azimuth_angle_deg", maxAllowedAzimuthAngleDeg, 90);

  RCLCPP_INFO_STREAM(get_logger(), "Waiting for parameters...");
  if(get_parameter("/ti_mmwave/doppler_vel_resolution").as_double() == 0.0)
  {
    return false;
  }



  get_parameter("/ti_mmwave/numAdcSamples", nr);
  get_parameter("/ti_mmwave/numLoops", nd);
  get_parameter("/ti_mmwave/num_TX", ntx);
  get_parameter("/ti_mmwave/f_s", fs);
  get_parameter("/ti_mmwave/f_c", fc);
  get_parameter("/ti_mmwave/BW", BW);
  get_parameter("/ti_mmwave/PRI", PRI);
  get_parameter("/ti_mmwave/t_fr", tfr);
  get_parameter("/ti_mmwave/max_range", max_range);
  get_parameter("/ti_mmwave/range_resolution", vrange);
  get_parameter("/ti_mmwave/max_doppler_vel", max_vel);
  get_parameter("/ti_mmwave/doppler_vel_resolution", vvel);

  RCLCPP_INFO_STREAM(get_logger(), "==============================");
  RCLCPP_INFO_STREAM(get_logger(), "List of parameters");
  RCLCPP_INFO_STREAM(get_logger(), "==============================");
  RCLCPP_INFO_STREAM(get_logger(), "data_port: " << dataSerialPort);
  RCLCPP_INFO_STREAM(get_logger(), "data_rate: " << dataBaudRate);
  RCLCPP_INFO_STREAM(get_logger(), "max_allowed_elevation_angle_deg: " <<  maxAllowedElevationAngleDeg);
  RCLCPP_INFO_STREAM(get_logger(), "max_allowed_azimuth_angle_deg: " <<  maxAllowedAzimuthAngleDeg);
  RCLCPP_INFO_STREAM(get_logger(), "myFrameID: " <<  frameID);
  RCLCPP_INFO_STREAM(get_logger(), "Number of range samples: " << nr);
  RCLCPP_INFO_STREAM(get_logger(), "Number of chirps: " << nd);
  RCLCPP_INFO_STREAM(get_logger(), "Number of TX antenna: " << ntx);
  RCLCPP_INFO_STREAM(get_logger(), "f_s: " << fs / 1e6 << " MHz");
  RCLCPP_INFO_STREAM(get_logger(), "Bandwidth: " << BW / 1e6 << " MHz");
  RCLCPP_INFO_STREAM(get_logger(), "PRI: " << PRI * 1e6 << " us");
  RCLCPP_INFO_STREAM(get_logger(), "Frame time: " << tfr * 1e3 << " ms");
  RCLCPP_INFO_STREAM(get_logger(), "Max range: " << max_range << " m");
  RCLCPP_INFO_STREAM(get_logger(), "Range resolution: " << vrange << "m");
  RCLCPP_INFO_STREAM(get_logger(), "Max Doppler: " << max_vel / 2 << "m/s");
  RCLCPP_INFO_STREAM(get_logger(), "Doppler resolution: " << vvel << " m/s");
  RCLCPP_INFO_STREAM(get_logger(), "==============================");

  timer_->cancel();
  timer_ = nullptr;
  return true;
}

void DataUARTHandler::start(void)
{

  if(setupParameters()){
    RCLCPP_INFO_STREAM(get_logger(), "Parameters set up successfully");
  } else {
    return;
  }

  pthread_t uartThread, sorterThread, swapThread;

  int iret1, iret2, iret3;

  pthread_mutex_init(&countSync_mutex, NULL);
  pthread_mutex_init(&nextBufp_mutex, NULL);
  pthread_mutex_init(&currentBufp_mutex, NULL);
  pthread_cond_init(&countSync_max_cv, NULL);
  pthread_cond_init(&read_go_cv, NULL);
  pthread_cond_init(&sort_go_cv, NULL);

  countSync = 0;

  /* Create independent threads each of which will execute function */
  iret1 = pthread_create(&uartThread, NULL, this->readIncomingData_helper, this);
  if (iret1)
  {
    RCLCPP_INFO(get_logger(), "Error - pthread_create() return code: %d\n", iret1);
    rclcpp::shutdown();
  }

  iret2 = pthread_create(&sorterThread, NULL, this->sortIncomingData_helper, this);
  if (iret2)
  {
    RCLCPP_INFO(get_logger(), "Error - pthread_create() return code: %d\n", iret1);
    rclcpp::shutdown();
  }

  iret3 = pthread_create(&swapThread, NULL, this->syncedBufferSwap_helper, this);
  if (iret3)
  {
    RCLCPP_INFO(get_logger(), "Error - pthread_create() return code: %d\n", iret1);
    rclcpp::shutdown();
  }

  pthread_join(iret1, NULL);
  RCLCPP_INFO(get_logger(), "DataUARTHandler Read Thread joined");
  pthread_join(iret2, NULL);
  RCLCPP_INFO(get_logger(), "DataUARTHandler Sort Thread joined");
  pthread_join(iret3, NULL);
  RCLCPP_INFO(get_logger(), "DataUARTHandler Swap Thread joined");

  pthread_mutex_destroy(&countSync_mutex);
  pthread_mutex_destroy(&nextBufp_mutex);
  pthread_mutex_destroy(&currentBufp_mutex);
  pthread_cond_destroy(&countSync_max_cv);
  pthread_cond_destroy(&read_go_cv);
  pthread_cond_destroy(&sort_go_cv);
}

void *DataUARTHandler::readIncomingData_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->readIncomingData());
}

void *DataUARTHandler::sortIncomingData_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->sortIncomingData());
}

void *DataUARTHandler::syncedBufferSwap_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->syncedBufferSwap());
}

} // namespace ti_mmwave_ros2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2::DataUARTHandler)

