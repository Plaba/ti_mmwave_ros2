// Copyright 2021 TI/Zhang/ETHZ-ASL

#include <string>
#include <vector>

#include "ti_mmwave_ros2/ParameterSetter.h"

namespace ti_mmwave_ros2
{

ParameterSetter::ParameterSetter()
{
}

void ParameterSetter::onInit()
{
}

void ParameterSetter::ParamsParser(ti_mmwave_ros2::srv::MMWaveCLI::Request::SharedPtr request,
 rclcpp::Node::SharedPtr nh)
{
  //   RCLCPP_ERROR(get_logger(), "%s",req.comm.c_str());
  //   RCLCPP_ERROR(get_logger(), "%s",res.resp.c_str());
  std::vector<std::string> v;
  std::string s = request->comm.c_str();
  std::istringstream ss(s);
  std::string token;
  std::string req;
  int i = 0;
  while (std::getline(ss, token, ' '))
  {
    v.push_back(token);
    if (i > 0)
    {
      if (!req.compare("profileCfg"))
      {
        switch (i)
        {
          case 2:
            nh->declare_parameter("/ti_mmwave/startFreq", std::stof(token));
            break;
          case 3:
            nh->declare_parameter("/ti_mmwave/idleTime", std::stof(token));
            break;
          case 4:
            nh->declare_parameter("/ti_mmwave/adcStartTime", std::stof(token));
            break;
          case 5:
            nh->declare_parameter("/ti_mmwave/rampEndTime", std::stof(token));
            break;
          case 8:
            nh->declare_parameter("/ti_mmwave/freqSlopeConst", std::stof(token));
            break;
          case 10:
            nh->declare_parameter("/ti_mmwave/numAdcSamples", std::stoi(token));
            break;
          case 11:
            nh->declare_parameter("/ti_mmwave/digOutSampleRate", std::stof(token));
            break;
          case 14:
            nh->declare_parameter("/ti_mmwave/rxGain", std::stof(token));
            break;
        }
      }
      else if (!req.compare("frameCfg"))
      {
        switch (i)
        {
          case 1:
            nh->declare_parameter("/ti_mmwave/chirpStartIdx", std::stoi(token));
            break;
          case 2:
            nh->declare_parameter("/ti_mmwave/chirpEndIdx", std::stoi(token));
            break;
          case 3:
            nh->declare_parameter("/ti_mmwave/numLoops", std::stoi(token));
            break;
          case 4:
            nh->declare_parameter("/ti_mmwave/numFrames", std::stoi(token));
            break;
          case 5:
            nh->declare_parameter("/ti_mmwave/framePeriodicity", std::stof(token));
            break;
        }
      }
    }
    else
      req = token;
    i++;
  }
}

void ParameterSetter::CalParams(rclcpp::Node::SharedPtr nh)
{
  float c0 = 299792458;
  int chirpStartIdx;
  int chirpEndIdx;
  int numLoops;
  float framePeriodicity;
  float startFreq;
  float idleTime;
  float adcStartTime;
  float rampEndTime;
  float digOutSampleRate;
  float freqSlopeConst;
  int numAdcSamples;

  nh->get_parameter("/ti_mmwave/startFreq", startFreq);
  nh->get_parameter("/ti_mmwave/idleTime", idleTime);
  nh->get_parameter("/ti_mmwave/adcStartTime", adcStartTime);
  nh->get_parameter("/ti_mmwave/rampEndTime", rampEndTime);
  nh->get_parameter("/ti_mmwave/digOutSampleRate", digOutSampleRate);
  nh->get_parameter("/ti_mmwave/freqSlopeConst", freqSlopeConst);
  nh->get_parameter("/ti_mmwave/numAdcSamples", numAdcSamples);

  nh->get_parameter("/ti_mmwave/chirpStartIdx", chirpStartIdx);
  nh->get_parameter("/ti_mmwave/chirpEndIdx", chirpEndIdx);
  nh->get_parameter("/ti_mmwave/numLoops", numLoops);
  nh->get_parameter("/ti_mmwave/framePeriodicity", framePeriodicity);  

  int ntx = chirpEndIdx - chirpStartIdx + 1;
  int nd = numLoops;
  int nr = numAdcSamples;
  float tfr = framePeriodicity * 1e-3;
  float fs = digOutSampleRate * 1e3;
  float kf = freqSlopeConst * 1e12;
  float adc_duration = nr / fs;
  float BW = adc_duration * kf;
  float PRI = (idleTime + rampEndTime) * 1e-6;
  float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2);

  float vrange = c0 / (2 * BW);
  float max_range = nr * vrange;
  float max_vel = c0 / (2 * fc * PRI) / ntx;
  float vvel = max_vel / nd;

  nh->declare_parameter("/ti_mmwave/num_TX", ntx);
  nh->declare_parameter("/ti_mmwave/f_s", fs);
  nh->declare_parameter("/ti_mmwave/f_c", fc);
  nh->declare_parameter("/ti_mmwave/BW", BW);
  nh->declare_parameter("/ti_mmwave/PRI", PRI);
  nh->declare_parameter("/ti_mmwave/t_fr", tfr);
  nh->declare_parameter("/ti_mmwave/max_range", max_range);
  nh->declare_parameter("/ti_mmwave/range_resolution", vrange);
  nh->declare_parameter("/ti_mmwave/max_doppler_vel", max_vel);
  nh->declare_parameter("/ti_mmwave/doppler_vel_resolution", vvel);
}

}  // namespace ti_mmwave_ros2
