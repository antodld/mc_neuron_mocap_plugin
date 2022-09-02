/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "../include/mc_neuron_mocap_plugin/MoCap.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>

namespace mc_plugin
{

struct mocap_plugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  void configure(const mc_rtc::Configuration & config)
  {
    mc_rtc::log::info("mocap_plugin::init called with configuration:\n{}", config.dump(true, true));
    if(config.has("ip"))
    {
      config("ip", ip_);
    }
    if(config.has("port"))
    {
      config("port", n_port_);
    }
    if(config.has("frequency"))
    {
      config("frequency", freq_);
    }
    if(config.has("sequence_size"))
    {
      mocap_.seq_size(config("sequence_size"));
    }
    if(config.has("activate_on_start"))
    {
      config("activate_on_start", active_at_start_);
    }
    spinner_on_ = false;
    if(data_thread_.joinable())
    {
      data_thread_.join();
    }
    if(active_at_start_)
    {
      spinner_on_ = true;
      data_thread_ = std::thread(&mocap_plugin::Data_Spinner, this);
      data_thread_.detach();
    }
  }

  ~mocap_plugin() override;

private:
  MoCap_Data mocap_;
  std::thread data_thread_;

  std::string ip_;
  int n_port_;
  int freq_;

  bool spinner_on_ = true;
  bool mocap_online_ = false;
  bool active_at_start_ = true;

  void Data_Spinner();
  sva::PTransformd get_pose(MoCap_Body_part part);
  sva::MotionVecd get_vel(MoCap_Body_part part);
  Eigen::Vector3d get_accel(MoCap_Body_part part);
};

} // namespace mc_plugin
