#include "plugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

mocap_plugin::~mocap_plugin()
{
  spinner_on_ = false;
  if(data_thread_.joinable())
  {
    data_thread_.join();
  }
}

void mocap_plugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{

  // config("ip", ip_);
  // config("port", n_port_);
  // config("frequency", freq_);
  // mocap_.data_freq(freq_);
  // mocap_.seq_size(config("sequence_size"));

  mc_rtc::log::info("[mocap plugin] Initialize mocap connection");

  if(controller.controller().config().has("mocap_plugin"))
  {
    configure(controller.controller().config()("mocap_plugin"));
  }
  else
  {
    mc_rtc::log::warning("[mocap plugin] Using default configuration");
    configure(config);
  }

  // spinner_on_ = true;
  // data_thread_ = std::thread(&mocap_plugin::Data_Spinner, this);
  // data_thread_.detach();
  // mc_rtc::log::info("[mocap plugin] data spinner thread created");

  controller.controller().datastore().make<bool>("mocap_plugin::online");
  controller.controller().datastore().make_call(
      "mocap_plugin::get_sequence",
      [this](MoCap_Body_part part, MoCap_Parameters param, int size, int freq) -> Eigen::MatrixXd {
        return mocap_.get_sequence(part, param, size, freq);
      });
  controller.controller().datastore().make_call(
      "mocap_plugin::get_pose", [this](MoCap_Body_part part) -> sva::PTransformd { return mocap_.get_pose(part); });
  controller.controller().datastore().make_call(
      "mocap_plugin::get_velocity", [this](MoCap_Body_part part) -> sva::MotionVecd { return mocap_.get_vel(part); });
  controller.controller().datastore().make_call(
      "mocap_plugin::get_accel",
      [this](MoCap_Body_part part) -> Eigen::Vector3d { return mocap_.get_linear_acc(part); });
  controller.controller().datastore().make_call(
      "mocap_plugin::get_footstate", [this](MoCap_Body_part part) -> int { return mocap_.foot_contact(part); });
  controller.controller().datastore().make_call("mocap_plugin::get_data_frequency", [this]() -> int { return freq_; });
  controller.controller().datastore().make_call("mocap_plugin::get_sequence_size",
                                                [this]() -> int { return mocap_.seq_size(); });

  controller.controller().gui()->addElement({"mocap_plugin"}, mc_rtc::gui::Label("Online", [this]() -> std::string {
                                              if(mocap_online_)
                                              {
                                                return "True";
                                              }
                                              else
                                              {
                                                return "False";
                                              }
                                            }));

  mc_rtc::log::success("[mocap plugin] Initialized");
}

void mocap_plugin::reset(mc_control::MCGlobalController & controller)
{

  spinner_on_ = false;
  if(data_thread_.joinable())
  {
    data_thread_.join();
  }
  spinner_on_ = true;
  
  data_thread_ = std::thread(&mocap_plugin::Data_Spinner, this);
  data_thread_.detach();
  mc_rtc::log::info("mocap_plugin::reset called");
}

void mocap_plugin::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("mocap_plugin::before called");
}

void mocap_plugin::after(mc_control::MCGlobalController & controller)
{

  // std::cout << "here " << std::endl;
  controller.controller().datastore().assign<bool>("mocap_plugin::online", mocap_online_);

  // mc_rtc::log::info(mocap_.foot_contact(LeftFoot));
  // mc_rtc::log::info(mocap_.get_pose(LeftForeArm).translation());
  // mc_rtc::log::info("mocap {}",mocap_online_);
}

mc_control::GlobalPlugin::GlobalPluginConfiguration mocap_plugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = false;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

void mocap_plugin::Data_Spinner()
{

  ClientSocket client_socket_ = ClientSocket(ip_, n_port_);

  while(spinner_on_)
  {

    if(client_socket_.connected())
    {
      std::string data;
    
      try
      {
        client_socket_ >> data;

        size_t data_indx_start = data.find("\n");
        if (data_indx_start != std::string::npos)
        {
          // std::cout << "start at " << data_indx_start << " size " << data.size() << std::endl;
          data = data.substr(data_indx_start + 1,data.size() - data_indx_start);
          // std::cout << "first elements " << data.substr(0,10) << std::endl;
        }
        // std::cout << data.size() << std::endl;
        // if (data.size() < 25000 && data.size() > 5000)
        if (data.size() > 5000)
        {
          mocap_.convert_data(data);
        }
        // else
        // {
        //   std::cout << "bad data " << data.size() << std::endl;
        // }
      }
      catch(SocketException &)
      {
      }
    }
    else
    {
      client_socket_.connect();
    }
    mocap_online_ = client_socket_.connected();

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1e3 / static_cast<double>(freq_))));
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("mocap_plugin", mc_plugin::mocap_plugin)
