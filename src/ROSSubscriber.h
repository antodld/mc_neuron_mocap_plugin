#pragma once
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <std_msgs_stamped/Float32MultiArrayStamped.h>
#include <thread>

/**
 * @brief Describes data obtained by a subscriber, along with the time since it
 * was obtained
 *
 * @tparam Data Type of the data obtained by the subscriber
 */
template<typename Data>
struct SubscriberData
{
  bool isValid() const noexcept
  {
    return time_ <= maxTime_;
  }

  void operator=(const SubscriberData<Data> & data)
  {
    value_ = data.value_;
    time_ = data.time_;
    maxTime_ = data.maxTime_;
  }

  const Data & value() const noexcept
  {
    return value_;
  }

  void tick(double dt)
  {
    time_ += dt;
  }

  void maxTime(double t)
  {
    maxTime_ = t;
  }

  double time() const noexcept
  {
    return time_;
  }

  double maxTime() const noexcept
  {
    return maxTime_;
  }

  void value(const Data & data)
  {
    value_ = data;
    time_ = 0;
  }

private:
  Data value_;
  double time_ = std::numeric_limits<double>::max();
  double maxTime_ = std::numeric_limits<double>::max();
};

/**
 * @brief Simple interface for subscribing to data. It is assumed here that the
 * data will be acquired in a separate thread (e.g ROS spinner) and
 * setting/getting the data is thread-safe.
 * Don't forget to call tick(double dt) to update the time and value
 */
template<typename Data>
struct Subscriber
{
  /** Update time and value */
  void tick(double dt)
  {
    data_.tick(dt);
  }

  void maxTime(double t)
  {
    data_.maxTime(t);
  }

  const SubscriberData<Data> data() const noexcept
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    return data_;
  }

protected:
  void value(const Data & data)
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    data_.value(data);
  }

  void value(const Data && data)
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    data_.value(data);
  }

private:
  SubscriberData<Data> data_;
  mutable std::mutex valueMutex_;
};

template<typename ROSMessageType, typename TargetType>
struct ROSSubscriber : public Subscriber<TargetType>
{
  template<typename ConverterFun>
  ROSSubscriber(ConverterFun && fun) : converter_(fun)
  {
  }

  void subscribe(ros::NodeHandle & nh, const std::string & topic, const unsigned bufferSize = 1)
  {
    sub_ = nh.subscribe(topic, bufferSize, &ROSSubscriber::callback, this);
  }

  std::string topic() const
  {
    return sub_.getTopic();
  }

  const ros::Subscriber & subscriber() const
  {
    return sub_;
  }

protected:
  void callback(const boost::shared_ptr<ROSMessageType const> & msg)
  {
    this->value(converter_(*msg));
  }

protected:
  ros::Subscriber sub_;
  // message_filters::Subscriber<TargetType(const ROSMessageType &)> synchro_sub_;
  std::function<TargetType(const ROSMessageType &)> converter_;
};

struct ROSPoseStampedSubscriber : public ROSSubscriber<geometry_msgs::PoseStamped, sva::PTransformd>
{
  ROSPoseStampedSubscriber()
  : ROSSubscriber([](const geometry_msgs::PoseStamped & msg) {
      const auto & t = msg.pose.position;
      const auto & r = msg.pose.orientation;
      auto pose = sva::PTransformd(Eigen::Quaterniond{r.w, r.x, r.y, r.z}.inverse(), Eigen::Vector3d{t.x, t.y, t.z});
      return pose;
    })
  {
  }
};

struct ROSAccelStampedSubscriber : public ROSSubscriber<geometry_msgs::AccelStamped, sva::MotionVecd>
{
  ROSAccelStampedSubscriber()
  : ROSSubscriber([](const geometry_msgs::AccelStamped & msg) {
      const auto & a = msg.accel.linear;
      const auto & w = msg.accel.angular;
      auto acc = sva::MotionVecd(Eigen::Vector3d{w.x, w.y, w.z}, Eigen::Vector3d{a.x, a.y, a.z});
      return acc;
    })
  {
  }
};

struct ROSBoolSubscriber : public ROSSubscriber<std_msgs::Bool, bool>
{
  ROSBoolSubscriber() : ROSSubscriber([](const std_msgs::Bool & msg) { return msg.data; }) {}
};

struct ROSMultiArraySubscriber : public ROSSubscriber<std_msgs::Float32MultiArray, std::vector<float>>
{
  ROSMultiArraySubscriber() : ROSSubscriber([](const std_msgs::Float32MultiArray & msg) { return msg.data; }) {}
};

struct ROSMultiArrayStampedSubscriber
: public ROSSubscriber<std_msgs_stamped::Float32MultiArrayStamped, std_msgs_stamped::Float32MultiArrayStamped>
{
  ROSMultiArrayStampedSubscriber()
  : ROSSubscriber([](const std_msgs_stamped::Float32MultiArrayStamped & msg) { return msg; })
  {
  }
};

struct ROSFloatSubscriber : public ROSSubscriber<std_msgs::Float64, double>
{
  ROSFloatSubscriber() : ROSSubscriber([](const std_msgs::Float64 & msg) { return msg.data; }) {}
};

struct ROSIntSubscriber : public ROSSubscriber<std_msgs::Int64, int>
{
  ROSIntSubscriber() : ROSSubscriber([](const std_msgs::Int64 & msg) { return msg.data; }) {}
};

struct ROSStringSubscriber : public ROSSubscriber<std_msgs::String, std::string>
{
  ROSStringSubscriber() : ROSSubscriber([](const std_msgs::String & msg) { return msg.data; }) {}
};
