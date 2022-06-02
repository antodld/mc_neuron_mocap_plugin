#pragma once
#include <mc_control/mc_controller.h>
#include "ROSSubscriber.h"
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <iterator>


enum MoCap_Body_part
{
  Hips,
  RightUpLeg,
  RightLeg,
  RightFoot,
  LeftUpLeg,
  LeftLeg,
  LeftFoot,
  RightShoulder,
  RightArm,
  RightForeArm,
  RightHand,
  LeftShoulder,
  LeftArm,
  LeftForeArm,
  LeftHand,
  Head,
  Neck1,
  Neck,
  Spine2,
  Spine1,
  Spine,
  RightHandThumb1,
  RightHandThumb2,
  RightHandThumb3,
  RightInHandIndex,
  RightHandIndex1,
  RightHandIndex2,
  RightHandIndex3,
  RightInHandMiddle,
  RightHandMiddle1,
  RightHandMiddle2,
  RightHandMiddle3,
  RightInHandRing,
  RightHandRing1,
  RightHandRing2,
  RightHandRing3,
  RightInHandPinky,
  RightHandPinky1,
  RightHandPinky2,
  RightHandPinky3,
  LeftHandThumb1,
  LeftHandThumb2,
  LeftHandThumb3,
  LeftInHandIndex,
  LeftHandIndex1,
  LeftHandIndex2,
  LeftHandIndex3,
  LeftInHandMiddle,
  LeftHandMiddle1,
  LeftHandMiddle2,
  LeftHandMiddle3,
  LeftInHandRing,
  LeftHandRing1,
  LeftHandRing2,
  LeftHandRing3,
  LeftInHandPinky,
  LeftHandPinky1,
  LeftHandPinky2,
  LeftHandPinky3
};

enum MoCap_Parameters
{
  MoCap_Position,
  MoCap_Velocity,
  MoCap_Quaternion,
  MoCap_Accelerated_Velocity,
  MoCap_Gyro
};

class MoCap_Data
{

public:
  MoCap_Data();
  ~MoCap_Data() = default;

  void node_sub(ros::NodeHandle nh);

  void tick(double dt)
  {
    sub_mocap_.tick(dt);

    sub_mocap_Lhd_acc.tick(dt);
    sub_mocap_Lhd_pose.tick(dt);
  }

  void Update_Data_();

  bool Datas_Online()
  {
    Data_stream =
        sub_mocap_.data().isValid() && sub_mocap_Lhd_pose.data().isValid() && sub_mocap_Lhd_acc.data().isValid();
    return Data_stream;
  }

  const Eigen::MatrixXd & get_LeftHandPose_seq()
  {
    return LeftHandPose_seq;
  }
  const Eigen::MatrixXd & get_LeftHandAcc_seq()
  {
    return LeftHandAcc_seq;
  }
  const Eigen::MatrixXd & get_RightHandPose_seq()
  {
    return RightHandPose_seq;
  }
  const Eigen::MatrixXd & get_RightHandAcc_seq()
  {
    return RightHandAcc_seq;
  }

  int ros_data_id()
  {
    if(sub_mocap_.data().isValid())
    {
      if(sub_mocap_.data().value().header.frame_id != "")
      {
        return std::stoi(sub_mocap_.data().value().header.frame_id);
      }
      return 0;
    }
    return 0;
  }

  sva::PTransformd get_pose(MoCap_Body_part part);
  sva::MotionVecd get_vel(MoCap_Body_part part);
  Eigen::Vector3d get_linear_acc(MoCap_Body_part part);

  double maxTime_ = 0.5;
  bool Data_stream = false;

  int foot_contact(MoCap_Body_part foot)
  {
    if(foot == LeftFoot)
    {
      return static_cast<int>(FootState.x());
    }
    else
    {
      return static_cast<int>(FootState.y());
    }
  }

  Eigen::Vector3d MoCap_Coord(MoCap_Body_part joint, MoCap_Parameters param);
  Eigen::Quaterniond MoCap_Quat(MoCap_Body_part joint);
  std::vector<float> GetParameters(MoCap_Body_part part, MoCap_Parameters param);
  Eigen::MatrixXd Get_Sequence(MoCap_Body_part part, MoCap_Parameters param);

  int N_samples_in = 60;
  double data_freq = 60;

  double LeftLeg_Z0 = 0;
  double RightLeg_Z0 = 0;

  int id = 0;

  std::chrono::high_resolution_clock::time_point t_clock;
  
  void convert_data(const std::string & data);


private:


  ROSMultiArrayStampedSubscriber sub_mocap_;
  ROSMultiArrayStampedSubscriber sub_mocap_Lhd_pose;
  ROSMultiArrayStampedSubscriber sub_mocap_Lhd_acc;
  ROSMultiArrayStampedSubscriber sub_mocap_Rhd_pose;
  ROSMultiArrayStampedSubscriber sub_mocap_Rhd_acc;

  Eigen::Vector2d FootState;
  Eigen::VectorXd m_Datas;
  Eigen::MatrixXd Datas;

  Eigen::MatrixXd LeftHandPose_seq;
  Eigen::MatrixXd LeftHandAcc_seq;
  Eigen::MatrixXd RightHandPose_seq;
  Eigen::MatrixXd RightHandAcc_seq;
};
