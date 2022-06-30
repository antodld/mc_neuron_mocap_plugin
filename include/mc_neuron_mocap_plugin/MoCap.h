#pragma once
#include <mc_control/mc_controller.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <math.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

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

  int seq_size()
  {
    return sequence_size;
  }
  void seq_size(int size)
  {
    sequence_size = size;
  }
  void data_freq(int f)
  {
    data_freq_ = f;
  }
  int data_freq()
  {
    return data_freq_;
  }

  sva::PTransformd get_pose(MoCap_Body_part part);
  sva::MotionVecd get_vel(MoCap_Body_part part);
  Eigen::Vector3d get_linear_acc(MoCap_Body_part part);
  Eigen::MatrixXd get_sequence(MoCap_Body_part part, MoCap_Parameters param, int size,int freq);

  void convert_data(std::string & data);

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

private:
  Eigen::Vector3d MoCap_Coord(MoCap_Body_part joint, MoCap_Parameters param);
  Eigen::Quaterniond MoCap_Quat(MoCap_Body_part joint);
  Eigen::VectorXd GetParameters(MoCap_Body_part part, MoCap_Parameters param);

  int sequence_size = 60;
  int data_freq_ = 60;
  std::chrono::high_resolution_clock::time_point t_clock;

  int n_elements_ = 16 * 21 + 3; // Number of data to take from the stream;

  Eigen::Vector2d FootState;
  Eigen::VectorXd m_Datas;
  Eigen::MatrixXd Datas;

  Eigen::MatrixXd LeftHandPose_seq;
  Eigen::MatrixXd LeftHandAcc_seq;
  Eigen::MatrixXd RightHandPose_seq;
  Eigen::MatrixXd RightHandAcc_seq;
};
