#include "MoCap.h"

MoCap_Data::MoCap_Data()
{
  m_Datas = Eigen::VectorXd::Zero(16 * 21 + 3);
  Datas = Eigen::MatrixXd::Zero(N_samples_in, 21 * 16 + 3);

  LeftHandPose_seq = Eigen::MatrixXd::Zero(N_samples_in, 3);
  LeftHandAcc_seq = Eigen::MatrixXd::Zero(N_samples_in, 3);
}
void MoCap_Data::node_sub(ros::NodeHandle nh)
{
  sub_mocap_.subscribe(nh, "MoCap/Data");
  sub_mocap_.maxTime(maxTime_);

  sub_mocap_Lhd_pose.subscribe(nh, "MoCap/L_HandPose_seq");
  sub_mocap_.maxTime(maxTime_);

  sub_mocap_Lhd_acc.subscribe(nh, "MoCap/L_HandAcc_seq");
  sub_mocap_.maxTime(maxTime_);

  sub_mocap_Rhd_pose.subscribe(nh, "MoCap/R_HandPose_seq");
  sub_mocap_.maxTime(maxTime_);

  sub_mocap_Rhd_acc.subscribe(nh, "MoCap/R_HandAcc_seq");
  sub_mocap_.maxTime(maxTime_);

  m_Datas = Eigen::VectorXd::Zero(16 * 21 + 3);
  Datas = Eigen::MatrixXd::Zero(N_samples_in, 21 * 16 + 3);
}

void MoCap_Data::Update_Data_()
{

  std::vector<float> data;
  data = sub_mocap_.data().value().data;
  std::vector<double> double_data(data.begin(), data.end());
  // std::cout << Datas.rows() << " " << Datas.cols() << std::endl;
  Datas.block(0, 0, N_samples_in - 1, Datas.cols()) = Datas.block(1, 0, N_samples_in - 1, Datas.cols());
  m_Datas = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(double_data.data(), m_Datas.size());
  Datas.row(N_samples_in - 1) = m_Datas.segment(0, Datas.cols());
  FootState.x() = MoCap_Coord(RightHandThumb1, MoCap_Position).x();
  FootState.y() = MoCap_Coord(RightHandThumb1, MoCap_Position).y();

  data = sub_mocap_Lhd_pose.data().value().data;
  int N_samples = (int)(data.size() / 3);
  double_data = std::vector<double>(data.begin(), data.end());
  LeftHandPose_seq = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(double_data.data(), N_samples, 3);

  data = sub_mocap_Lhd_acc.data().value().data;
  N_samples = (int)(data.size() / 3);
  double_data = std::vector<double>(data.begin(), data.end());
  LeftHandAcc_seq = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(double_data.data(), N_samples, 3);

  data = sub_mocap_Rhd_pose.data().value().data;
  N_samples = (int)(data.size() / 3);
  double_data = std::vector<double>(data.begin(), data.end());
  RightHandPose_seq = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(double_data.data(), N_samples, 3);

  data = sub_mocap_Rhd_acc.data().value().data;
  N_samples = (int)(data.size() / 3);
  double_data = std::vector<double>(data.begin(), data.end());
  RightHandAcc_seq = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(double_data.data(), N_samples, 3);
}

sva::PTransformd MoCap_Data::get_pose(MoCap_Body_part part)
{
  return sva::PTransformd(MoCap_Quat(part).normalized().toRotationMatrix().transpose(),
                          MoCap_Coord(part, MoCap_Position));
}
sva::MotionVecd MoCap_Data::get_vel(MoCap_Body_part part)
{
  return sva::MotionVecd(MoCap_Coord(part, MoCap_Gyro), MoCap_Coord(part, MoCap_Position));
}
Eigen::Vector3d MoCap_Data::get_linear_acc(MoCap_Body_part part)
{
  return Eigen::Vector3d(MoCap_Coord(part, MoCap_Accelerated_Velocity));
}

Eigen::MatrixXd MoCap_Data::Get_Sequence(MoCap_Body_part part, MoCap_Parameters param)
{

  Eigen::MatrixXd Output = Eigen::MatrixXd::Zero(3, N_samples_in);

  if(param > MoCap_Quaternion)
  {
    Output = Datas.block(0, 16 * part + param * 3 + 1, N_samples_in, 3).transpose();
  }
  else if(param == MoCap_Quaternion)
  {

    Output = Eigen::MatrixXd::Zero(4, N_samples_in);
    Output = Datas.block(0, 16 * part + param * 3 + 1, N_samples_in, 4).transpose();
  }
  else
  {

    Output = Datas.block(0, 16 * part + param * 3, N_samples_in, 3).transpose();
  }

  if(param == MoCap_Accelerated_Velocity)
  {

    Output.row(2) -= Eigen::VectorXd::Ones(Output.cols());
    Output *= 9.8;
  }

  return Output;
}

std::vector<float> MoCap_Data::GetParameters(MoCap_Body_part part, MoCap_Parameters param)
{

  std::vector<float> output(3);

  if(param > MoCap_Quaternion)
  {

    for(int i = 0; i < 3; ++i)
    {

      output[i] = (m_Datas(16 * part + param * 3 + 1 + i));
    }
    return output;
  }
  else if(param == MoCap_Quaternion)
  {
    for(int i = 0; i < 3; ++i)
    {

      output[i] = (m_Datas(16 * part + param * 3 + i));
    }
    output.push_back(m_Datas(16 * part + param * 3 + 3));
    return output;
  }
  else
  {
    for(int i = 0; i < 3; ++i)
    {

      output[i] = (m_Datas(16 * part + param * 3 + i));
    }
    return output;
  }
  return output;
}

Eigen::Vector3d MoCap_Data::MoCap_Coord(MoCap_Body_part joint, MoCap_Parameters param)
{

  std::vector<float> DataOut(GetParameters(joint, param));
  Eigen::Vector3d Output = Eigen::Vector3d{DataOut[0], DataOut[1], DataOut[2]};
  if(param == MoCap_Accelerated_Velocity)
  {

    Output.z() -= 1;
    Output *= -9.8;
  }

  return Output;
}

Eigen::Quaterniond MoCap_Data::MoCap_Quat(MoCap_Body_part joint)
{
  std::vector<float> vec(GetParameters(joint, MoCap_Quaternion));
  Eigen::Quaterniond output;

  output.x() = vec[1];
  output.y() = vec[2];
  output.z() = vec[3];
  output.w() = vec[0];

  return output;
}
