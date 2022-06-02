#include "MoCap.h"

MoCap_Data::MoCap_Data()
{
  m_Datas = Eigen::VectorXd::Zero(n_elements_);
  Datas = Eigen::MatrixXd::Zero(sequence_size, n_elements_);

  LeftHandPose_seq = Eigen::MatrixXd::Zero(sequence_size, 3);
  LeftHandAcc_seq = Eigen::MatrixXd::Zero(sequence_size, 3);
}

void MoCap_Data::convert_data(const std::string & data)
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
  std::vector<double> double_data;
  // std::stringstream data_stream(data);
  // std::string item;
  // while (std::getline(data_stream, item, ' ')) {
  //     std::cout << " item " << item << std::endl;
  //     if(item.size() !=0){double_data.push_back(stod(item))};
  // }
  size_t indx_start = 0;
  size_t pos = 0;

  while(double_data.size() < n_elements_)
  {
    size_t pos = data.find(' ', indx_start);
    std::string double_val = data.substr(indx_start, pos - indx_start);
    if(pos < data.length()) (double_data.push_back(std::stod(double_val)));
    indx_start = pos + 1;
  }

  m_Datas = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(double_data.data(), double_data.size());
  FootState.x() = MoCap_Coord(RightHandThumb1, MoCap_Position).x();
  FootState.y() = MoCap_Coord(RightHandThumb1, MoCap_Position).y();
  Datas.block(0, 0, sequence_size - 1, Datas.cols()) = Datas.block(1, 0, sequence_size - 1, Datas.cols());
  Datas.row(sequence_size - 1) = m_Datas.segment(0, Datas.cols());

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info(time_span.count());
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

Eigen::MatrixXd MoCap_Data::get_sequence(MoCap_Body_part part, MoCap_Parameters param, int size)
{

  Eigen::MatrixXd Output = Eigen::MatrixXd::Zero(3, sequence_size);

  if(param > MoCap_Quaternion)
  {
    Output = Datas.block(sequence_size - size, 16 * part + param * 3 + 1, size, 3).transpose();
  }
  else if(param == MoCap_Quaternion)
  {

    Output = Eigen::MatrixXd::Zero(4, sequence_size);
    Output = Datas.block(sequence_size - size, 16 * part + param * 3 + 1, size, 4).transpose();
  }
  else
  {

    Output = Datas.block(sequence_size - size, 16 * part + param * 3, size, 3).transpose();
  }

  if(param == MoCap_Accelerated_Velocity)
  {

    Output.row(2) -= Eigen::VectorXd::Ones(Output.cols());
    Output *= 9.8;
  }

  return Output;
}

Eigen::VectorXd MoCap_Data::GetParameters(MoCap_Body_part part, MoCap_Parameters param)
{

  if(param > MoCap_Quaternion)
  {

    return m_Datas.segment(16 * part + param * 3 + 1, 3);
  }
  else if(param == MoCap_Quaternion)
  {

    return m_Datas.segment(16 * part + param * 3, 4);
  }
  else
  {
    return m_Datas.segment(16 * part + param * 3, 3);
  }
}

Eigen::Vector3d MoCap_Data::MoCap_Coord(MoCap_Body_part joint, MoCap_Parameters param)
{

  Eigen::Vector3d DataOut(GetParameters(joint, param));
  if(param == MoCap_Accelerated_Velocity)
  {

    DataOut.z() -= 1;
    DataOut *= -9.8;
  }

  return DataOut;
}

Eigen::Quaterniond MoCap_Data::MoCap_Quat(MoCap_Body_part joint)
{
  Eigen::VectorXd vec(GetParameters(joint, MoCap_Quaternion));
  Eigen::Quaterniond output;

  output.x() = vec[1];
  output.y() = vec[2];
  output.z() = vec[3];
  output.w() = vec[0];

  return output;
}
