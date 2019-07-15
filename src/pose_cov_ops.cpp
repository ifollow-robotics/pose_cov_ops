/*
 * pose_cov_ops.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: JLBC
 *
 */

#include "pose_cov_ops/pose_cov_ops.h"
#include <boost/python.hpp>

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

using namespace mrpt::poses;
using namespace mrpt::math;

void pose_cov_ops::compose(const geometry_msgs::Pose &a,
                           const geometry_msgs::Pose &b,
                           geometry_msgs::Pose &out)
{
  CPose3D A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE), OUT(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  OUT.composeFrom(A, B);
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance &a,
                           const geometry_msgs::PoseWithCovariance &b,
                           geometry_msgs::PoseWithCovariance &out)
{
  CPose3DPDFGaussian A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian OUT = A + B;
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance &a,
                           const geometry_msgs::Pose &b,
                           geometry_msgs::PoseWithCovariance &out)
{
  CPose3DPDFGaussian A(UNINITIALIZED_POSE);
  CPose3D B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  A += B;
  mrpt_bridge::convert(A, out);
}
void pose_cov_ops::compose(const geometry_msgs::Pose &a,
                           const geometry_msgs::PoseWithCovariance &b,
                           geometry_msgs::PoseWithCovariance &out)
{
  CPose3D A(UNINITIALIZED_POSE);
  CPose3DPDFGaussian B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  B.changeCoordinatesReference(A); // b = a (+) b
  mrpt_bridge::convert(B, out);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::Pose &out)
{
  CPose3D A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE), OUT(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  OUT.inverseComposeFrom(A, B);
  mrpt_bridge::convert(OUT, out);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out)
{
  CPose3DPDFGaussian A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::PoseWithCovariance &out)
{
  CPose3DPDFGaussian A(UNINITIALIZED_POSE);
  CPose3D B_mean(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B_mean);

  const CPose3DPDFGaussian B(B_mean, CMatrixDouble66()); // Cov=zeros

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out)
{
  CPose3D A_mean(UNINITIALIZED_POSE);
  CPose3DPDFGaussian B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A_mean);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian A(A_mean, CMatrixDouble66()); // Cov=zeros

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}

template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M &msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

std::string composePose(const std::string &str_a, const std::string &str_b)
{
  geometry_msgs::Pose a = from_python<geometry_msgs::Pose>(str_a);
  geometry_msgs::Pose b = from_python<geometry_msgs::Pose>(str_b);
  geometry_msgs::Pose trans_compose;
  pose_cov_ops::compose(a, b,trans_compose);
  return to_python(trans_compose);
}

// char const *greet(char const *str_a)
// {
//   return str_a;
// }

BOOST_PYTHON_MODULE(pose_cov_ops)
{
  using namespace boost::python;
  // def("greet", greet);
  def("composePose", composePose);
  
}