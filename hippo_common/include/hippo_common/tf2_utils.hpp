#pragma once
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <string>
namespace hippo_common {
namespace tf2_utils {
Eigen::Quaterniond EulerToQuaternion(double _roll, double _pitch, double _yaw);
/// @brief Computes euler angles as roll, pitch, yaw from quaternion.
/// @param _q Has to be a normalized quaternion!
/// @return
inline Eigen::Vector3d QuaternionToEuler(const Eigen::Quaterniond &_q) {
  return _q.toRotationMatrix().eulerAngles(0, 1, 2);
}

/// @brief Make sure both vectors have unit length. Returns a quaternion
/// describing the rotation between both vectors so _v2 = q.rotate(_v1)
/// @param _v1
/// @param _v2
/// @return
Eigen::Quaterniond RotationBetweenNormalizedVectors(const Eigen::Vector3d &_v1,
                                                    const Eigen::Vector3d &_v2);
namespace frame_id {
static constexpr char kBarometerName[] = "barometer";
static constexpr char kBaseLinkName[] = "base_link";
static constexpr char kInertialName[] = "map";

inline std::string Prefix(rclcpp::Node *_node) {
  std::string name = _node->get_namespace();
  // remove leading slash as tf2 does not like leading slashes.
  name.erase(0, name.find_first_not_of('/'));
  return name;
}
inline std::string Barometer(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBarometerName;
}
inline std::string BaseLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBaseLinkName;
}

inline std::string InertialFrame() { return kInertialName; }

}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common
