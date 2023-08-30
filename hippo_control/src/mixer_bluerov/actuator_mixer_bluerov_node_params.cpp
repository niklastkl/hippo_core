#include "actuator_mixer_bluerov_node.hpp"

namespace hippo_control {
namespace mixer_bluerov {
void ActuatorMixerNode::DeclareParams() {
  /*
    std::string name = "mixer_matrix";
  size_t matrix_size;
  auto mixer_matrix = declare_parameter<std::vector<double>>(
      name, hippo_common::param_utils::Description("Mixer Matrix", true));

  matrix_size = mixer_bluerov::kOutputChannels * mixer_bluerov::InputChannels::kCount;
  if (mixer_matrix.size() != matrix_size) {
    throw std::runtime_error("Invalid size of Mixer Matrix. Expected " +
                             std::to_string(matrix_size) + " but got " +
                             std::to_string(mixer_matrix.size()));
  }
  */
  size_t matrix_size;
  matrix_size = mixer_bluerov::kOutputChannels * mixer_bluerov::InputChannels::kCount;

  std::string name;
  Eigen::Matrix<double, mixer_bluerov::InputChannels::kCount, mixer_bluerov::kOutputChannels> mixer_matrix;
  name = "geometry.alpha_f";
  double alpha_f = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster angle front", true));
  name = "geometry.alpha_r";
  double alpha_r = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster angle rear", true));
  name = "geometry.l_hf";
  double l_hf = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster lever horizontal front", true));
  name = "geometry.l_hr";
  double l_hr = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster lever horizontal rear", true));
  name = "geometry.l_vx";
  double l_vx = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster lever vertical x", true));
  name = "geometry.l_vy";
  double l_vy = declare_parameter<double>(name, hippo_common::param_utils::Description("Thruster lever vertical y", true));

  mixer_matrix << 0, 0, 0, 0, -l_vy, -l_vy, l_vy, l_vy,
                  0, 0, 0, 0, -l_vx, l_vx, -l_vx, l_vx,
                  l_hf, -l_hf, l_hr, -l_hr, 0, 0, 0, 0,
                  cos(alpha_f), cos(alpha_f), cos(alpha_r), cos(alpha_r), 0, 0, 0, 0,
                  sin(alpha_f), -sin(alpha_f), -sin(alpha_r), sin(alpha_r), 0, 0, 0, 0,
                  0, 0, 0, 0, 1, -1, -1, 1;


  name = "input_limits";
  auto limit_matrix = declare_parameter<std::vector<double>>(
      name, hippo_common::param_utils::Description("Input Limits", true));
  matrix_size = mixer_bluerov::kOutputChannels * mixer_bluerov::InputChannels::kCount;
  if (limit_matrix.size() != matrix_size) {
    throw std::runtime_error(
        "Invalid size of input_limits parameter. Expected " +
        std::to_string(matrix_size) + " but got " +
        std::to_string(limit_matrix.size()));
  }

  std::string descr;
  rcl_interfaces::msg::ParameterDescriptor param;

  name = "output_scalings";
  descr =
      "Scaling factor for motor signals after normalization to [-1.0, 1.0].";
  param = hippo_common::param_utils::Description(descr);
  auto output_scalings = declare_parameter<std::vector<double>>(name, param);
  if (output_scalings.size() != kOutputChannels) {
    throw std::runtime_error(
        "Invalid size for output_scalings parameter. Expected " +
        std::to_string(kOutputChannels) + " but got " +
        std::to_string(output_scalings.size()));
  }

  static constexpr int cols = mixer_bluerov::InputChannels::kCount;
  static constexpr int rows = mixer_bluerov::kOutputChannels;
  for (int i_out = 0; i_out < rows; ++i_out) {
    mixer_bluerov::Mapping mapping;
    for (int i_in = 0; i_in < cols; ++i_in) {
      mapping.input_limits[i_in] = limit_matrix[i_out * cols + i_in];
    }
    mapping.output_scaling = output_scalings[i_out];
    mixer_.SetMapping(i_out, mapping);
    mixer_.SetMixerMatrix(mixer_matrix);
  }

  name = "zero_throttle_threshold";
  descr = "Thrust threshold until which zero output is sent.";
  param = hippo_common::param_utils::Description(descr);
  auto zero_thrust_threshold = declare_parameter<double>(name, param);
  mixer_.SetZeroThrustThreshold(zero_thrust_threshold);

  std::unordered_map<int, std::string> prefixes;
  prefixes[mixer_bluerov::ThrustDirection::forward] = "forward.";
  prefixes[mixer_bluerov::ThrustDirection::backward] = "backward.";
  std::vector<int> idxs = {mixer_bluerov::ThrustDirection::forward, mixer_bluerov::ThrustDirection::backward};
  for (auto i : idxs) {
    name = prefixes.at(i) + "constant_coefficient";
    descr = "Constant coefficient c of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto constant_coefficient = declare_parameter<double>(name, param);
    mixer_.SetConstantCoefficient(constant_coefficient, i);

    name = prefixes.at(i) + "linear_coefficient";
    descr = "Linear coefficient b of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto linear_coefficient = declare_parameter<double>(name, param);
    mixer_.SetLinearCoefficient(linear_coefficient, i);

    name = prefixes.at(i) + "quadratic_coefficient";
    descr = "Quadratic coefficient a of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto quadratic_coefficient = declare_parameter<double>(name, param);
    mixer_.SetQuadraticCoefficient(quadratic_coefficient, i);

    name = prefixes.at(i) + "minimum";
    descr = "Minimum input value";
    param = hippo_common::param_utils::Description(descr);
    auto minimum = declare_parameter<double>(name, param);
    mixer_.SetMinimum(minimum, i);
  }

  name = "max_rotations_per_second";
  descr = "The thrusters maximum rotations per second used for normalization.";
  param = hippo_common::param_utils::Description(descr);
  auto max_rotations_per_second = declare_parameter<double>(name, param);
  mixer_.SetMaxRotationsPerSecond(max_rotations_per_second);
}

rcl_interfaces::msg::SetParametersResult ActuatorMixerNode::OnThrustParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Undhandled";
  std::unordered_map<int, std::string> prefixes;
  prefixes.at(mixer_bluerov::ThrustDirection::forward) = "forward.";
  prefixes.at(mixer_bluerov::ThrustDirection::backward) = "backward.";
  std::vector<int> idxs = {mixer_bluerov::ThrustDirection::forward, mixer_bluerov::ThrustDirection::backward};
  for (const rclcpp::Parameter &parameter : _parameters) {
    double tmp_double;
    bool found = false;
    for (auto i : idxs){
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "linear_coefficient", tmp_double)) {
        mixer_.SetLinearCoefficient(tmp_double, i);
        result.reason = "Set linear_coefficient.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "quadratic_coefficient", tmp_double)) {
        mixer_.SetQuadraticCoefficient(tmp_double, i);
        result.reason = "Set quadratic_coefficient.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "constant_coefficient", tmp_double)) {
        mixer_.SetConstantCoefficient(tmp_double, i);
        result.reason = "Set constant_coefficient.";
        found = true;
        break;
      }
    }
    if (found){
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "zero_thrust_threshold", tmp_double)) {
      mixer_.SetZeroThrustThreshold(tmp_double);
      result.reason = "Set zero_thrust_threshold";
      continue;
    }
  }
  return result;
}
}  // namespace mixer
}  // namespace hippo_control
