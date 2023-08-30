#pragma once
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace hippo_control {
namespace mixer_bluerov {
static constexpr int kOutputChannels = 8;
namespace InputChannels {
static constexpr int kTorqueX = 0;
static constexpr int kTorqueY = 1;
static constexpr int kTorqueZ = 2;
static constexpr int kThrustX = 3;
static constexpr int kThrustY = 4;
static constexpr int kThrustZ = 5;
static constexpr int kCount = 6;
}  // namespace InputChannels

enum ThrustDirection {
    forward,
    backward,
};

struct Mapping {
  std::array<double, InputChannels::kCount> input_limits{};
  double output_scaling;
};

struct Output {
  double total = 0.0;
  std::array<double, kOutputChannels> channels{};
};

struct ThrusterModel {
    double quadratic_coefficient;
    double linear_coefficient;
    double constant_coefficient;
    double minimum;
};

class SimpleMixer {
 public:
  SimpleMixer();
  double test;


  void SetMapping(int _index, const Mapping &_mapping);
  void SetMixerMatrix(Eigen::Matrix<double, InputChannels::kCount, kOutputChannels> mixer_matrix);
  void SetZeroThrustThreshold(double _v) { zero_throttle_threshold_ = _v; }
  inline double ZeroThrustThreshold() const { return zero_throttle_threshold_; }
  void SetConstantCoefficient(double _v, int idx) { thruster_models_[idx].constant_coefficient = _v; UpdateMinima();}
  inline double ConstantCoefficient(int idx) const { return thruster_models_[idx].constant_coefficient; }
  void SetLinearCoefficient(double _v, int idx) { thruster_models_[idx].linear_coefficient = _v; UpdateMinima();}
  inline double LinearCoefficient(int idx) const { return thruster_models_[idx].linear_coefficient; }
  void SetQuadraticCoefficient(double _v, int idx) { thruster_models_[idx].quadratic_coefficient = _v; ; UpdateMinima();}
  inline double QuadraticCoefficient(int idx) const { return thruster_models_[idx].quadratic_coefficient; }
  void SetMinimum(double _v, int idx) { thruster_models_[idx].minimum = _v; UpdateMinima();}
  inline double Minimum(int idx) const { return thruster_models_[idx].minimum; }
  void SetMaxRotationsPerSecond(double _v) { max_rotations_per_second_ = _v; }
  double MaxRotationsPerSecond() const { return max_rotations_per_second_; }

  std::array<double, kOutputChannels> Mix(
      const std::array<double, InputChannels::kCount> &_actuator_controls);

 private:
  void UpdateMinima();  // updates minimum such that there are no issues possible with the current mapping
  double ThrustToRevsPerSec(double _thrust, int direction);
  double RevsPerSecToThrust(double _thrust, int direction);
  double RevsPerSecToThrust(const ThrusterModel& model, double _thrust);
    /// @brief per motor mappings of torque/thrust commands
  Mapping mappings_[kOutputChannels];
  Output outputs_[kOutputChannels];
  Eigen::Matrix<double, InputChannels::kCount, kOutputChannels> mixer_matrix_;
  Eigen::Matrix<double, kOutputChannels, InputChannels::kCount> mixer_matrix_inverse_;
  double zero_throttle_threshold_;
  std::array<ThrusterModel, 2> thruster_models_;
  // used as a scaler to normalize the motor command
  double max_rotations_per_second_{1.0};
  double ApplyInput(
const std::array<double, InputChannels::kCount> &_actuator_controls);
  void ScaleOutputs(double _scale);
  void ResetOutputs();
};
}  // namespace mixer
}  // namespace hippo_control
