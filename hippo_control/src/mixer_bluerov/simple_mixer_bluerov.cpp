#include "hippo_control/mixer_bluerov/simple_mixer_bluerov.hpp"

#include <math.h>

namespace hippo_control {
    namespace mixer_bluerov {

        SimpleMixer::SimpleMixer() {};

        void SimpleMixer::SetMapping(int _index, const Mapping &_mapping) {
          if ((_index >= kOutputChannels)) {
            return;
          }
          mappings_[_index] = _mapping;
        }


        void SimpleMixer::SetMixerMatrix(Eigen::Matrix<double, InputChannels::kCount, kOutputChannels> mixer_matrix) {
          mixer_matrix_ = mixer_matrix;
          mixer_matrix_inverse_ = mixer_matrix_.completeOrthogonalDecomposition().pseudoInverse();
        }

        void SimpleMixer::ResetOutputs() {
          for (Output &output: outputs_) {
            output.total = 0.0;
            output.channels.fill(0.0);
          }
        }

        void SimpleMixer::UpdateMinima() {
          double extremum;
          double eps = 1e-8;
          for (ThrusterModel &model: thruster_models_) {
            if (model.quadratic_coefficient > 0) {
              extremum =
                      -model.linear_coefficient / std::max(model.quadratic_coefficient, eps); // assumes that parabola is opened on top
            } else if (model.quadratic_coefficient == 0.0){
              extremum = 0.0;
            } else {
              std::cout << "Expected quadratic coefficient greater or equal zero, but got: " << model.quadratic_coefficient
                        << std::endl;
              return;
            }
            if (extremum > 0.0) {
              model.minimum = std::max(model.minimum, RevsPerSecToThrust(model, extremum) + eps);
            } else {
              model.minimum = std::max(model.minimum, RevsPerSecToThrust(model, 0.0) + eps);
            }
          }
        }

        double SimpleMixer::ApplyInput(
                const std::array<double, InputChannels::kCount> &_actuator_controls) {
          ResetOutputs();
          // scaling factor to scale the maximum output to 1.0, if any output is > 1.0
          double scaling = 1.0;
          for (int i_out = 0; i_out < kOutputChannels; ++i_out) {
            for (int i_in = 0; i_in < InputChannels::kCount; ++i_in) {
              double tmp =
                      _actuator_controls[i_in] * mixer_matrix_inverse_(i_out, i_in);
              outputs_[i_out].total += tmp;
              outputs_[i_out].channels[i_in] += tmp;
            }
            double thrust = abs(outputs_[i_out].total);

            double output;
            if (outputs_[i_out].total >= 0) {
              output = ThrustToRevsPerSec(thrust, ThrustDirection::forward);
            } else {
              output = ThrustToRevsPerSec(thrust, ThrustDirection::backward);
            }

            output /= max_rotations_per_second_;
            output *= mappings_[i_out].output_scaling;
            if (outputs_[i_out].total < 0) {
              outputs_[i_out].total = -1.0 * output;
            } else {
              outputs_[i_out].total = output;
            }
            scaling = std::max(output, scaling);
          }
          return scaling;
        }

        double SimpleMixer::RevsPerSecToThrust(double _thrust, int direction) {
          return thruster_models_[direction].quadratic_coefficient * std::pow(_thrust, 2) +
                 thruster_models_[direction].linear_coefficient * _thrust +
                 thruster_models_[direction].constant_coefficient;

        }

        double SimpleMixer::RevsPerSecToThrust(const ThrusterModel& model, double _thrust) {
          return model.quadratic_coefficient * std::pow(_thrust, 2) +
                 model.linear_coefficient * _thrust +
                 model.constant_coefficient;

        }

        double SimpleMixer::ThrustToRevsPerSec(double _thrust, int direction) {
          if (_thrust < zero_throttle_threshold_) {
            return 0.0;
          }
          _thrust = std::max(_thrust, thruster_models_[direction].minimum);
          if (thruster_models_[direction].linear_coefficient == 0.0) {
            if (thruster_models_[direction].quadratic_coefficient == 0.0) {
              // it does not make sense to have F(n) = const, so return 0.0
              return 0.0;
            }
            // F(n) = an² + c
            return sqrt((_thrust - thruster_models_[direction].constant_coefficient) /
                        thruster_models_[direction].quadratic_coefficient);
          }

          if (thruster_models_[direction].quadratic_coefficient == 0.0) {
            // F(n) = bn + c
            return (_thrust - thruster_models_[direction].constant_coefficient) /
                   thruster_models_[direction].linear_coefficient;
          }
          // full quadratic polynomial
          return (-1.0 * thruster_models_[direction].linear_coefficient +
                  sqrt(4.0 * thruster_models_[direction].quadratic_coefficient * _thrust +
                       thruster_models_[direction].linear_coefficient * thruster_models_[direction].linear_coefficient -
                       4.0 * thruster_models_[direction].quadratic_coefficient *
                       thruster_models_[direction].constant_coefficient)) /
                 (2.0 * thruster_models_[direction].quadratic_coefficient);
        }

        void SimpleMixer::ScaleOutputs(double _scale) {
          for (Output &output: outputs_) {
            output.total /= _scale;
          }
        }

        std::array<double, kOutputChannels> SimpleMixer::Mix(
                const std::array<double, InputChannels::kCount> &_actuator_controls) {
          double scale = ApplyInput(_actuator_controls);
          if (scale > 1.0) {
            ScaleOutputs(scale);
          }
          std::array<double, kOutputChannels> out;
          for (int i = 0; i < kOutputChannels; ++i) {
            out[i] = outputs_[i].total;
          }
          return out;
        }

    }  // namespace mixer
}  // namespace hippo_control
