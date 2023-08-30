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


        void SimpleMixer::InitializeMixerMatrix(double alpha_f, double alpha_r, double l_hf, double l_hr, double l_vx, double l_vy){
          mixer_matrix_ <<  0, 0, 0, 0, -l_vy, -l_vy, l_vy, l_vy,
                            0, 0, 0, 0, -l_vx, l_vx, -l_vx, l_vx,
                            l_hf, -l_hf, l_hr, -l_hr, 0, 0, 0, 0,
                            cos(alpha_f), cos(alpha_f), cos(alpha_r), cos(alpha_r), 0, 0, 0, 0,
                            sin(alpha_f), -sin(alpha_f), -sin(alpha_r), sin(alpha_r), 0, 0, 0, 0,
                            0, 0, 0, 0, 1, -1, -1, 1;
          mixer_matrix_inverse_ = mixer_matrix_.completeOrthogonalDecomposition().pseudoInverse();
          for (int i = 0; i < kOutputHorizontal; i++) {
            for (int j = 0; j < InputChannels::kInputHorizontal; j++) {
              mixer_matrix_inverse_horizontal_(i, j) = mixer_matrix_inverse_(kOutputIdxsHorizontal[i], InputChannels::kIdxsHorizontal[j]);
            }
          }
          for (int i = 0; i < kOutputVertical; i++) {
            for (int j = 0; j < InputChannels::kInputVertical; j++) {
              mixer_matrix_inverse_vertical_(i, j) = mixer_matrix_inverse_(kOutputIdxsVertical[i], InputChannels::kIdxsVertical[j]);
            }
          }
          nullspace_horizontal_ << cos(alpha_r) / cos(alpha_f), cos(alpha_r) / cos(alpha_f), -1, -1;
          nullspace_vertical_.setOnes();
        }

        void SimpleMixer::ResetOutputs() {
          for (Output &output: outputs_) {
            output.total = 0.0;
            output.channels.fill(0.0);
          }
        }

        void SimpleMixer::UpdateMinima() {
          double extremum;
          for (ThrusterModel &model: thruster_models_) {
            if (model.quadratic_coefficient > 0) {
              extremum =
                      -model.linear_coefficient / std::max(model.quadratic_coefficient, eps_); // assumes that parabola is opened on top
            } else if (model.quadratic_coefficient == 0.0){
              extremum = 0.0;
            } else {
              std::cout << "Expected quadratic coefficient greater or equal zero, but got: " << model.quadratic_coefficient
                        << std::endl;
              return;
            }
            if (extremum > 0.0) {
              model.minimum = std::max(model.minimum, RevsPerSecToThrust(model, extremum) + eps_);
            } else {
              model.minimum = std::max(model.minimum, RevsPerSecToThrust(model, 0.0) + eps_);
            }
          }
        }

        template <int size_output>
        void SimpleMixer::IsInDeadzone(const Eigen::Matrix<double, size_output, 1> &thrusters0, double deadzone_min, double deadzone_max, const Eigen::Matrix<double, size_output, 1> &nullspace, const double &scaling, double &scaling_out, ScalingStatus &scaling_status) {
          for (int i = 0; i < size_output; i++) {
            if ((thrusters0(i) + scaling * nullspace(i) > deadzone_min) &&
                (thrusters0(i) + scaling * nullspace(i) < deadzone_max)) {
              if (((deadzone_min - thrusters0(i)) / nullspace(i) * scaling > 0)
                  && (abs((deadzone_min - thrusters0(i)) / nullspace(i)) > abs(scaling))) {
                scaling_out = (deadzone_min - thrusters0(i)) / nullspace(i) +
                              eps_ * scaling / abs(scaling);
                scaling_status.IS_FEASIBLE = true;
                scaling_status.IN_DEADZONE = true; // feasible, in deadzone, scaling
                return;
              } else if (((deadzone_max - thrusters0(i)) / nullspace(i) * scaling > 0) &&
                         (abs((deadzone_max - thrusters0(i)) / nullspace(i)) > abs(scaling))) {
                scaling_out = (deadzone_max - thrusters0(i)) / nullspace(i) + eps_ * scaling /
                                                                              abs(scaling);
                // return value slightly greater than deadzone bound to avoid being stuck at bound of deadzone
                scaling_status.IS_FEASIBLE = true;
                scaling_status.IN_DEADZONE = true; // feasible, in deadzone, scaling
                return;
              } else {
                scaling_out = scaling;
                scaling_status.IS_FEASIBLE = false;
                scaling_status.IN_DEADZONE = true; // feasible, in deadzone, scaling
                return;
              }
            }
          }
          scaling_out = scaling;
          scaling_status.IS_FEASIBLE = true;
          scaling_status.IN_DEADZONE = false; // feasible, in deadzone, scaling
        }


        template <int size_output>
        void SimpleMixer::IsSaturated(const Eigen::Matrix<double, size_output, 1> &thrusters, double limit_min, double limit_max, ScalingStatus &scaling_status) {
          for (int i = 0; i < size_output; i++) {
            if (thrusters[i] > limit_max || thrusters[i] < limit_min) {
              scaling_status.IS_SATURATED = true;
              return;
            }
          }
          scaling_status.IS_SATURATED = false;
          return;
        }

        template <int size_output>
        Eigen::Matrix<double, size_output, 1> SimpleMixer::ChooseBetter(const std::array<Eigen::Matrix<double, size_output, 1>, 2> &thrusters_scaled, const Eigen::Matrix<double, size_output, 1> &last_thrusters){
          double cost_0 = thrusters_scaled[0].norm() + weighting_last_input_ * (thrusters_scaled[0] - last_thrusters).norm();
          double cost_1 = thrusters_scaled[1].norm() + weighting_last_input_ * (thrusters_scaled[1] - last_thrusters).norm();
          if (cost_0 <= cost_1) {
            return thrusters_scaled[0];
          } else {
            return thrusters_scaled[1];
          }
        }

        template <int size_input, int size_output>
        Eigen::Matrix<double, size_output, 1> SimpleMixer::ResolveDeadzone(const Eigen::Matrix<double, size_output, size_input> &M_inv, const Eigen::Matrix<double, size_output, 1> &nullspace_vector,
                                     double deadzone_min, double deadzone_max, double limit_min, double limit_max, const Eigen::Matrix<double, size_input, 1> &force_input, const Eigen::Matrix<double, size_output, 1> &last_thruster) {
          Eigen::Matrix<double, size_output, 1> thrusters = M_inv * force_input;
          double out_scaling_tmp;
          ScalingStatus scaling_status{true, true, false};
          IsInDeadzone(thrusters, deadzone_min, deadzone_max, nullspace_vector, 0.0,
                                                        out_scaling_tmp, scaling_status);
          if (!scaling_status.IN_DEADZONE) {
            return thrusters;
          }

          std::array<double, 2> scalings = {eps_, -eps_};
          std::array<Eigen::Matrix<double, size_output, 1>, 2> thrusters_scaled = {thrusters, thrusters};
          std::array<bool, 2> thrusters_feasible = {false, false};

          for (int i = 0; i < 2; i++) {
            scaling_status.IS_FEASIBLE = true;
            scaling_status.IN_DEADZONE = true;
            scaling_status.IS_SATURATED = false;
            while (scaling_status.IS_FEASIBLE) {
              IsInDeadzone(thrusters, deadzone_min, deadzone_max, nullspace_vector, scalings[i], scalings[i],
                           scaling_status);
              const Eigen::Matrix<double, size_output, 1> tmp = thrusters + scalings[i] * nullspace_vector;
              IsSaturated(tmp, limit_min, limit_max, scaling_status);
              if (scaling_status.IS_SATURATED) {
                thrusters_feasible[i] = false;
                break;
              }
              if (scaling_status.IS_FEASIBLE && !scaling_status.IN_DEADZONE) {
                break;
              }
            }
            if (scaling_status.IS_FEASIBLE && !scaling_status.IN_DEADZONE && !scaling_status.IS_SATURATED) {
              thrusters_scaled[i] = thrusters + scalings[i] * nullspace_vector;
              thrusters_feasible[i] = true;
            } else {
              thrusters_feasible[i] = false;
            }
          }

          if (thrusters_feasible[0] && thrusters_feasible[1]) {
            return ChooseBetter(thrusters_scaled, last_thruster);
          } else if (thrusters_feasible[0]) {
            return thrusters_scaled[0];
          } else if (thrusters_feasible[1]) {
            return thrusters_scaled[1];
          } else {
            return thrusters;
          }
        }

        double SimpleMixer::ApplyInput(
                const std::array<double, InputChannels::kCount> &_actuator_controls) {
          ResetOutputs();
          // scaling factor to scale the maximum output to 1.0, if any output is > 1.0
          double scaling = 1.0;

          double limit_min = scaling_saturation_limit_low_ * RevsPerSecToThrust(thruster_models_[ThrustDirection::backward], max_rotations_per_second_);
          double limit_max = scaling_saturation_limit_up_ * RevsPerSecToThrust(thruster_models_[ThrustDirection::forward], max_rotations_per_second_);

          if (compensate_deadzone_){
            Eigen::Matrix<double, InputChannels::kInputHorizontal, 1> input_vec_horizontal;
            for (int i = 0; i < InputChannels::kInputHorizontal; i++) {
              input_vec_horizontal(i) = _actuator_controls[InputChannels::kIdxsHorizontal[i]];
            }
            Eigen::Matrix<double, kOutputHorizontal, 1> output_vec_horizontal;
            output_vec_horizontal = ResolveDeadzone(mixer_matrix_inverse_horizontal_, nullspace_horizontal_, -thruster_models_[ThrustDirection::backward].deadzone_minimum, thruster_models_[ThrustDirection::forward].deadzone_minimum, limit_min, limit_max, input_vec_horizontal, last_output_horizontal_);
            last_output_horizontal_ = output_vec_horizontal;
            Eigen::Matrix<double, InputChannels::kInputVertical, 1> input_vec_vertical;
            for (int i = 0; i < InputChannels::kInputVertical; i++) {
              input_vec_vertical(i) = _actuator_controls[InputChannels::kIdxsVertical[i]];
            }
            Eigen::Matrix<double, kOutputVertical, 1> output_vec_vertical;
            output_vec_vertical = ResolveDeadzone(mixer_matrix_inverse_vertical_, nullspace_vertical_, -thruster_models_[ThrustDirection::backward].deadzone_minimum, thruster_models_[ThrustDirection::forward].deadzone_minimum, limit_min, limit_max, input_vec_vertical, last_output_vertical_);
            last_output_vertical_ = output_vec_vertical;

            //check:
            /*
            Eigen::Vector<double, 6> ground_truth;
            for (int i = 0; i < 6; i++){
              ground_truth(i) = _actuator_controls[i];
            }
            Eigen::Vector<double, kOutputChannels> output_vec;
            for (int i = 0; i < kOutputHorizontal; i++) {
              output_vec(kOutputIdxsHorizontal[i]) = output_vec_horizontal[i];
            }
            for (int i = 0; i < kOutputVertical; i++) {
              output_vec(kOutputIdxsVertical[i]) = output_vec_vertical[i];
            }
             */

            // write back into outputs:
            for (int i = 0; i < kOutputHorizontal; i++) {
              outputs_[kOutputIdxsHorizontal[i]].total = output_vec_horizontal[i];
            }
            for (int i = 0; i < kOutputVertical; i++) {
              outputs_[kOutputIdxsVertical[i]].total = output_vec_vertical[i];
            }
          } else {
            for (int i_out = 0; i_out < kOutputChannels; ++i_out) {
              for (int i_in = 0; i_in < InputChannels::kCount; ++i_in) {
                double tmp =
                        _actuator_controls[i_in] * mixer_matrix_inverse_(i_out, i_in);
                outputs_[i_out].total += tmp;
                outputs_[i_out].channels[i_in] += tmp;
              }
            }
          }

          for (int i_out = 0; i_out < kOutputChannels; ++i_out) {
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
