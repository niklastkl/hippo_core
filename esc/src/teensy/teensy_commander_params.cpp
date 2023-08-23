#include <hippo_common/param_utils.hpp>

#include "teensy_commander.hpp"

namespace esc {
    namespace teensy {
        void TeensyCommander::DeclareParams() {
            std::string name;
            std::string description;
            rcl_interfaces::msg::ParameterDescriptor descriptor;

            name = "serial_port";
            description = "Full filename of the serial port (e.g. /dev/ttyACM0)";
            descriptor = hippo_common::param_utils::Description(description, true);
            {
                auto &param = params_.serial_port;
                param = declare_parameter(name, param, descriptor);
            }

            std::vector<std::string> prefixes = {"lower.", "upper."};
            std::vector<std::string> description_prefixes = {"lower", "upper"};
            for (int i = 0; i < int(prefixes.size()); i++) {
                Coefficients *coeffs;
                if (i == 0) {
                    coeffs = &mapping_coeffs_.lower;
                } else {
                    coeffs = &mapping_coeffs_.upper;
                }

                name = "coeffs_rpm_pwm." + prefixes[i] + "voltage";
                description = description_prefixes[i] + "voltage bound for determined coefficients";
                descriptor = hippo_common::param_utils::Description(description, true);
                coeffs->voltage = declare_parameter(name, 15.0, descriptor);

                std::vector<double> default_coeffs = {0.0, 0.0, 1500.0};
                std::vector<double> loaded_coeffs;
                name = "coeffs_rpm_pwm." + prefixes[i] + "forward";
                description = description_prefixes[i] + " coefficients for forward turning direction";
                descriptor = hippo_common::param_utils::Description(description, true);
                loaded_coeffs = declare_parameter(name, default_coeffs, descriptor);
                if (loaded_coeffs.size() != n_coeffs) {
                    coeffs->forward = {0.0, 0.0, 1500.0};
                    RCLCPP_ERROR(this->get_logger(), "%s", ("Dimension of declared mapping coefficients" +
                                                            std::to_string(int(loaded_coeffs.size())) +
                                                            "is unequal " + std::to_string(n_coeffs) +"!").c_str());
                } else {
                    std::copy(loaded_coeffs.begin(), loaded_coeffs.end(), coeffs->forward.begin());
                }

                name = "coeffs_rpm_pwm." + prefixes[i] + "backward";
                description = description_prefixes[i] + " coefficients for forward turning direction";
                descriptor = hippo_common::param_utils::Description(description, true);
                loaded_coeffs = declare_parameter(name, default_coeffs, descriptor);
                if (loaded_coeffs.size() != n_coeffs) {
                    coeffs->backward = {0.0, 0.0, 1500.0};
                    RCLCPP_ERROR(this->get_logger(), "%s", ("Dimension of declared mapping coefficients" +
                                                            std::to_string(int(loaded_coeffs.size())) +
                                                            "is unequal " + std::to_string(n_coeffs) +"!").c_str());
                } else {
                    std::copy(loaded_coeffs.begin(), loaded_coeffs.end(), coeffs->backward.begin());
                }
            }


        }
    }  // namespace teensy
}  // namespace esc
