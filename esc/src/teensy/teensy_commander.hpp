#pragma once

#include <termios.h>

#include <esc_serial.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace esc {
namespace teensy {

constexpr int n_coeffs = 3;

inline double interpolate(const double &x, const double &x_low, const double &x_up, const double &y_low, const double &y_up){
    return (y_up - y_low) / (x_up - x_low) * (x - x_low);
}

class TeensyCommander : public rclcpp::Node {
 public:
  static constexpr int kThrottleInputTimeoutMs = 500;
  static constexpr int kReadSerialIntervalMs = 100;
  static constexpr int kPublishArmingStateIntervalMs = 500;
  static constexpr int kPublishBatteryVoltageIntervalMs = 500;
  explicit TeensyCommander(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string serial_port{"/dev/teensy_data"};
  };
  void DeclareParams();
  bool InitSerial(std::string _port_name);
  void InitPublishers();
  void InitTimers();
  void InitSubscribers();
  void InitServices();
  void SetThrottle(const std::array<double, 8> &_values);
  void SetThrottle(double _value);
  uint16_t InputToPWM(double input);
  void PublishArmingState();
  void PublishBatteryVoltage();
  void PublishThrusterValues(std::array<double, 8> &_values);

  void HandleActuatorControlsMessage(esc_serial::ActuatorControlsMessage &_msg);
  void HandleBatteryVoltageMessage(esc_serial::BatteryVoltageMessage &_msg);

  void ReadSerial();

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscribers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_sub_;
  void OnActuatorControls(hippo_msgs::msg::ActuatorControls::ConstSharedPtr);

  //////////////////////////////////////////////////////////////////////////////
  // Services
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_service_;

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnThrottleInputTimeout();
  void OnReadSerialTimer();
  void OnPublishArmingStateTimer();
  void OnPublishBatteryVoltageTimer();

  void ServeArming(const std_srvs::srv::SetBool_Request::SharedPtr _request,
                   std_srvs::srv::SetBool_Response::SharedPtr _response);

  //////////////////////////////////////////////////////////////////////////////
  // Timers
  //////////////////////////////////////////////////////////////////////////////
  struct Timers {
    rclcpp::TimerBase::SharedPtr control_timeout;
    rclcpp::TimerBase::SharedPtr publish_arming_state;
    rclcpp::TimerBase::SharedPtr publish_battery_voltage;
    rclcpp::TimerBase::SharedPtr read_serial;
  };

  Timers timers_;

  Params params_;
  int serial_port_;
  struct termios tty_;
  bool serial_initialized_{false};
  bool timed_out_{true};
  bool armed_{false};
  double battery_voltage_{0.0};

  struct Coefficients{
      std::array<double, n_coeffs> forward;
      std::array<double, n_coeffs> backward;
      double voltage;
  };

  struct MappingCoefficients{
      Coefficients upper;
      Coefficients lower;
  };

  MappingCoefficients mapping_coeffs_;

  esc_serial::Packet packet_;
};
}  // namespace teensy
}  // namespace esc
