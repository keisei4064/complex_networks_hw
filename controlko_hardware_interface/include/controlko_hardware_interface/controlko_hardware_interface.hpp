#ifndef CONTROLKO_HARDWARE_INTERFACE__CONTROLKO_HARDWARE_INTERFACE_HPP_
#define CONTROLKO_HARDWARE_INTERFACE__CONTROLKO_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controlko_hardware_interface/visibility_control.h"
#include "dr_denis_rrbot_comms.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace controlko_hardware_interface
{

class RRBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> joint_pos_commands_;
  std::vector<double> joint_vel_commands_;
  std::vector<double> joint_pos_states_;
  std::vector<double> joint_vel_states_;
  std::vector<double> joint_acc_states_;

  std::vector<double> gpio_commands_;
  std::vector<double> gpio_states_;
  std::vector<bool> gpio_ins_storage_;
  std::vector<bool> gpio_outs_storage_;
  std::vector<bool> gpio_cmds_storage_;
  std::vector<double> sensor_states_;

  std::vector<std::string> position_command_interface_names_;
  std::vector<std::string> velocity_command_interface_names_;

  std::unique_ptr<dr_denis_rrbot_comms::DrDenisRRBotComms> rrbot_comms_;
};

}  // namespace controlko_hardware_interface

#endif  // CONTROLKO_HARDWARE_INTERFACE____CONTROLKO_HARDWARE_INTERFACE_HPP_
