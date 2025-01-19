/*
 * @file unitree_api_adapter.hpp
 * @date 1/10/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Weston Robot Pte. Ltd.
 */

#ifndef UNITREE_API_ADAPTER_HPP
#define UNITREE_API_ADAPTER_HPP

#include <controller_interface/controller_interface.hpp>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace unitree_api_adapter {
class UnitreeApiAdapter final
    : public controller_interface::ControllerInterface {
 public:
  CONTROLLER_INTERFACE_PUBLIC
  UnitreeApiAdapter() = default;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  void InitializeSdk();
  void UpdateLowState();
  void UpdateHighState();
  void UpdateJointCommand();
  void LowCmdCallback(unitree_go::msg::LowCmd::SharedPtr cmd);

  // internal variable for hardware interface setup
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> imu_interface_types_;

  std::string imu_name_;
  std::string command_prefix_;

  // command/state interfaces
  using LoanedCommandInterfaceRef =
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>;
  std::vector<LoanedCommandInterfaceRef> joint_torque_command_interface_;
  std::vector<LoanedCommandInterfaceRef> joint_position_command_interface_;
  std::vector<LoanedCommandInterfaceRef> joint_velocity_command_interface_;
  std::vector<LoanedCommandInterfaceRef> joint_kp_command_interface_;
  std::vector<LoanedCommandInterfaceRef> joint_kd_command_interface_;

  using LoanedStateInterfaceRef =
      std::reference_wrapper<hardware_interface::LoanedStateInterface>;
  std::vector<LoanedStateInterfaceRef> joint_effort_state_interface_;
  std::vector<LoanedStateInterfaceRef> joint_position_state_interface_;
  std::vector<LoanedStateInterfaceRef> joint_velocity_state_interface_;

  std::vector<LoanedStateInterfaceRef> imu_state_interface_;

  std::unordered_map<std::string, std::vector<LoanedCommandInterfaceRef>*>
      command_interface_map_ = {
          {"effort", &joint_torque_command_interface_},
          {"position", &joint_position_command_interface_},
          {"velocity", &joint_velocity_command_interface_},
          {"kp", &joint_kp_command_interface_},
          {"kd", &joint_kd_command_interface_}};

  std::unordered_map<std::string, std::vector<LoanedStateInterfaceRef>*>
      state_interface_map_ = {{"position", &joint_position_state_interface_},
                              {"effort", &joint_effort_state_interface_},
                              {"velocity", &joint_velocity_state_interface_}};

  // unitree sdk variables
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr high_state_pub_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_sub_;

  std::mutex low_cmd_mutex_;
  unitree_go::msg::LowCmd low_cmd_;
};
}  // namespace unitree_api_adapter

#endif  // UNITREE_API_ADAPTER_HPP
