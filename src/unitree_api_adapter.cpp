/*
 * @file unitree_api_adapter.cpp
 * @date 1/10/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Weston Robot Pte. Ltd.
 */

#include "unitree_api_adapter/unitree_api_adapter.hpp"

namespace unitree_api_adapter {
controller_interface::InterfaceConfiguration
UnitreeApiAdapter::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : command_interface_types_) {
      if (!command_prefix_.empty()) {
        conf.names.push_back(command_prefix_ + "/" + joint_name + "/" +=
                             interface_type);
      } else {
        conf.names.push_back(joint_name + "/" += interface_type);
      }
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration
UnitreeApiAdapter::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" += interface_type);
    }
  }
  for (const auto &interface_type : imu_interface_types_) {
    conf.names.push_back(imu_name_ + "/" += interface_type);
  }
  for (const auto &ft_sensor_name : ft_sensor_names_) {
    for (const auto &interface_type : ft_sensor_interface_types_) {
      conf.names.push_back(ft_sensor_name + "/" += interface_type);
    }
  }

  return conf;
}

controller_interface::return_type UnitreeApiAdapter::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  (void)time;
  (void)period;
  UpdateLowState();
  UpdateHighState();
  UpdateJointCommand();

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_init() {
  // acquire parameters
  joint_names_ =
      auto_declare<std::vector<std::string> >("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string> >(
      "command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string> >(
      "state_interfaces", state_interface_types_);
  imu_interface_types_ = auto_declare<std::vector<std::string> >(
      "imu_interfaces", state_interface_types_);
  ft_sensor_interface_types_ = auto_declare<std::vector<std::string> >(
      "ft_sensor_interfaces", state_interface_types_);

  imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
  ft_sensor_suffix_ =
      auto_declare<std::string>("ft_sensor_suffix", ft_sensor_suffix_);
  ft_sensor_names_ = auto_declare<std::vector<std::string> >("ft_sensor_names",
                                                             ft_sensor_names_);

  command_prefix_ =
      auto_declare<std::string>("command_prefix", command_prefix_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  RCLCPP_INFO(get_node()->get_logger(),
              "Activating UnitreeSdk2Adapter controller");

  // assign command interfaces
  for (auto &interface : command_interfaces_) {
    std::string interface_name = interface.get_interface_name();
    if (const size_t pos = interface_name.find('/'); pos != std::string::npos) {
      command_interface_map_[interface_name.substr(pos + 1)]->push_back(
          interface);
    } else {
      command_interface_map_[interface_name]->push_back(interface);
    }
  }

  // assign state interfaces
  for (auto &interface : state_interfaces_) {
    if (interface.get_prefix_name() == imu_name_) {
      imu_state_interface_.emplace_back(interface);
    } else if (interface.get_prefix_name().find(ft_sensor_suffix_) !=
               std::string::npos) {
      ft_sensor_interface_map_[interface.get_interface_name()]->push_back(
          interface);
    } else {
      state_interface_map_[interface.get_interface_name()]->push_back(
          interface);
    }
  }

  // init sdk
  InitializeSdk();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_error(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeApiAdapter::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"

void UnitreeApiAdapter::InitializeSdk() {
  low_state_pub_ =
      this->get_node()->create_publisher<unitree_go::msg::LowState>("lowstate",
                                                                    10);

  low_cmd_sub_ = this->get_node()->create_subscription<unitree_go::msg::LowCmd>(
      "lowcmd", 10,
      std::bind(&UnitreeApiAdapter::LowCmdCallback, this,
                std::placeholders::_1));
}

void UnitreeApiAdapter::LowCmdCallback(unitree_go::msg::LowCmd::SharedPtr cmd) {
  std::lock_guard<std::mutex> lock(low_cmd_mutex_);
  low_cmd_ = *cmd;
}

void UnitreeApiAdapter::UpdateLowState() {
  unitree_go::msg::LowState low_state_msg;

  // update joint state and sensor data
  // motor sequence, currently only 12 motors are used
  //  FR_ 0->0, FR_ 1->1, FR_ 2->2
  // FL_ 0->3, FL_ 1->4, FL_ 2->5 RR_ 0->6, RR_ 1->7, RR_
  // 2->8 RL_ 0->9, RL_ 1->10, RL_ 2->11
  for (int i = 0; i < 4; i++) {
    low_state_msg.motor_state[i * 3].q =
        joint_position_state_interface_[i * 3].get().get_value();
    low_state_msg.motor_state[i * 3 + 1].q =
        joint_position_state_interface_[i * 3 + 1].get().get_value();
    low_state_msg.motor_state[i * 3 + 2].q =
        joint_position_state_interface_[i * 3 + 2].get().get_value();

    low_state_msg.motor_state[i * 3].dq =
        joint_velocity_state_interface_[i * 3].get().get_value();
    low_state_msg.motor_state[i * 3 + 1].dq =
        joint_velocity_state_interface_[i * 3 + 1].get().get_value();
    low_state_msg.motor_state[i * 3 + 2].dq =
        joint_velocity_state_interface_[i * 3 + 2].get().get_value();

    low_state_msg.motor_state[i * 3].tau_est =
        joint_effort_state_interface_[i * 3].get().get_value();
    low_state_msg.motor_state[i * 3 + 1].tau_est =
        joint_effort_state_interface_[i * 3 + 1].get().get_value();
    low_state_msg.motor_state[i * 3 + 2].tau_est =
        joint_effort_state_interface_[i * 3 + 2].get().get_value();
  }

  // quaternion: w,x,y,z
  low_state_msg.imu_state.quaternion[0] =
      imu_state_interface_[0].get().get_value();
  low_state_msg.imu_state.quaternion[1] =
      imu_state_interface_[1].get().get_value();
  low_state_msg.imu_state.quaternion[2] =
      imu_state_interface_[2].get().get_value();
  low_state_msg.imu_state.quaternion[3] =
      imu_state_interface_[3].get().get_value();

  low_state_msg.imu_state.gyroscope[0] =
      imu_state_interface_[4].get().get_value();
  low_state_msg.imu_state.gyroscope[1] =
      imu_state_interface_[5].get().get_value();
  low_state_msg.imu_state.gyroscope[2] =
      imu_state_interface_[6].get().get_value();

  low_state_msg.imu_state.accelerometer[0] =
      imu_state_interface_[7].get().get_value();
  low_state_msg.imu_state.accelerometer[1] =
      imu_state_interface_[8].get().get_value();
  low_state_msg.imu_state.accelerometer[2] =
      imu_state_interface_[9].get().get_value();

  // force sensor
  // Arrays: 0-FR, 1-FL, 2-RR, 3-RL
  double fr_fx = ft_sensor_fx_state_interface_[0].get().get_value();
  double fr_fy = ft_sensor_fy_state_interface_[0].get().get_value();
  double fr_fz = ft_sensor_fz_state_interface_[0].get().get_value();
  low_state_msg.foot_force[0] =
      sqrt(fr_fx * fr_fx + fr_fy * fr_fy + fr_fz * fr_fz);

  double fl_fx = ft_sensor_fx_state_interface_[1].get().get_value();
  double fl_fy = ft_sensor_fy_state_interface_[1].get().get_value();
  double fl_fz = ft_sensor_fz_state_interface_[1].get().get_value();
  low_state_msg.foot_force[1] =
      sqrt(fl_fx * fl_fx + fl_fy * fl_fy + fl_fz * fl_fz);

  double rr_fx = ft_sensor_fx_state_interface_[2].get().get_value();
  double rr_fy = ft_sensor_fy_state_interface_[2].get().get_value();
  double rr_fz = ft_sensor_fz_state_interface_[2].get().get_value();
  low_state_msg.foot_force[2] =
      sqrt(rr_fx * rr_fx + rr_fy * rr_fy + rr_fz * rr_fz);

  double rl_fx = ft_sensor_fx_state_interface_[3].get().get_value();
  double rl_fy = ft_sensor_fy_state_interface_[3].get().get_value();
  double rl_fz = ft_sensor_fz_state_interface_[3].get().get_value();
  low_state_msg.foot_force[3] =
      sqrt(rl_fx * rl_fx + rl_fy * rl_fy + rl_fz * rl_fz);

  low_state_pub_->publish(low_state_msg);
}

void UnitreeApiAdapter::UpdateHighState() {
  // std::lock_guard<std::mutex> lock(high_state_mutex_);
  //   high_state.position()[0] = data_->sensordata[dim_motor_sensor_ + 10];
  //   high_state.position()[1] = data_->sensordata[dim_motor_sensor_ + 11];
  //   high_state.position()[2] = data_->sensordata[dim_motor_sensor_ + 12];
  //
  //   high_state.velocity()[0] = data_->sensordata[dim_motor_sensor_ + 13];
  //   high_state.velocity()[1] = data_->sensordata[dim_motor_sensor_ + 14];
  //   high_state.velocity()[2] = data_->sensordata[dim_motor_sensor_ + 15];
}

void UnitreeApiAdapter::UpdateJointCommand() {
  std::lock_guard<std::mutex> lock(low_cmd_mutex_);
  // update joint command
  for (int i = 0; i < 12; i++) {
    if (!joint_position_command_interface_[i].get().set_value(
            low_cmd_.motor_cmd[i].q) ||
        !joint_velocity_command_interface_[i].get().set_value(
            low_cmd_.motor_cmd[i].dq) ||
        !joint_torque_command_interface_[i].get().set_value(
            low_cmd_.motor_cmd[i].tau) ||
        !joint_kp_command_interface_[i].get().set_value(
            low_cmd_.motor_cmd[i].kp) ||
        !joint_kd_command_interface_[i].get().set_value(
            low_cmd_.motor_cmd[i].kd)) {
      RCLCPP_INFO(get_node()->get_logger(), "Failed to set command to joint %d",
                  i);
    }

    // RCLCPP_INFO(get_node()->get_logger(),
    //             "setting joint %d: %f, %f, %f, %f, %f", i,
    //             low_cmd_.motor_cmd[i].q, low_cmd_.motor_cmd[i].dq,
    //             low_cmd_.motor_cmd[i].tau, low_cmd_.motor_cmd[i].kp,
    //             low_cmd_.motor_cmd[i].kd);
  }
}
}  // namespace unitree_api_adapter

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_api_adapter::UnitreeApiAdapter,
                       controller_interface::ControllerInterface);
