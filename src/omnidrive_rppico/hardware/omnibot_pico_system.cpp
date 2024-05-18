#include "omnidrive3_rppico/omnibot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnidrive3_rppico
{
hardware_interface::CallbackReturn omnidrive3RpPicoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.wheel1_name = info_.hardware_parameters["wheel1_name"];
  cfg_.wheel2_name = info_.hardware_parameters["wheel2_name"];
  cfg_.wheel3_name = info_.hardware_parameters["wheel3_name"];
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Setting params from URDF");

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baudrate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_1_.setup(cfg_.wheel1_name, cfg_.enc_counts_per_rev);
  wheel_2_.setup(cfg_.wheel2_name, cfg_.enc_counts_per_rev);
  wheel_3_.setup(cfg_.wheel3_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Setting up hardware interfaces");
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("omnidrive3RpPicoHardware"), "Loading joint '%s'", joint.name.c_str());

    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("omnidrive3RpPicoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("omnidrive3RpPicoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("omnidrive3RpPicoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("omnidrive3RpPicoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("omnidrive3RpPicoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Successfully set up hardware interfaces");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> omnidrive3RpPicoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Exporting state interfaces");
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_POSITION, &wheel_1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.vel));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_POSITION, &wheel_2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_POSITION, &wheel_3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.vel));  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> omnidrive3RpPicoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn omnidrive3RpPicoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn omnidrive3RpPicoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn omnidrive3RpPicoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn omnidrive3RpPicoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type omnidrive3RpPicoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_1_.enc, wheel_2_.enc, wheel_3_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_1_.pos;
  wheel_1_.pos = wheel_1_.calc_enc_angle();
  wheel_1_.vel = (wheel_1_.pos - pos_prev) / delta_seconds;
  
  pos_prev = wheel_1_.pos;
  wheel_2_.pos = wheel_2_.calc_enc_angle();
  wheel_2_.vel = (wheel_2_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_3_.pos;
  wheel_3_.pos = wheel_3_.calc_enc_angle();
  wheel_3_.vel = (wheel_3_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type omnidrive3_rppico ::omnidrive3RpPicoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_1_counts_per_loop = wheel_1_.cmd / wheel_1_.rads_per_count;
  int motor_2_counts_per_loop = wheel_2_.cmd / wheel_2_.rads_per_count;
  int motor_3_counts_per_loop = wheel_3_.cmd / wheel_3_.rads_per_count;
  


  comms_.set_motor_values(motor_1_counts_per_loop, 
                          motor_2_counts_per_loop,
                          motor_3_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace omnidrive3_rppico

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omnidrive3_rppico::omnidrive3RpPicoHardware, hardware_interface::SystemInterface)
