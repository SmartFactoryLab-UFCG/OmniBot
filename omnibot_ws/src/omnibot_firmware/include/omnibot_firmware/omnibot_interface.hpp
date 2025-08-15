// Em: /home/mateus/OmniBot/omnibot_ws/src/omnibot_firmware/include/omnibot_firmware/omnibot_interface.hpp

#ifndef OMNIBOT_INTERFACE_HPP
#define OMNIBOT_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>

namespace omnibot_firmware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OmnibotInterface : public hardware_interface::SystemInterface
{
public:
  OmnibotInterface();
  ~OmnibotInterface();

  // Métodos da interface SystemInterface que estamos a sobrepor
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Variáveis de membro
  LibSerial::SerialPort pico_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
};

}  // namespace omnibot_firmware

#endif // OMNIBOT_INTERFACE_HPPs