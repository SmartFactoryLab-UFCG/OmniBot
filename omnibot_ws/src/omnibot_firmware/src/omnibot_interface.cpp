#include "omnibot_firmware/omnibot_interface.hpp" // Altere o nome do seu .hpp
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <vector>
#include <string>

namespace omnibot_firmware // Novo namespace
{
class OmnibotInterface : public hardware_interface::SystemInterface // Nova classe
{
public:
  OmnibotInterface() = default;
  ~OmnibotInterface()
  {
    if (pico_.IsOpen())
    {
      try
      {
        pico_.Close();
      }
      catch (...)
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("OmnibotInterface"),
                            "Algo deu errado ao fechar a conexão com a porta " << port_);
      }
    }
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override
  {
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::FAILURE;
    }

    try
    {
      port_ = info_.hardware_parameters.at("port");
    }
    catch (const std::out_of_range& e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("OmnibotInterface"), "Nenhuma Porta Serial fornecida! A abortar");
      return CallbackReturn::FAILURE;
    }

    // O código original já é genérico e funcionará para 3 juntas
    velocity_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(rclcpp::get_logger("OmnibotInterface"), "A iniciar o hardware do robô...");
    // Redimensiona os vetores para 3 juntas
    velocity_commands_ = { 0.0, 0.0, 0.0 };
    position_states_ = { 0.0, 0.0, 0.0 };
    velocity_states_ = { 0.0, 0.0, 0.0 };
    last_run_ = rclcpp::Clock().now();

    try
    {
      pico_.Open(port_);
      pico_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("OmnibotInterface"),
                          "Algo deu errado ao interagir com a porta " << port_);
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("OmnibotInterface"), "Hardware iniciado, pronto para receber comandos");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(rclcpp::get_logger("OmnibotInterface"), "A parar o hardware do robô...");
    if (pico_.IsOpen())
    {
      try
      {
        pico_.Close();
      }
      catch (...)
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("OmnibotInterface"),
                            "Algo deu errado ao fechar a conexão com a porta " << port_);
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("OmnibotInterface"), "Hardware parado");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    if (pico_.IsDataAvailable())
    {
      std::string message;
      pico_.ReadLine(message);

      // Calcula o tempo decorrido para a integração da posição
      auto current_time = rclcpp::Clock().now();
      auto dt = (current_time - last_run_).seconds();
      last_run_ = current_time;

      std::stringstream ss(message);
      std::string token;
      // Processa cada parte da mensagem, separada por vírgulas. Ex: "1p30.0,"
      while (std::getline(ss, token, ','))
      {
        if (token.length() < 4) // Ignora tokens inválidos
        {
          continue;
        }

        try
        {
          // O primeiro char é o ID do motor (1, 2, ou 3)
          int motor_id = std::stoi(token.substr(0, 1));
          // Converte o ID para o índice do vetor (1->0, 2->1, 3->2)
          int motor_index = motor_id - 1;

          // O segundo char é o sinal ('p' ou 'n')
          int multiplier = (token.at(1) == 'p') ? 1 : -1;

          // O resto da string é a velocidade
          double velocity = std::stod(token.substr(2));
          
          // Garante que o índice é válido antes de aceder ao vetor
          if (motor_index >= 0 && motor_index < velocity_states_.size())
          {
            // Atualiza o estado da velocidade
            velocity_states_.at(motor_index) = multiplier * velocity;
            // Integra a posição usando a nova velocidade e o dt
            position_states_.at(motor_index) += velocity_states_.at(motor_index) * dt;
          }
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("OmnibotInterface"), 
            "Erro ao processar o token '" << token << "': " << e.what());
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    std::stringstream message_stream;
    // Formata a velocidade com uma casa decimal
    message_stream << std::fixed << std::setprecision(1);

    // Itera sobre os 3 comandos de velocidade
    for (size_t i = 0; i < velocity_commands_.size(); ++i)
    {
      // ID do motor (1, 2, ou 3)
      message_stream << (i + 1);

      // Sinal ('p' ou 'n')
      message_stream << (velocity_commands_.at(i) >= 0 ? 'p' : 'n');

      double abs_velocity = std::abs(velocity_commands_.at(i));

      // Adiciona um zero à esquerda se a velocidade for menor que 10.0
      if (abs_velocity < 10.0)
      {
        message_stream << "0";
      }

      message_stream << abs_velocity << ",";
    }

    try
    {
      pico_.Write(message_stream.str());
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("OmnibotInterface"),
                          "Algo deu errado ao enviar a mensagem "
                              << message_stream.str() << " para a porta " << port_);
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

private:
  LibSerial::SerialPort pico_; // Renomeado para pico_
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
};

}  // namespace omnibot_firmware

// Exporta a classe para que o ros2_control a encontre
PLUGINLIB_EXPORT_CLASS(omnibot_firmware::OmnibotInterface, hardware_interface::SystemInterface)