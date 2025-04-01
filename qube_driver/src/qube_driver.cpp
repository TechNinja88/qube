#include "qube_driver/qube_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <tuple>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief The ArduinoHardware class is a hardware interface for a single servo motor
 */
namespace qube_driver
{
    /**
     * @brief Initialize a new ArduinoHardware::ArduinoHardware object
     */
    hardware_interface::CallbackReturn ArduinoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Get the joint name, device, baud rate, and timeout from the hardware info
        cfg_.joint_name = info_.hardware_parameters["motor_name"];
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

        motor_.setup(cfg_.joint_name);

        // Check that the joint has the correct interfaces
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // Check that the joint has the correct command interfaces
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArduinoHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArduinoHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            // Check that the joint has the correct state interfaces
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArduinoHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArduinoHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArduinoHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Export the state interfaces for the joint
     * @return A vector of state interfaces
     */
    std::vector<hardware_interface::StateInterface> ArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                motor_.name, hardware_interface::HW_IF_POSITION, &motor_.angle));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_.name, hardware_interface::HW_IF_VELOCITY, &motor_.angular_velocity));
  
        return state_interfaces;
    }

    /**
     * @brief Export the command interfaces for the joint
     * @return A vector of command interfaces
     */     
    std::vector<hardware_interface::CommandInterface> ArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motor_.name, hardware_interface::HW_IF_VELOCITY, &motor_.velocity_cmd));

        return command_interfaces;
    }

    /**
     * @brief Configure the ArduinoHardware object
     * @param previous_state The previous state of the ArduinoHardware object
     * @return A callback return status
     */
    hardware_interface::CallbackReturn ArduinoHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Configuring ...please wait...");
        // Open the serial connection
        comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Clean up the ArduinoHardware object
     * @param previous_state The previous state of the ArduinoHardware object
     * @return A callback return status
     */
    hardware_interface::CallbackReturn ArduinoHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Cleaning up ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Activate the ArduinoHardware object
     * @param previous_state The previous state of the ArduinoHardware object
     * @return A callback return status
     */
    hardware_interface::CallbackReturn ArduinoHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Activating ...please wait...");
        // Start the communication thread
        if (!comms_.connected())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        comms_.start_communication_thread();
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivate the ArduinoHardware object
     * @param previous_state The previous state of the ArduinoHardware object
     * @return A callback return status
     */
    hardware_interface::CallbackReturn ArduinoHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Deactivating ...please wait...");
        // Stop the communication thread
        comms_.stop_communication_thread();
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Read the state of the joint
     * @param time The current time
     * @param period The time period
     * @return A return status
     */
    hardware_interface::return_type ArduinoHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Read the angle of the motor
        float read_angle = comms_.get_motor_angle();

        double delta_seconds = period.seconds();
        double angle_prev = 0;

        angle_prev = motor_.angle;
        motor_.angle = static_cast<double>(read_angle);
        motor_.angular_velocity = (motor_.angle - angle_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    /**
     * @brief Write the command to the joint
     * @param time The current time
     * @param period The time period
     * @return A return status
     */
    hardware_interface::return_type qube_driver ::ArduinoHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Set the speed of the motor as the velocity command from the controller
        comms_.set_motor_speed(motor_.velocity_cmd);
        return hardware_interface::return_type::OK;
    }

} // namespace qube_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    qube_driver::ArduinoHardware, hardware_interface::SystemInterface)