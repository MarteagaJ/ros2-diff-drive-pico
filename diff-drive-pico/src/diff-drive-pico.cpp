#include "diff-drive-pico.hpp"

namespace diff_drive_pico
{
    CallbackReturn DiffDrivePico::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (
            hardware_interface::ActuatorInterface::on_init(hardware_info) !=
            CallbackReturn::SUCCESS)
        {
            CallbackReturn::ERROR;
        }

        base_x_ = 0.0;
        base_y_ = 0.0;
        base_theta_ = 0.0;

        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
        hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
        // END: This part here is for exemplary purposes - Please do not copy to your production code
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDrivePico::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }

    CallbackReturn DiffDrivePico::on_configure(const State &previous_state)
    {
        RCLCPP_INFO(logger_, "Configuring...");
        RCLCPP_INFO(logger_, "Finished Configuration");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrivePico::on_cleanup(const State &previous_state)
    {
        RCLCPP_INFO(logger_, "Cleaning Up...");
        RCLCPP_INFO(logger_, "Finished Cleaning Up");
        return CallbackReturn::SUCCESS;
    }

    return_type DiffDrivePico::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        int fd;

        if ((fd = serialOpen("/dev/ttyACM0", 9600)) < 0)
        {
            fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
            return 1;
        }

        for (;;)
        {
            putchar(serialGetchar(fd));
            fflush(stdout);
        }
    }
}