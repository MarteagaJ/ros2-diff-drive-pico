#include "diff_drive_pico.hpp"

namespace diff_drive_pico
{
    hardware_interface::CallbackReturn DiffDrivePico::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (
            hardware_interface::ActuatorInterface::on_init(hardware_info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        first = 1;
        prev_ticks_l = 0;
        prev_ticks_r = 0;
        leftticks = 0;
        rightticks = 0;
        lefttick_offset = 0;
        righttick_offset = 0;

        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
        // hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
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
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDrivePico::export_state_interfaces()
    {
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

    std::vector<hardware_interface::CommandInterface> DiffDrivePico::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDrivePico::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Activating ...please wait...");

        // for (auto i = 0; i < hw_start_sec_; i++)
        // {
        //     rclcpp::sleep_for(std::chrono::seconds(1));
        //     RCLCPP_INFO(
        //         rclcpp::get_logger("AutonomousWaiterSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
        // }
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDrivePico::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Deactivating ...please wait...");

        // for (auto i = 0; i < hw_stop_sec_; i++)
        // {
        //     rclcpp::sleep_for(std::chrono::seconds(1));
        //     RCLCPP_INFO(
        //         rclcpp::get_logger("AutonomousWaiterSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
        // }
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffDrivePico::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        lcm::LCM lcmInstanceEncoder(MULTICAST_URL);
        if (!lcmInstanceEncoder.good())
        {
            RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "LCM IS BADDDDD");
        }
        float radius = 0.042; // radius of the wheels
        float enc2meters = ((2.0 * 3.14159 * radius) / (78.0 * 20.0));
        lcmInstanceEncoder.subscribe(MBOT_ENCODERS_CHANNEL, &DiffDrivePico::handleEncoders, this);
        lcmInstanceEncoder.handle();
        if (first)
        {
            first = 0;

            lefttick_offset = leftticks;
            righttick_offset = rightticks;

            prev_ticks_l = lefttick_offset;
            prev_ticks_r = righttick_offset;
        }

        for (uint i = 0; i < hw_commands_.size(); i++)
        {
            // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Leftticks: %d, and prev_ticks_l: %d  DURING  State Interface Updates", leftticks, prev_ticks_l);
            // RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Rightticks: %d, and prev_ticks_r: %d  DURING  State Interface Updates", rightticks, prev_ticks_r);
            if (i == 0)
            {
                hw_velocities_[i] = (((leftticks - lefttick_offset) - (prev_ticks_l - lefttick_offset)) / period.seconds()) * enc2meters / radius;
                hw_positions_[i] = (leftticks - lefttick_offset) * enc2meters / radius;
            }
            else
            {
                hw_velocities_[i] = (((rightticks - righttick_offset) - (prev_ticks_r - righttick_offset)) / period.seconds()) * enc2meters / radius;
                hw_positions_[i] = (rightticks - righttick_offset) * enc2meters / radius;
            }

        }

        prev_ticks_l = leftticks;
        prev_ticks_r = rightticks;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffDrivePico::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        lcm::LCM lcmInstanceEncoder(MULTICAST_URL);
        lcm::LCM lcmInstanceMotor(MULTICAST_URL);
        if (!lcmInstanceEncoder.good())
        {
            RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "LCM IS BADDDDD");
        }
        if (!lcmInstanceMotor.good())
        {
            RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "LCM IS BADDDDD");
        }
        lcmInstanceEncoder.subscribe(MBOT_ENCODERS_CHANNEL, &DiffDrivePico::handleEncoders, this);
        lcmInstanceEncoder.handle();
        if (first)
        {
            first = 0;

            lefttick_offset = leftticks;
            righttick_offset = rightticks;

            prev_ticks_l = lefttick_offset;
            prev_ticks_r = righttick_offset;
        }

        float radius = 0.042; // radius of the wheels
        mbot_motor_command_t cmd;
        cmd.utime = (int)time.seconds();
        cmd.trans_v = hw_commands_[0] * radius;
        cmd.angular_v = hw_commands_[1] * radius;
        lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        // lcmInstance.publish(MBOT_TIMESYNC_CHANNEL, &timestamp);


        // RCLCPP_INFO(
        //     rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Got translational command %.5f and angular command %.5f at time %d!", cmd.trans_v,
        //     cmd.angular_v,
        //     ((int)cmd.utime));
        // }


        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diff_drive_pico::DiffDrivePico, hardware_interface::ActuatorInterface)
