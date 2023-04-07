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

        base_x_ = 0.0;
        base_y_ = 0.0;
        base_theta_ = 0.0;
        time_num_ = 0;

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
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Activating ...please wait...");

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

        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDrivePico::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Deactivating ...please wait...");

        // for (auto i = 0; i < hw_stop_sec_; i++)
        // {
        //     rclcpp::sleep_for(std::chrono::seconds(1));
        //     RCLCPP_INFO(
        //         rclcpp::get_logger("AutonomousWaiterSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
        // }
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffDrivePico::read(
        const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        double radius = 0.02; // radius of the wheels
        double dist_w = 0.1;  // distance between the wheels
        for (uint i = 0; i < hw_commands_.size(); i++)
        {
            // Simulate DiffBot wheels's movement as a first-order system
            // Update the joint status: this is a revolute joint without any limit.
            // Simply integrates
            hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_commands_[i];
            hw_velocities_[i] = hw_commands_[i];

            // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
            RCLCPP_INFO(
                rclcpp::get_logger("AutonomousWaiterSystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
                hw_velocities_[i], info_.joints[i].name.c_str());
            // END: This part here is for exemplary purposes - Please do not copy to your production code
        }

        // Update the free-flyer, i.e. the base notation using the classical
        // wheel differentiable kinematics
        double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
        double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
        double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
        base_x_ += base_dx * period.seconds();
        base_y_ += base_dy * period.seconds();
        base_theta_ += base_dtheta * period.seconds();

        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(
            rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",
            base_x_, base_y_, base_theta_);
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffDrivePico::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Writing...");
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Test");
        lcm::LCM lcmInstance_(MULTICAST_URL);
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Not An LCM overall library issue...");
        mbot_motor_command_t cmd;
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Success!! Continuing Writing...");
        cmd.utime = time_num_;
        time_num_++;
        cmd.trans_v = 2;
        cmd.angular_v = 2;
        lcmInstance_.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);

        for (auto i = 0u; i < hw_commands_.size(); i++)
        {
            // Simulate sending commands to the hardware
            RCLCPP_INFO(
                rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Got translational command %.5f and angular command %.5f at time %d!", cmd.trans_v, 
                cmd.angular_v, 
                ((int)cmd.utime));
        }
        RCLCPP_INFO(rclcpp::get_logger("AutonomousWaiterSystemHardware"), "Joints successfully written!");
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

    // hardware_interface::CallbackReturn DiffDrivePico::on_configure(const rclcpp_lifecycle::State &previous_state)
    // {
    //     RCLCPP_INFO(logger_, "Configuring...");
    //     RCLCPP_INFO(logger_, "Finished Configuration");
    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn DiffDrivePico::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    // {
    //     RCLCPP_INFO(logger_, "Cleaning Up...");
    //     RCLCPP_INFO(logger_, "Finished Cleaning Up");
    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }

    // hardware_interface::return_type DiffDrivePico::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    // {
    //     int fd;

    //     if ((fd = serialOpen("/dev/ttyACM0", 9600)) < 0)
    //     {
    //         fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    //         return 1;
    //     }

    //     for (;;)
    //     {
    //         putchar(serialGetchar(fd));
    //         fflush(stdout);
    //     }
    // }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_drive_pico::DiffDrivePico, hardware_interface::ActuatorInterface)