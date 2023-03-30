#include "hardware_interface/actuator_interface.hpp"

namespace diff_drive_pico
{
    class DiffDrivePico : public hardware_interface::ActuatorInterface 
    {

    public:
        DiffDrivePico(){}

        CallbackReturn on_configure(const State & previous_state);

        CallbackReturn on_cleanup(const State & previous_state);

        CallbackReturn on_shutdown(const State & previous_state);

        CallbackReturn on_activate(const State & previous_state);

        CallbackReturn on_deactivate(const State & previous_state);

        CallbackReturn on_error(const State & previous_state);

        // CallbackReturn on_init(const HardwareInfo & hardware_info);

        std::vector<StateInterface> export_state_interfaces();

        std::vector<CommandInterface> export_command_interfaces();

        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);

        return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);
    }
}