#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>


namespace diff_drive_pico
{
    class DiffDrivePico : public ActuatorInterface 
    {

    public:
        DiffDrivePico()
        : logger_(rclcpp::get_logger("DiffDrivePico"))
        {}

        CallbackReturn on_init(const HardwareInfo & hardware_info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_activate(const State & previous_state) override;

        CallbackReturn on_deactivate(const State & previous_state) override;

        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        

        // CallbackReturn on_configure(const State & previous_state) override;

        // CallbackReturn on_cleanup(const State & previous_state) override;

        // CallbackReturn on_shutdown(const State & previous_state) override;

        // CallbackReturn on_error(const State & previous_state) override;

    private:
        rclcpp::Logger logger_;

        std::chrono::time_point<std::chrono::system_clock> time_;
    }
}