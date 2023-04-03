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
    class DiffDrivePico : public hardware_interface::ActuatorInterface 
    {

    public:
        DiffDrivePico()
        : logger_(rclcpp::get_logger("DiffDrivePico"))
        {}

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        

        // hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    private:
        rclcpp::Logger logger_;
        std::chrono::time_point<std::chrono::system_clock> time_;

        // Parameters for the DiffBot simulation
        // double hw_start_sec_;
        // double hw_stop_sec_;

        // Store the command for the simulated robot
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        // Store the wheeled robot position
        double base_x_, base_y_, base_theta_;
    };
}