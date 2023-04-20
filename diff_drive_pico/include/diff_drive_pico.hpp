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

#include <lcm/lcm-cpp.hpp>
#include "../lcmtypes/mbot_motor_command_t.hpp"
#include "../lcmtypes/message_received_t.hpp"
#include "../lcmtypes/mbot_motor_command_t.hpp"
#include "../lcmtypes/odometry_t.hpp"
#include "../lcmtypes/mbot_encoder_t.hpp"
#include "../lcmtypes/pose_xyt_t.hpp"
// #include "lcmtypes/robot_path_t.hpp"
#include "../lcmtypes/timestamp_t.hpp"
// #include "../lcmtypes/message_received_t.hpp"
// #include "common/pose_trace.hpp"
#include "../common/lcm_config.h"
// #include "../common/timestamp.h"
#include "../common/mbot_channels.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <stdint.h>

namespace diff_drive_pico
{
    class DiffDrivePico : public hardware_interface::ActuatorInterface
    {

    public:
        DiffDrivePico()
            : logger_(rclcpp::get_logger("DiffDrivePico"))
        {
            // lcmInstance_(MULTICAST_URL);
        }

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        // hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        void handleEncoders(const lcm::ReceiveBuffer *buf, const std::string &channel, const mbot_encoder_t *encoders)
        {
            enc_time = encoders->utime;
            leftticks = encoders->leftticks;
            rightticks = encoders->rightticks;
            left_delta = encoders->left_delta;
            right_delta = encoders->right_delta;
        }

    private:
        rclcpp::Logger logger_;

        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        int first;
        int enc_time;
        int lefttick_offset;
        int righttick_offset;
        int prev_ticks_l;
        int prev_ticks_r;
        int leftticks;
        int rightticks;
        int left_delta;
        int right_delta;
    };
}