#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>
#include <unordered_set>
#include <sched.h>
#include <sys/mman.h>

#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>


#include "boxygo_moteus_control/moteus_hardware_interface.hpp"
#include "moteus_multiplex.h"
#include "moteus_protocol.h"
#include "rclcpp/rclcpp.hpp"
#include "moteus.h"

//#define TEST_MODE

namespace moteus_hardware_interface
{
    MoteusHardwareInterface::~MoteusHardwareInterface()
    {
        on_deactivate(rclcpp_lifecycle::State());
    }

    /**
     * @brief on_init
     *
     * @param info defines the hardware interface configuration
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn MoteusHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Starting moteus_hardware_interface on_init()");

        /**
         * @brief Initialize state and command vectors
         * 
         *  Resize the vectors to the number of joints in the URDF file and set the values to NaN
         */
        hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_errors_.resize(info_.joints.size(), 0);
        hw_state_states_.resize(info_.joints.size(), 0);
        hw_state_voltages_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        hw_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kps_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kis_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        /**
         * @brief Reads the URDF file and sets the parameters and limits for each joint
         * 
         */

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
        // Set params for each joint
        hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "can_id set to: %d", hw_actuator_can_ids_.back());
        hw_actuator_axis_directions_.push_back(std::stoi(joint.parameters.at("axis_direction")));
        hw_actuator_gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
        }

        /**
         * @brief Ensure that all CAN IDs are unique
         * 
         * this is a more stringent requirement, since you could have the same IDs on different channels,
         * but it causes problems when reading the feedback from the actuators
         */
        std::unordered_set<int> seenIDs;
        for (int id : hw_actuator_can_ids_) {
            if (!seenIDs.insert(id).second) {
                RCLCPP_ERROR(rclcpp::get_logger("MoteusHardwareInterface"), "Failed to start: duplicate CAN IDs present");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // onInit() -> Configure realtime scheduling
        {
            int realtime_cpu1 = 0; // first core
            int realtime_cpu2 = 1; // second core
            cpu_set_t cpuset = {};
            CPU_ZERO(&cpuset);
            CPU_SET(realtime_cpu1, &cpuset); // set first core
            CPU_SET(realtime_cpu2, &cpuset); // set second core

            const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
            if (r < 0)
            {
                throw std::runtime_error("Error setting CPU affinity");
            }

            std::cout << "Affinity set to CPUs " << realtime_cpu1 << " and " << realtime_cpu2 << "\n";
        }
        {
            struct sched_param params = {};
            params.sched_priority = 10;
            const int r = ::sched_setscheduler(0, SCHED_RR, &params);
            if (r < 0)
            {
                throw std::runtime_error("Error setting realtime scheduler");
            }
        }
        {
            const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
            if (r < 0)
            {
                throw std::runtime_error("Error locking memory");
            }
        }

#ifndef TEST_MODE
        for (unsigned long i = 0; i < info_.joints.size(); i++)
        {
            mjbots::moteus::Controller::Options options;

            options.id = hw_actuator_can_ids_[i];
            options.position_format.position = mjbots::moteus::kFloat;
            options.position_format.velocity = mjbots::moteus::kFloat;
            options.position_format.feedforward_torque = mjbots::moteus::kFloat;
            options.position_format.kp_scale = mjbots::moteus::kInt8;
            options.position_format.kd_scale = mjbots::moteus::kInt8;    

            controllers_[i] = std::make_shared<mjbots::moteus::Controller>(options);
        }

        // Stop everything to clear faults.
        for (const auto& pair : controllers_) {
            RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Sending Actuator at controller %d id: %d to idle...", pair.first, pair.second->options().id);
            int fault = pair.second->SetStop()->values.fault;
            //int mode = pair.second.get()

            if (fault != 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Controller with id: %d reported fault: %d", pair.second->options().id, fault);

                return hardware_interface::CallbackReturn::ERROR;
            }

            //pair.second->DiagnosticWrite("tel stop\n");
            //pair.second->DiagnosticFlush();
            
            // std::ostringstream ostr;
            // ostr << "conf set servopos.position_min " << (hw_actuator_position_mins_[pair.first] / (2.0 * M_PI * hw_actuator_gear_ratios_[pair.first]));
            // pair.second->DiagnosticCommand(ostr.str());
            // ostr.clear();
            // ostr << "conf set servopos.position_max " << (hw_actuator_position_maxs_[pair.first] / (2.0 * M_PI * hw_actuator_gear_ratios_[pair.first]));
            // pair.second->DiagnosticCommand(ostr.str());
            // ostr.clear();
        }
        
        // todo:
        // set moteus parameters via debug protocol
        // as shown in: https://github.com/mjbots/moteus/blob/main/lib/cpp/examples/diagnostic_protocol.cc
        // i.e. min max endpoints, max vel torques etc, pid parameters
        // maybe omit safety critical params to be only changed on moteus directly via tview

        for (const auto& pair : controllers_) {
            auto ctrl = pair.second;

            // exact = 0 → aktualna mechaniczna pozycja staje się zerem
            ctrl->DiagnosticWrite("d exact 0.0\n");
            ctrl->DiagnosticFlush();

            RCLCPP_INFO(
                rclcpp::get_logger("MoteusHardwareInterface"),
                "Temporary zero offset set for actuator id=%d",
                ctrl->options().id
            );
        }

        // We did not specify a transport so the default one was used when
        // constructing our Controller instances.  We need to get access to
        // that in order to send commands simultaneously to multiple servos.
        transport_ = mjbots::moteus::Controller::MakeSingletonTransport({});
        


#endif


        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "MoteusHardwareInterface successfully initialized!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // TODO: Update the state interfaces . Do we want to add Temperature, Voltage?
    std::vector<hardware_interface::StateInterface> MoteusHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Add joint state interfaces
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
        }

        return state_interfaces;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset values always when configuring hardware
        for (uint i = 0; i < hw_state_positions_.size(); i++)
        {
            hw_state_positions_[i] = 0;
            hw_state_velocities_[i] = 0;
            hw_state_efforts_[i] = 0;
            hw_command_positions_[i]  = std::numeric_limits<double>::quiet_NaN();
            hw_command_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
            hw_command_efforts_[i]    = std::numeric_limits<double>::quiet_NaN();
            hw_command_kps_[i] = 0;
            hw_command_kds_[i] = 0;
            hw_command_kis_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> MoteusHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kp", &hw_command_kps_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kd", &hw_command_kds_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "ki", &hw_command_kis_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: implement
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
#ifndef TEST_MODE
        for (const auto& pair : controllers_) {
            RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "Sending Actuator at Joint %d to idle...", pair.second->options().id);
            pair.second->SetStop();
        }
#endif

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MoteusHardwareInterface"), "on_error");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MoteusHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        (void)period;

        if (servo_data_.empty()) {
            RCLCPP_DEBUG(
                rclcpp::get_logger("MoteusHardwareInterface"),
                "read(): servo_data_ empty, keeping previous states");
            return hardware_interface::return_type::OK;
        }

        for (const auto &pair : servo_data_) {
            unsigned int canid = pair.first;
            const auto &r = pair.second;

            std::size_t index = 0;
            bool found = false;
            for (std::size_t i = 0; i < hw_actuator_can_ids_.size(); ++i) {
                if (static_cast<unsigned int>(hw_actuator_can_ids_[i]) == canid) {
                    index = i;
                    found = true;
                    break;
                }
            }

            if (!found) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoteusHardwareInterface"),
                    "read(): got reply from unknown CAN id %u", canid);
                continue;
            }

            const double dir = static_cast<double>(hw_actuator_axis_directions_[index]);

            hw_state_efforts_[index] = r.torque;

            const double wheel_pos_rad =
                dir * r.position * 2.0 * M_PI;
            const double wheel_vel_rad =
                dir * r.velocity * 2.0 * M_PI;

            hw_state_positions_[index]  = wheel_pos_rad;
            hw_state_velocities_[index] = wheel_vel_rad;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MoteusHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("MoteusHardwareInterface"),
            "%f %f %f | %f %f %f | %f %f %f",
            hw_command_efforts_[0], hw_command_efforts_[1], hw_command_efforts_[2],
            hw_command_velocities_[0], hw_command_velocities_[1], hw_command_velocities_[2],
            hw_command_positions_[0], hw_command_positions_[1], hw_command_positions_[2]);

        command_frames_.clear();
        replies_.clear();

    #ifndef TEST_MODE
        for (auto i = 0u; i < hw_command_velocities_.size(); i++)
        {   
            mjbots::moteus::PositionMode::Command position_command;

            if (!std::isnan(hw_command_velocities_[i]))
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("MoteusHardwareInterface"),
                    "vel %d = %f", i, hw_command_velocities_[i]);

                const double wheel_vel_rad_s = hw_command_velocities_[i];
                const double dir = static_cast<double>(hw_actuator_axis_directions_[i]);

                const double motor_rev_s =
                    wheel_vel_rad_s / (dir * 2.0 * M_PI);

                position_command.position = std::numeric_limits<float>::quiet_NaN();
                position_command.velocity = static_cast<float>(motor_rev_s);
            }
            else
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("MoteusHardwareInterface"),
                    "no cmd %d", i);

                position_command.position = std::numeric_limits<float>::quiet_NaN();
                position_command.velocity = 0.0f;
            }

            command_frames_.push_back(controllers_[i]->MakePosition(position_command));
        }
            
        transport_->BlockingCycle(&command_frames_[0], command_frames_.size(), &replies_);

        servo_data_.clear();

        for (const auto& frame : replies_) {
            servo_data_[frame.source] = mjbots::moteus::Query::Parse(frame.data, frame.size);
        }
    #endif    

    #ifdef TEST_MODE
        for (auto i = 0u; i < hw_command_positions_.size(); i++)
        {
            hw_state_positions_[i]  = hw_command_positions_[i];
            hw_state_velocities_[i] = hw_command_velocities_[i];
            hw_state_efforts_[i]    = hw_command_efforts_[i];
        }
    #endif

        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn MoteusHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS; 
    }

    int MoteusHardwareInterface::double_to_uint(double x, double x_min, double x_max,
                                                int bits)
    {
        /// Converts a double to an unsigned int, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
    }

    double MoteusHardwareInterface::uint_to_double(int x_int, double x_min, double x_max,
                                                   int bits)
    {
        /// converts unsigned int to double, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }

    double MoteusHardwareInterface::wrap_angle(double angle, double angle_min, double angle_max)
    {
        /// Wraps an angle to the range [angle_min, angle_max] ///
        double span = angle_max - angle_min;
        return angle - span * floor((angle - angle_min) / span);
    }

    double MoteusHardwareInterface::unwrap_angle(double angle, double prev_angle, double angle_min, double angle_max)
    {
        /// Deals with wrap-around for a continuously changing angle ///
        double span = angle_max - angle_min;
        double prev_angle_wrapped = wrap_angle(prev_angle, angle_min, angle_max);
        double d_angle = angle - prev_angle_wrapped;
        if (d_angle > span / 2)
        {
            d_angle -= span;
        }
        else if (d_angle < -span / 2)
        {
            d_angle += span;
        }
        return prev_angle + d_angle;
    }

    void MoteusHardwareInterface::update_state_interfaces()
    {
        // Update the state interfaces
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            // hw_state_positions_[i] = hw_actuators_[i]->getPosition();
            // hw_state_velocities_[i] = hw_actuators_[i]->getVelocity();
            // hw_state_efforts_[i] = hw_actuators_[i]->getEffort();
            // hw_state_voltages_[i] = hw_actuators_[i]->getVoltage();
            // hw_state_temperatures_[i] = hw_actuators_[i]->getTemperature();
            // hw_state_errors_[i] = hw_actuators_[i]->getError();
            // hw_state_states_[i] = hw_actuators_[i]->getState();
        }
    }

} // namespace pi3hat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    moteus_hardware_interface::MoteusHardwareInterface, hardware_interface::SystemInterface)



int main()
{
 return 0;
}