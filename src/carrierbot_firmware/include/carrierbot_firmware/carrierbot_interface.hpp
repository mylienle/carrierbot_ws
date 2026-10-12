#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carrierbot_msgs/msg/carrierbot_telemetry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include "carrierbot_firmware/can_node.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>

namespace carrierbot_firmware
{
    class CarrierbotInterface : public hardware_interface::SystemInterface
    {
    public:
        CarrierbotInterface();
        virtual ~CarrierbotInterface();

        hardware_interface::return_type configure(
            const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type start() override;
        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;
        hardware_interface::return_type write() override;

        std::string get_name() const override;
        hardware_interface::status get_status() const override;

    private:
        // v1 (robot_fablab) kinematics / encoder constants
        static constexpr double kWheelRadiusM = 0.1;
        static constexpr double kWheelRadiusMm = 100.0;
        static constexpr int kPulsePerRevolution = 10000;
        static constexpr double kVelocityScale = 20.0;  // same ×20 / ÷20 as v1
        static constexpr double kPi = 3.14159265358979323846;

        void handleEncoderData(const std::vector<uint8_t> &data);
        void handleRfidData(const std::vector<uint8_t> &data);
        void onRotateAngle(const std_msgs::msg::Float32::SharedPtr msg);
        void sendRotateAngle(double angle_deg);
        void publishTelemetry();
        void sendWheelVelocities();
        void sendDriveMode(int mode);
        void sendZeroCommand();
        void sendCan(uint16_t can_id, const std::vector<uint8_t> &data);
        void startPollLoop();
        void stopPollLoop();

        static int convertPulse(double velocity_mps);
        static double convertVelocityFromPulse(int pulse);

        hardware_interface::HardwareInfo info_;
        std::unique_ptr<WaveshareCAN> can_interface_;
        std::string port_;
        int baudrate_{2000000};
        int drive_mode_{1};                 // robot_fablab_ws hardware launch uses mode 1
        double poll_period_sec_{0.05};      // v1 cycle_transmit default

        std::vector<double> velocity_command_;
        std::vector<double> position_state_;
        std::vector<double> velocity_state_;
        carrierbot_msgs::msg::CarrierbotTelemetry telemetry_msg_;
        std::mutex state_mutex_;
        std::mutex can_mutex_;

        // Raw encoder pulses/s from CAN 0x011 (before ÷20 scale)
        int raw_left_pulse_s_{0};
        int raw_right_pulse_s_{0};
        float voltage_{0.0f};

        rclcpp::Node::SharedPtr ros_node_;
        rclcpp::Publisher<carrierbot_msgs::msg::CarrierbotTelemetry>::SharedPtr telemetry_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rfid_pub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rotate_sub_;
        rclcpp::Time last_run_;

        std::thread poll_thread_;
        std::atomic<bool> poll_running_{false};

        // v1 RFID tag → (full_name, short_name)
        std::map<std::vector<uint8_t>, std::pair<std::string, std::string>> rfid_database_;
    };
}
