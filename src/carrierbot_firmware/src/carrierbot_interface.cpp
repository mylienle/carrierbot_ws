#include "carrierbot_firmware/carrierbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cstring>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

namespace carrierbot_firmware
{
    CarrierbotInterface::CarrierbotInterface() = default;

    CarrierbotInterface::~CarrierbotInterface()
    {
        stopPollLoop();
        if (can_interface_ != nullptr)
        {
            try
            {
                sendZeroCommand();
                can_interface_->close();
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                    "Failed to close CAN port: " << e.what());
            }
            can_interface_.reset();
        }
    }

    int CarrierbotInterface::convertPulse(double velocity_mps)
    {
        // v1 ConvertPulse: PPR * v(m/s) * 1000 / (2*pi*r_mm)
        return static_cast<int>(
            kPulsePerRevolution * velocity_mps * 1000.0 / (2.0 * kPi * kWheelRadiusMm));
    }

    double CarrierbotInterface::convertVelocityFromPulse(int pulse)
    {
        const double circumference_mm = 2.0 * kPi * kWheelRadiusMm;
        return (static_cast<double>(pulse) * circumference_mm) /
               (static_cast<double>(kPulsePerRevolution) * 1000.0);
    }

    hardware_interface::return_type CarrierbotInterface::configure(
        const hardware_interface::HardwareInfo &info)
    {
        info_ = info;

        try
        {
            port_ = info_.hardware_parameters.at("port");
            baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CarrierbotInterface"), "No CAN port specified!");
            return hardware_interface::return_type::ERROR;
        }

        if (info_.hardware_parameters.count("drive_mode"))
        {
            drive_mode_ = std::stoi(info_.hardware_parameters.at("drive_mode"));
        }
        if (info_.hardware_parameters.count("poll_period"))
        {
            poll_period_sec_ = std::stod(info_.hardware_parameters.at("poll_period"));
        }

        // Same RFID table as v1 can_node.cpp
        rfid_database_ = {
            {{0xd2, 0x0f, 0x49, 0x2e, 0xba, 0x55, 0xaa, 0xc8}, {"HOAI PHU", "Phu"}},
            {{0xd2, 0xb1, 0x3d, 0x05, 0x5b, 0x55, 0xaa, 0xc8}, {"MINH KY", "Ky"}},
            {{0xfa, 0xdc, 0x02, 0xcd, 0xe9, 0x55, 0xaa, 0xc8}, {"QUANG DUY", "Duy"}},
            {{0xef, 0xa8, 0x98, 0x1e, 0xc1, 0x55, 0xaa, 0xc8}, {"CHI THIEN", "Thien"}},
            {{0xb6, 0x87, 0x13, 0x2b, 0x09, 0x55, 0xaa, 0xc8}, {"VAN LOI", "Loi"}},
            {{0xc2, 0xbf, 0xb0, 0x2e, 0xe3, 0x55, 0xaa, 0xc8}, {"BACH THU", "Thu"}},
            {{0xd2, 0xb8, 0x3d, 0x04, 0x5b, 0x55, 0xaa, 0xc8}, {"DINH HUY", "Huy"}},
        };

        can_interface_ = std::make_unique<WaveshareCAN>(port_, baudrate_, 2.0);
        ros_node_ = std::make_shared<rclcpp::Node>("carrierbot_interface");
        telemetry_pub_ = ros_node_->create_publisher<carrierbot_msgs::msg::CarrierbotTelemetry>(
            "/carrierbot/telemetry", 10);
        rfid_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
            "/carrierbot/rfid", 10);
        rotate_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32>(
            "/carrierbot/rotate_angle", 10,
            std::bind(&CarrierbotInterface::onRotateAngle, this, std::placeholders::_1));

        velocity_command_.assign(info_.joints.size(), 0.0);
        position_state_.assign(info_.joints.size(), 0.0);
        velocity_state_.assign(info_.joints.size(), 0.0);

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterface"),
            "Configured v1 hardware: port=%s baud=%d mode=%d poll=%.3fs R=%.3fm",
            port_.c_str(), baudrate_, drive_mode_, poll_period_sec_, kWheelRadiusM);

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface>
    CarrierbotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]);
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    CarrierbotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]);
        }
        return command_interfaces;
    }

    void CarrierbotInterface::sendCan(uint16_t can_id, const std::vector<uint8_t> &data)
    {
        if (!can_interface_)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(can_mutex_);
        can_interface_->send(can_id, data);
    }

    void CarrierbotInterface::sendDriveMode(int mode)
    {
        std::vector<uint8_t> data(8, 0);
        data[0] = static_cast<uint8_t>(mode);
        sendCan(0x020, data);
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"), "Sent drive mode %d (CAN 0x020)", mode);
    }

    void CarrierbotInterface::sendZeroCommand()
    {
        std::vector<uint8_t> data(8, 0);
        sendCan(0x030, data);
    }

    void CarrierbotInterface::sendRotateAngle(double angle_deg)
    {
        // robot_fablab_ws/src/MQTT/xoay_subscriber.py sends angle*100
        // as signed int16, big-endian, in the first two CAN data bytes.
        const double scaled = angle_deg * 100.0;
        const double clamped = std::max(
            static_cast<double>(std::numeric_limits<int16_t>::min()),
            std::min(scaled, static_cast<double>(std::numeric_limits<int16_t>::max())));
        const int16_t angle_scaled = static_cast<int16_t>(clamped);
        const uint16_t encoded = static_cast<uint16_t>(angle_scaled);

        std::vector<uint8_t> data(8, 0);
        data[0] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
        data[1] = static_cast<uint8_t>(encoded & 0xFF);
        sendCan(0x040, data);

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterface"),
            "Sent rotate angle %.2f (scaled=%d) via CAN 0x040",
            angle_deg, static_cast<int>(angle_scaled));
    }

    void CarrierbotInterface::onRotateAngle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        try
        {
            sendRotateAngle(msg->data);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("CarrierbotInterface"),
                "Failed to send rotate CAN 0x040: " << e.what());
        }
    }

    void CarrierbotInterface::startPollLoop()
    {
        if (poll_running_)
        {
            return;
        }
        poll_running_ = true;
        poll_thread_ = std::thread([this]() {
            using namespace std::chrono_literals;
            while (poll_running_)
            {
                try
                {
                    sendCan(0x050, {1, 0, 0, 0, 0, 0, 0, 0});
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN_STREAM(
                        rclcpp::get_logger("CarrierbotInterface"),
                        "CAN poll 0x050 failed: " << e.what());
                }
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(poll_period_sec_));
            }
        });
    }

    void CarrierbotInterface::stopPollLoop()
    {
        poll_running_ = false;
        if (poll_thread_.joinable())
        {
            poll_thread_.join();
        }
    }

    hardware_interface::return_type CarrierbotInterface::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"), "Starting v1 robot hardware");
        if (!can_interface_)
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            can_interface_->open();
            auto callback = [this](uint16_t can_id, const std::vector<uint8_t> &data)
            {
                if (can_id == 0x011)
                {
                    handleEncoderData(data);
                }
                else if (can_id == 0x019)
                {
                    handleRfidData(data);
                }
            };
            can_interface_->start_receive_loop(callback);

            // Match robot_fablab_ws launch: send the configured hardware drive mode.
            sendDriveMode(drive_mode_);
            startPollLoop();
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Cannot open CAN: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CarrierbotInterface::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"), "Stopping robot hardware");
        stopPollLoop();
        try
        {
            if (can_interface_)
            {
                sendZeroCommand();
                can_interface_->close();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Failed to close CAN port: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    void CarrierbotInterface::handleEncoderData(const std::vector<uint8_t> &data)
    {
        if (data.size() < 8)
        {
            return;
        }

        int left_pulse = 0;
        int right_pulse = 0;
        std::memcpy(&left_pulse, &data[0], sizeof(int));
        std::memcpy(&right_pulse, &data[4], sizeof(int));

        std::lock_guard<std::mutex> lock(state_mutex_);
        raw_left_pulse_s_ = left_pulse;
        raw_right_pulse_s_ = right_pulse;
    }

    void CarrierbotInterface::handleRfidData(const std::vector<uint8_t> &data)
    {
        auto it = rfid_database_.find(data);
        if (it == rfid_database_.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("CarrierbotInterface"), "Unknown RFID tag");
            return;
        }

        const std::string &full_name = it->second.first;
        const std::string &short_name = it->second.second;

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time), "%H:%M:%S");
        const std::string timestamp = ss.str();

        // Payload matches v1 name_publisher.py fields
        std_msgs::msg::String msg;
        msg.data = "{\"message\":\"" + short_name +
                   "\",\"user\":\"" + full_name +
                   "\",\"time\":\"" + timestamp + "\"}";

        if (rfid_pub_)
        {
            rfid_pub_->publish(msg);
        }

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterface"),
            "RFID detected: %s (%s) at %s",
            full_name.c_str(), short_name.c_str(), timestamp.c_str());
    }

    hardware_interface::return_type CarrierbotInterface::read()
    {
        // Process rotate MQTT→ROS subscriptions without a dedicated spin thread
        if (ros_node_)
        {
            rclcpp::spin_some(ros_node_);
        }

        rclcpp::Time now = rclcpp::Clock().now();
        if (last_run_.nanoseconds() == 0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        double dt = (now - last_run_).seconds();
        if (dt <= 0.0 || dt > 1.0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        int left_pulse = 0;
        int right_pulse = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            left_pulse = raw_left_pulse_s_;
            right_pulse = raw_right_pulse_s_;
        }

        // v1 updateWheelOdometry scale: v_l = -left/20, v_r = right/20 (m/s)
        const double left_mps = -convertVelocityFromPulse(left_pulse) / kVelocityScale;
        const double right_mps = convertVelocityFromPulse(right_pulse) / kVelocityScale;

        const double left_rad_s = left_mps / kWheelRadiusM;
        const double right_rad_s = right_mps / kWheelRadiusM;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Joint order in ros2_control.xacro: [0]=right, [1]=left
            velocity_state_[0] = right_rad_s;
            velocity_state_[1] = left_rad_s;
            telemetry_msg_.left_rps = static_cast<float>(left_rad_s / (2.0 * kPi));
            telemetry_msg_.right_rps = static_cast<float>(right_rad_s / (2.0 * kPi));
            telemetry_msg_.left_velocity = static_cast<float>(left_mps);
            telemetry_msg_.right_velocity = static_cast<float>(right_mps);
        }

        position_state_[0] += velocity_state_[0] * dt;
        position_state_[1] += velocity_state_[1] * dt;
        publishTelemetry();

        last_run_ = now;
        return hardware_interface::return_type::OK;
    }

    void CarrierbotInterface::sendWheelVelocities()
    {
        if (!can_interface_ || velocity_command_.size() < 2)
        {
            return;
        }

        double right_cmd_rad_s = 0.0;
        double left_cmd_rad_s = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            right_cmd_rad_s = velocity_command_[0];
            left_cmd_rad_s = velocity_command_[1];
        }

        // rad/s -> m/s, then apply v1 ×20 before pulse conversion
        const double left_mps = left_cmd_rad_s * kWheelRadiusM;
        const double right_mps = right_cmd_rad_s * kWheelRadiusM;
        const int left_pulse = convertPulse(left_mps * kVelocityScale);
        const int right_pulse = convertPulse(right_mps * kVelocityScale);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            telemetry_msg_.left_velocity = static_cast<float>(left_mps);
            telemetry_msg_.right_velocity = static_cast<float>(right_mps);
        }
        publishTelemetry();

        uint8_t data[8];
        std::memcpy(data, &left_pulse, sizeof(int));
        std::memcpy(data + 4, &right_pulse, sizeof(int));
        sendCan(0x030, std::vector<uint8_t>(data, data + 8));
    }

    hardware_interface::return_type CarrierbotInterface::write()
    {
        try
        {
            sendWheelVelocities();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Failed to send CAN command: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    std::string CarrierbotInterface::get_name() const
    {
        return info_.name;
    }

    hardware_interface::status CarrierbotInterface::get_status() const
    {
        return hardware_interface::status::STARTED;
    }

    void CarrierbotInterface::publishTelemetry()
    {
        if (!telemetry_pub_)
        {
            return;
        }
        telemetry_pub_->publish(telemetry_msg_);
    }

}  // namespace carrierbot_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(carrierbot_firmware::CarrierbotInterface,
                       hardware_interface::SystemInterface)
