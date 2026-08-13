#ifndef CAN_NODE_HPP_
#define CAN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <vector>
#include <functional>
#include <thread>
#include <chrono>
#include <iostream>
#include <cstring>
#include <atomic>
#include <memory>
#include <stdexcept>

class WaveshareCAN {
public:
    using Callback = std::function<void(uint16_t, const std::vector<uint8_t>&)>;

    WaveshareCAN(const std::string& port = "/dev/usbcan", uint32_t baudrate = 2000000, float timeout = 1.0)
        : port_(port), baudrate_(baudrate), timeout_(timeout), fd_(-1), rx_running_(false) {}

    ~WaveshareCAN() {
        close();
    }

    void open() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            throw std::runtime_error("Failed to open serial port: " + port_ + " (" + std::strerror(errno) + ")");
        }

        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B2000000);
        cfsetospeed(&options, B2000000);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        options.c_cc[VTIME] = static_cast<int>(timeout_ * 10); // Timeout in tenths of a second
        options.c_cc[VMIN] = 0; // Minimum bytes to read
        tcsetattr(fd_, TCSANOW, &options);

        // Clear input buffer
        tcflush(fd_, TCIFLUSH);

        std::cout << "Serial port opened: " << port_ << " @ " << baudrate_ << " baud\n";
    }

    void close() {
        rx_running_ = false;

        // Đóng file descriptor trước để buộc thread read() thoát ra.
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }

        // Luôn join nếu thread đã được tạo, kể cả khi nó đã tự thoát.
        if (rx_thread_ && rx_thread_->joinable()) {
            rx_thread_->join();
        }

        if (rx_thread_) {
            rx_thread_.reset();
        }
    }

    void send(uint16_t can_id, const std::vector<uint8_t>& data) {
        if (fd_ == -1) {
            throw std::runtime_error("Serial port is not open. Call open() first.");
        }

        std::vector<uint8_t> frame;
        frame.push_back(0xAA); // Start byte
        frame.push_back(0xC8); // CMD byte
        frame.push_back(can_id & 0xFF);        // IDL
        frame.push_back((can_id >> 8) & 0xFF); // IDH

        std::vector<uint8_t> padded_data = data;
        if (padded_data.size() > 8) {
            padded_data.resize(8);
        } else {
            padded_data.resize(8, 0);
        }
        frame.insert(frame.end(), padded_data.begin(), padded_data.end());
        frame.push_back(0x55); // Tail byte

        if (write(fd_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
            throw std::runtime_error("Failed to write full frame");
        }
    }

    std::pair<uint16_t, std::vector<uint8_t>> receive()
    {
        if (fd_ == -1)
        {
            throw std::runtime_error("Serial port is not open. Call open() first.");
        }

        uint8_t b;
        // Wait for start byte (0xAA)
        while (true)
        {
            if (read_exact(&b, 1) && b == 0xAA)
            {
                break;
            }
        }

        // Frame format used by the v1 Waveshare adapter:
        // AA C8 IDL IDH DATA[8] 55
        uint8_t cmd;
        if (!read_exact(&cmd, 1))
        {
            throw std::runtime_error("Failed to read CMD byte");
        }
        if (cmd != 0xC8)
        {
            throw std::runtime_error("Invalid CAN command byte");
        }

        uint8_t idl;
        uint8_t idh;
        if (!read_exact(&idl, 1) || !read_exact(&idh, 1))
        {
            throw std::runtime_error("Failed to read CAN ID");
        }

        // Read 8 data bytes
        std::vector<uint8_t> data(8);
        if (!read_exact(data.data(), 8))
        {
            throw std::runtime_error("Failed to read data bytes");
        }

        // Read tail byte
        uint8_t tail;
        if (!read_exact(&tail, 1))
        {
            throw std::runtime_error("Failed to read tail byte");
        }
        if (tail != 0x55)
        {
            throw std::runtime_error("Invalid CAN frame tail");
        }

        uint16_t can_id = static_cast<uint16_t>(idl) |
                          (static_cast<uint16_t>(idh) << 8);

        // std::cout << "Received: ID=0x" << std::hex << can_id << " Data=";
        // for (uint8_t b : data)
        // {
        //     std::cout << std::hex << (int)b << " ";
        // }
        // std::cout << std::dec << "\n";

        return {can_id, data};
    }

    void start_receive_loop(Callback callback) {
        if (rx_thread_ && rx_thread_->joinable()) {
            std::cout << "🔄 Receive loop already running.\n";
            return;
        }

        rx_running_ = true;
        rx_thread_ = std::make_unique<std::thread>(
            &WaveshareCAN::receive_worker, this, callback
        );
        std::cout << "🔄 Receive loop started (thread)\n";
    }

private:
    bool read_exact(uint8_t* buffer, size_t len) {
        size_t bytes_read = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (bytes_read < len) {
            ssize_t n = read(fd_, buffer + bytes_read, len - bytes_read);
            if (n > 0) {
                bytes_read += n;
            } else if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout or no data available
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                if (elapsed >= timeout_ * 1000) {
                    return false; // Timeout
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else if (errno == EBADF) {
                // Bad file descriptor - likely closed during shutdown
                throw std::runtime_error("Bad file descriptor");
            } else {
                std::cerr << "Read error: " << std::strerror(errno) << "\n";
                return false;
            }
        }
        return true;
    }

    void receive_worker(Callback callback) {
        while (rx_running_) {
            try {
                auto result = receive();
                auto can_id = result.first;
                auto data   = result.second;
                if (rx_running_) {  // Check flag before callback
                    callback(can_id, data);
                }
            } catch (const std::exception& e) {
                if (rx_running_) {  // Only log if still running
                    // Check if it's a "Bad file descriptor" error (happens during shutdown)
                    if (std::string(e.what()).find("Bad file descriptor") != std::string::npos) {
                        // This is expected during shutdown, exit gracefully
                        rx_running_ = false;
                        break;
                    }
                    std::cerr << "Error in receive loop: " << e.what() << ". Retrying...\n";
                    // Clear input buffer to resynchronize
                    if (fd_ != -1) {
                        tcflush(fd_, TCIFLUSH);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }
        std::cout << "🛑 Receive worker thread stopped\n";
    }

    std::string port_;
    uint32_t baudrate_;
    float timeout_;
    int fd_;
    std::atomic<bool> rx_running_;
    std::unique_ptr<std::thread> rx_thread_;
};


#endif  // CAN_NODE_HPP
