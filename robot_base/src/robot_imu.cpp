#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Standard C++ and POSIX headers for serial communication
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

// For convenience
using std::string;
using std::cout;
using std::endl;

// Global serial file descriptor (could also be encapsulated in the class)
int serial_fd = -1;

/**
 * @brief Opens the serial port and configures it.
 * 
 * @param port The serial port device (e.g., "/dev/IMU")
 * @param baud_rate The desired baud rate (currently unused in this example since we hardcode B230400)
 * @return int 0 on success, -1 on failure
 */
int open_serial(const string &port, int baud_rate)
{
  serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd == -1)
  {
    cout << "串口打开失败！" << endl;
    return -1;
  }
  else
  {
    cout << "串口打开成功！" << endl;
  }
  struct termios options;
  tcgetattr(serial_fd, &options);
  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_iflag |= IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 1;
  cfsetospeed(&options, B230400);
  cfsetispeed(&options, B230400);
  tcflush(serial_fd, TCIFLUSH);
  if (tcsetattr(serial_fd, TCSANOW, &options) != 0)
  {
    cout << "imu串口设置失败！" << endl;
    return -1;
  }
  else
  {
    cout << "imu串口设置成功！" << endl;
  }
  return 0;
}

class RobotImuNode : public rclcpp::Node
{
public:
  RobotImuNode()
  : Node("imu_pub")
  {
    // Create a publisher for sensor_msgs::msg::Imu on the "imu" topic.
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    // Open the serial port.
    if (open_serial("/dev/IMU", 230400) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the IMU serial port!");
      rclcpp::shutdown();
      return;
    }

    // Set up the fixed header fields for the IMU message.
    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
  }

  /**
   * @brief The main loop that reads data from the serial port and publishes IMU messages.
   */
  void run()
  {
    // Buffer for reading serial data.
    unsigned char buffer[16];
    // Temporary arrays for sensor data.
    short ax[6] = {0};
    short gx[6] = {0};
    // Loop until ROS 2 is shutdown.
    while (rclcpp::ok())
    {
      // Read one byte from the serial port.
      if (read(serial_fd, buffer, 1) <= 0) {
        continue;
      }
      if (buffer[0] == 0x55)
      {
        // Read the second byte.
        if (read(serial_fd, buffer + 1, 1) <= 0) {
          continue;
        }
        // If the second byte indicates accelerometer data.
        if (buffer[1] == 0x51)
        {
          // Read the next 9 bytes into ax.
          if (read(serial_fd, ax, 9) <= 0) {
            continue;
          }
          // Process accelerometer data.
          imu_msg_.linear_acceleration.x = ax[0] / 32768.0 * 16.0 * 9.80665;
          imu_msg_.linear_acceleration.y = ax[1] / 32768.0 * 16.0 * 9.80665;
          imu_msg_.linear_acceleration.z = ax[2] / 32768.0 * 16.0 * 9.80665;
          // Adjust sign if needed.
          imu_msg_.linear_acceleration.y = -imu_msg_.linear_acceleration.y;
          // Set the covariance for the linear acceleration.
          imu_msg_.linear_acceleration_covariance = {1e1, 0, 0,
                                                      0, 1e-1, 0,
                                                      0, 0, 1e-1};
          // (Optionally, you might choose to publish here or wait for gyro data.)
        }
        // If the second byte indicates gyroscope data.
        else if (buffer[1] == 0x52)
        {
          if (read(serial_fd, gx, 9) <= 0) {
            continue;
          }
          imu_msg_.angular_velocity.x = gx[0] / 32768.0 * 2000 * (3.1415926535 / 180.0);
          imu_msg_.angular_velocity.y = gx[1] / 32768.0 * 2000 * (3.1415926535 / 180.0);
          imu_msg_.angular_velocity.z = gx[2] / 32768.0 * 2000 * (3.1415926535 / 180.0);
          // Set the covariance for the angular velocity.
          imu_msg_.angular_velocity_covariance = {1e6, 0, 0,
                                                    0, 1e6, 0,
                                                    0, 0, 1e-6};
          // Update the timestamp in the header.
          imu_msg_.header.stamp = this->now();
          // Publish the complete IMU message.
          imu_pub_->publish(imu_msg_);
        }
        // (Additional data types, such as magnetic field or orientation, could be processed here.)
      }
      // Optionally sleep or yield if needed.
    }
    close(serial_fd);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotImuNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
