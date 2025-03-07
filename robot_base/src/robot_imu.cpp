// 生成ros发布imu主题的代码
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// 生成c++串口初始化和读取代码
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

using namespace std;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher() : Node("imu_pub"), serial_fd_(-1)
  {
    // 创建发布者
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    
    // 打开串口
    if (open_serial("/dev/IMU", 230400) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      return;
    }
    
    // 初始化IMU消息
    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
    
    // 创建定时器
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ImuPublisher::timer_callback, this));
  }
  
  ~ImuPublisher()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  int open_serial(string port, int baud_rate)
  {
    // 忽略baud_rate参数，使用固定的波特率230400
    (void)baud_rate; // 显式忽略参数，避免警告
    
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY); // 打开串口
    if (serial_fd_ == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "串口打开失败！");
      return -1;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "串口打开成功！");
    }
    struct termios options; // 串口配置结构体
    tcgetattr(serial_fd_, &options);
    options.c_cflag |= CLOCAL; // 忽略调制解调器状态行
    options.c_cflag |= CREAD;  // 启用接收器
    options.c_cflag &= ~CSIZE; // 字符长度掩码。取值为CS5, CS6, CS7或CS8
    options.c_cflag |= CS8;    // 8位数据位
    options.c_cflag &= ~PARENB; // 校验位
    options.c_cflag &= ~CSTOPB; // 停止位
    options.c_iflag |= IGNPAR;  // 忽略帧错误和奇偶校验错
    options.c_oflag = 0;        // 输出模式
    options.c_lflag = 0;        // 不激活终端模式
    options.c_cc[VTIME] = 0; // 读取一个字符等待1*(1/10)s
    options.c_cc[VMIN] = 1;  // 读取字符的最少个数为1
    cfsetospeed(&options, B230400); // 设置波特率为230400
    cfsetispeed(&options, B230400);
    tcflush(serial_fd_, TCIFLUSH); // 清空输入缓存区
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) // TCSANOW：不等数据传输完毕就立即改变属性
    {
      RCLCPP_ERROR(this->get_logger(), "imu串口设置失败！");
      return -1;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "imu串口设置成功！");
    }
    return 0;
  }
  
  void timer_callback()
  {
    unsigned char buffer[16];
    short ax[6] = {0};
    short gx[6] = {0};
    
    // 读取串口数据
    int bytes_read = read(serial_fd_, buffer, 1);
    if (bytes_read > 0 && buffer[0] == 0x55)
    {
      read(serial_fd_, (buffer + 1), 1);
      if (buffer[1] == 0x51)
      {
        read(serial_fd_, ax, 9);
        imu_msg_.linear_acceleration.x = ax[0] / 32768.0 * 16.0 * 9.80665;
        imu_msg_.linear_acceleration.y = ax[1] / 32768.0 * 16.0 * 9.80665;
        imu_msg_.linear_acceleration.z = ax[2] / 32768.0 * 16.0 * 9.80665;
        
        // 旋转180度
        imu_msg_.linear_acceleration.y = -imu_msg_.linear_acceleration.y;
        
        // 设置协方差
        imu_msg_.linear_acceleration_covariance = {
          1e1, 0, 0,
          0, 1e-1, 0,
          0, 0, 1e-1
        };
      }
      else if (buffer[1] == 0x52)
      {
        read(serial_fd_, gx, 9);
        imu_msg_.angular_velocity.x = gx[0] / 32768.0 * 2000 * (3.1415926535 / 180.0);
        imu_msg_.angular_velocity.y = gx[1] / 32768.0 * 2000 * (3.1415926535 / 180.0);
        imu_msg_.angular_velocity.z = gx[2] / 32768.0 * 2000 * (3.1415926535 / 180.0);
        
        // 设置协方差
        imu_msg_.angular_velocity_covariance = {
          1e6, 0, 0,
          0, 1e6, 0,
          0, 0, 1e-6
        };
        
        // 设置时间戳并发布消息
        imu_msg_.header.stamp = this->now();
        imu_publisher_->publish(imu_msg_);
      }
    }
  }
  
  int serial_fd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
