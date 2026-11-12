#pragma once

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

class CanReadAidinSensor
{
public:
  CanReadAidinSensor(
    rclcpp::Node::SharedPtr node,
    const std::string & can_interface,
    double sampling_period_seconds);
  ~CanReadAidinSensor();

  void run();

private:
  static constexpr canid_t DEVICE_ID = 0x01;
  static constexpr unsigned char FIELD_BIAS = 0x02;
  static constexpr unsigned char FIELD_CONTINUOUS_MODE = 0x03;
  static constexpr unsigned char BIAS_INITIAL_SET = 0x01;
  static constexpr unsigned char CONTINUOUS_MODE_SPECIAL = 0x02;

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroset_srv_;

  std::string can_interface_;
  double sampling_period_seconds_;
  int socket_fd_{-1};
  bool initialized_{false};
  struct sockaddr_can addr_ {};
  struct ifreq ifr_ {};
  struct can_frame rxframe_ {};
  struct can_frame txframe_ {};
  std::array<std::array<unsigned char, 8>, 2> can_rawdata_ {};

  int bias_flag_{0};
  std::array<float, 6> wrench_ {};
  std::array<float, 6> wrench_bias_ {};

  bool can_initialize();
  int can_data_recv(unsigned char data0, unsigned char data1, unsigned char data2);
  int can_data_send(unsigned char data0, unsigned char data1, unsigned char data2);
  void aidin_command_connection();
  void aidin_command_zeroset();
  void read_ftsensor();
  void set_bias();
  void publish_wrench();
  void zeroset_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};
