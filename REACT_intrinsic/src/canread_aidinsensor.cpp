#include "canread_aidinsensor.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <unistd.h>

CanReadAidinSensor::CanReadAidinSensor(
  rclcpp::Node::SharedPtr node,
  const std::string & can_interface,
  double sampling_period_seconds)
: node_(std::move(node)),
  can_interface_(can_interface),
  sampling_period_seconds_(sampling_period_seconds > 0.0 ? sampling_period_seconds : 0.002)
{
  ftsensor_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/ftsensor/measured_value", 10);
  zeroset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "/ftsensor/zeroset",
    std::bind(
      &CanReadAidinSensor::zeroset_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(sampling_period_seconds_));
  timer_ = node_->create_wall_timer(period, std::bind(&CanReadAidinSensor::publish_wrench, this));

  initialized_ = can_initialize();
}

CanReadAidinSensor::~CanReadAidinSensor()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

bool CanReadAidinSensor::can_initialize()
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create SocketCAN socket: %s", std::strerror(errno));
    return false;
  }

  std::strncpy(ifr_.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0) {
    RCLCPP_ERROR(
      node_->get_logger(), "Cannot find CAN interface '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_)) < 0) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to bind '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  // Ensure shutdown and service callbacks are not blocked forever when no CAN frame arrives.
  struct timeval timeout {};
  timeout.tv_usec = 100000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  txframe_.can_id = 0x102;
  txframe_.can_dlc = 8;
  std::memset(txframe_.data, 0, sizeof(txframe_.data));

  aidin_command_connection();
  aidin_command_zeroset();
  RCLCPP_INFO(node_->get_logger(), "SocketCAN connected on %s", can_interface_.c_str());
  return true;
}

int CanReadAidinSensor::can_data_recv(
  unsigned char data0, unsigned char data1, unsigned char data2)
{
  txframe_.data[0] = data0;
  txframe_.data[1] = data1;
  txframe_.data[2] = data2;
  if (write(socket_fd_, &txframe_, sizeof(txframe_)) != static_cast<ssize_t>(sizeof(txframe_))) {
    RCLCPP_WARN(node_->get_logger(), "SocketCAN command write failed: %s", std::strerror(errno));
  }

  return recv(socket_fd_, &rxframe_, sizeof(rxframe_), 0);
}

int CanReadAidinSensor::can_data_send(
  unsigned char data0, unsigned char data1, unsigned char data2)
{
  txframe_.data[0] = data0;
  txframe_.data[1] = data1;
  txframe_.data[2] = data2;
  return write(socket_fd_, &txframe_, sizeof(txframe_));
}

void CanReadAidinSensor::aidin_command_connection()
{
  can_data_send(DEVICE_ID, FIELD_CONTINUOUS_MODE, CONTINUOUS_MODE_SPECIAL);
}

void CanReadAidinSensor::aidin_command_zeroset()
{
  can_data_send(DEVICE_ID, FIELD_BIAS, BIAS_INITIAL_SET);
}

void CanReadAidinSensor::read_ftsensor()
{
  if (!initialized_) {
    return;
  }

  if (can_data_recv(DEVICE_ID, FIELD_CONTINUOUS_MODE, 0x00) <= 0) {
    return;
  }

  const canid_t frame_id = rxframe_.can_id & CAN_SFF_MASK;
  if (frame_id < DEVICE_ID || frame_id > DEVICE_ID + 1) {
    return;
  }

  const std::size_t frame_index = static_cast<std::size_t>(frame_id - DEVICE_ID);
  const std::size_t dlc = std::min<std::size_t>(rxframe_.can_dlc, can_rawdata_[frame_index].size());
  for (std::size_t i = 0; i < dlc; ++i) {
    can_rawdata_[frame_index][i] = rxframe_.data[i];
  }

  std::array<unsigned short, 3> data_recv {};
  for (std::size_t i = 0; i < data_recv.size(); ++i) {
    const auto raw = static_cast<unsigned short>(
      static_cast<unsigned short>(can_rawdata_[frame_index][2 * i]) * 256U +
      static_cast<unsigned short>(can_rawdata_[frame_index][2 * i + 1]));
    data_recv[i] = static_cast<unsigned short>(raw - 32767U);
  }

  if (frame_id == DEVICE_ID) {
    if (bias_flag_ == 0) {
      for (std::size_t i = 0; i < 3; ++i) {
        wrench_bias_[i] = static_cast<float>(data_recv[i]) / 100.0F - 300.0F;
      }
      bias_flag_ = 1;
    }
    for (std::size_t i = 0; i < 3; ++i) {
      wrench_[i] = static_cast<float>(data_recv[i]) / 100.0F - 300.0F - wrench_bias_[i];
    }
  } else {
    if (bias_flag_ == 1) {
      for (std::size_t i = 0; i < 3; ++i) {
        wrench_bias_[i + 3] = static_cast<float>(data_recv[i]) / 500.0F - 50.0F;
      }
      bias_flag_ = 2;
    }
    for (std::size_t i = 0; i < 3; ++i) {
      wrench_[i + 3] =
        static_cast<float>(data_recv[i]) / 500.0F - 50.0F - wrench_bias_[i + 3];
    }
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "FT: %.2f %.2f %.2f N | %.3f %.3f %.3f Nm",
    wrench_[0], wrench_[1], wrench_[2], wrench_[3], wrench_[4], wrench_[5]);
}

void CanReadAidinSensor::set_bias()
{
  bias_flag_ = 0;
}

void CanReadAidinSensor::zeroset_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  set_bias();
  response->success = true;
  response->message = "Force/torque bias will be refreshed from the next two CAN frames.";
  RCLCPP_INFO(node_->get_logger(), "Zero-set requested");
}

void CanReadAidinSensor::publish_wrench()
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = node_->now();
  message.header.frame_id = "ft_sensor";
  message.wrench.force.x = wrench_[0];
  message.wrench.force.y = wrench_[1];
  message.wrench.force.z = wrench_[2];
  message.wrench.torque.x = wrench_[3];
  message.wrench.torque.y = wrench_[4];
  message.wrench.torque.z = wrench_[5];
  ftsensor_pub_->publish(message);
}

void CanReadAidinSensor::run()
{
  while (rclcpp::ok()) {
    read_ftsensor();
    rclcpp::spin_some(node_);
  }
}
