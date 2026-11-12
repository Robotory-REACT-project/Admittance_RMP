#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class SensorDataLogger : public rclcpp::Node
{
private:
  static constexpr std::size_t kFtSensorCount = 2;
  static constexpr std::size_t kProximitySensorCount = 2;
  static constexpr std::uint8_t kAllProximityFields = 0x1Fu;

  enum class ProximityField : std::uint8_t
  {
    kRaw = 0x01u,
    kTof = 0x02u,
    kSigma = 0x04u,
    kCap = 0x08u,
    kFused = 0x10u,
  };

  struct ProximityFrame
  {
    double elapsed_sec{0.0};
    double message_stamp_sec{0.0};
    double raw_count{0.0};
    double tof_m{0.0};
    double cap_sigma_mm{0.0};
    double cap_distance_m{0.0};
    double fused_distance_m{0.0};
    std::uint8_t received_fields{0u};
  };

public:
  SensorDataLogger()
  : Node("sensor_data_logger"), start_time_(std::chrono::steady_clock::now())
  {
    const std::array<std::string, kFtSensorCount> ft_topics = {
      declare_parameter<std::string>("ft_topic1", "/ur10skku/ftdata_1"),
      declare_parameter<std::string>("ft_topic2", "/ur10skku/ftdata_2")};
    const std::array<std::string, kProximitySensorCount> raw_topics = {
      declare_parameter<std::string>("raw_topic1", "/filter_raw1"),
      declare_parameter<std::string>("raw_topic2", "/filter_raw2")};
    const std::array<std::string, kProximitySensorCount> tof_topics = {
      declare_parameter<std::string>("tof_topic1", "/tof_distance1"),
      declare_parameter<std::string>("tof_topic2", "/tof_distance2")};
    const std::array<std::string, kProximitySensorCount> sigma_topics = {
      declare_parameter<std::string>("sigma_topic1", "/cap_sigma1"),
      declare_parameter<std::string>("sigma_topic2", "/cap_sigma2")};
    const std::array<std::string, kProximitySensorCount> cap_topics = {
      declare_parameter<std::string>("cap_topic1", "/cap_distance1"),
      declare_parameter<std::string>("cap_topic2", "/cap_distance2")};
    const std::array<std::string, kProximitySensorCount> fused_topics = {
      declare_parameter<std::string>("fused_topic1", "/fused_distance1"),
      declare_parameter<std::string>("fused_topic2", "/fused_distance2")};
    const auto log_root_directory = declare_parameter<std::string>(
      "log_root_directory", default_log_root_directory());

    for (const auto & ft_topic : ft_topics) {
      if (ft_topic.empty()) {
        throw std::invalid_argument("FT logger topic names must not be empty");
      }
    }
    for (std::size_t sensor = 0; sensor < kProximitySensorCount; ++sensor) {
      if (raw_topics[sensor].empty() || tof_topics[sensor].empty() ||
        sigma_topics[sensor].empty() || cap_topics[sensor].empty() ||
        fused_topics[sensor].empty())
      {
        throw std::invalid_argument("proximity logger topic names must not be empty");
      }
    }
    if (log_root_directory.empty()) {
      throw std::invalid_argument("log_root_directory must not be empty");
    }

    session_directory_ = create_session_directory(log_root_directory);
    for (std::size_t sensor = 0; sensor < kFtSensorCount; ++sensor) {
      const auto path = session_directory_ /
        ("ft_sensor" + std::to_string(sensor + 1) + ".txt");
      ft_streams_[sensor].open(path, std::ios::out | std::ios::trunc);
      if (!ft_streams_[sensor].is_open()) {
        throw std::runtime_error("failed to open " + path.string());
      }
      auto & stream = ft_streams_[sensor];
      stream << std::fixed << std::setprecision(9);
      stream << "# sensor_id: " << (sensor + 1) << '\n';
      stream << "# ft_topic: " << ft_topics[sensor] << '\n';
      stream <<
        "# columns: elapsed_sec message_stamp_sec force_x_N force_y_N force_z_N "
        "torque_x_Nm torque_y_Nm torque_z_Nm\n";
    }

    for (std::size_t sensor = 0; sensor < kProximitySensorCount; ++sensor) {
      const auto path = session_directory_ /
        ("proximity_sensor" + std::to_string(sensor + 1) + ".txt");
      proximity_streams_[sensor].open(path, std::ios::out | std::ios::trunc);
      if (!proximity_streams_[sensor].is_open()) {
        throw std::runtime_error("failed to open " + path.string());
      }
      auto & stream = proximity_streams_[sensor];
      stream << std::fixed << std::setprecision(9);
      stream << "# sensor_id: " << (sensor + 1) << '\n';
      stream << "# raw_topic: " << raw_topics[sensor] << '\n';
      stream << "# tof_topic: " << tof_topics[sensor] << '\n';
      stream << "# sigma_topic: " << sigma_topics[sensor] << '\n';
      stream << "# cap_topic: " << cap_topics[sensor] << '\n';
      stream << "# fused_topic: " << fused_topics[sensor] << '\n';
      stream <<
        "# columns: elapsed_sec message_stamp_sec sequence raw_count "
        "tof_m cap_sigma_mm cap_distance_m fused_distance_m\n";
    }
    flush_logs();

    const auto logger_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    for (std::size_t sensor = 0; sensor < kFtSensorCount; ++sensor) {
      ft_subscriptions_[sensor] =
        create_subscription<geometry_msgs::msg::WrenchStamped>(
        ft_topics[sensor], logger_qos,
        [this, sensor](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
          log_ft(sensor, *msg);
        });
    }

    for (std::size_t sensor = 0; sensor < kProximitySensorCount; ++sensor) {
      raw_subscriptions_[sensor] = create_subscription<sensor_msgs::msg::Range>(
        raw_topics[sensor], logger_qos,
        [this, sensor](sensor_msgs::msg::Range::ConstSharedPtr msg) {
          collect_proximity(sensor, ProximityField::kRaw, *msg);
        });
      tof_subscriptions_[sensor] = create_subscription<sensor_msgs::msg::Range>(
        tof_topics[sensor], logger_qos,
        [this, sensor](sensor_msgs::msg::Range::ConstSharedPtr msg) {
          collect_proximity(sensor, ProximityField::kTof, *msg);
        });
      sigma_subscriptions_[sensor] = create_subscription<sensor_msgs::msg::Range>(
        sigma_topics[sensor], logger_qos,
        [this, sensor](sensor_msgs::msg::Range::ConstSharedPtr msg) {
          collect_proximity(sensor, ProximityField::kSigma, *msg);
        });
      cap_subscriptions_[sensor] = create_subscription<sensor_msgs::msg::Range>(
        cap_topics[sensor], logger_qos,
        [this, sensor](sensor_msgs::msg::Range::ConstSharedPtr msg) {
          collect_proximity(sensor, ProximityField::kCap, *msg);
        });
      fused_subscriptions_[sensor] = create_subscription<sensor_msgs::msg::Range>(
        fused_topics[sensor], logger_qos,
        [this, sensor](sensor_msgs::msg::Range::ConstSharedPtr msg) {
          collect_proximity(sensor, ProximityField::kFused, *msg);
        });
    }

    flush_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {flush_logs();});

    RCLCPP_INFO(
      get_logger(), "Automatic synchronized sensor logging started: %s",
      session_directory_.c_str());
  }

  ~SensorDataLogger() override
  {
    flush_logs();
  }

private:
  static std::string default_log_root_directory()
  {
    const char * user_home = std::getenv("HOME");
    if (user_home != nullptr && user_home[0] != '\0') {
      return (
        std::filesystem::path(user_home) / "ros2_ws" / "src" /
        "REACT_intrinsic" / "sensor_data_logs").string();
    }
    return "/tmp/REACT_intrinsic_sensor_data_logs";
  }

  static std::filesystem::path create_session_directory(const std::string & log_root)
  {
    const std::filesystem::path root(log_root);
    std::filesystem::create_directories(root);

    const auto current_time = std::chrono::system_clock::now();
    const std::time_t current_time_t = std::chrono::system_clock::to_time_t(current_time);
    std::tm local_time{};
    localtime_r(&current_time_t, &local_time);

    std::ostringstream directory_name;
    directory_name << "session_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    const std::string base_name = directory_name.str();

    for (std::size_t suffix = 0; suffix < 1000; ++suffix) {
      const std::string candidate_name = suffix == 0 ?
        base_name : base_name + "_" + std::to_string(suffix);
      const auto candidate = root / candidate_name;
      if (std::filesystem::create_directory(candidate)) {
        return candidate;
      }
    }

    throw std::runtime_error("failed to create a unique sensor log session directory");
  }

  double elapsed_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  }

  static double message_stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  void log_ft(
    const std::size_t sensor, const geometry_msgs::msg::WrenchStamped & msg)
  {
    ft_streams_[sensor] << elapsed_seconds() << ' '
                        << message_stamp_seconds(msg.header.stamp) << ' '
                        << msg.wrench.force.x << ' '
                        << msg.wrench.force.y << ' '
                        << msg.wrench.force.z << ' '
                        << msg.wrench.torque.x << ' '
                        << msg.wrench.torque.y << ' '
                        << msg.wrench.torque.z << '\n';
  }

  void collect_proximity(
    const std::size_t sensor, const ProximityField field,
    const sensor_msgs::msg::Range & msg)
  {
    const rclcpp::Time message_time(msg.header.stamp);
    const std::int64_t stamp_ns = message_time.nanoseconds();
    auto & pending = pending_proximity_[sensor];
    auto result = pending.try_emplace(stamp_ns);
    auto & frame = result.first->second;
    if (result.second) {
      frame.elapsed_sec = elapsed_seconds();
      frame.message_stamp_sec = message_time.seconds();
    }

    switch (field) {
      case ProximityField::kRaw:
        frame.raw_count = msg.range;
        break;
      case ProximityField::kTof:
        frame.tof_m = msg.range;
        break;
      case ProximityField::kSigma:
        frame.cap_sigma_mm = msg.range;
        break;
      case ProximityField::kCap:
        frame.cap_distance_m = msg.range;
        break;
      case ProximityField::kFused:
        frame.fused_distance_m = msg.range;
        break;
    }
    frame.received_fields |= static_cast<std::uint8_t>(field);

    if (frame.received_fields == kAllProximityFields) {
      auto & stream = proximity_streams_[sensor];
      stream << frame.elapsed_sec << ' '
             << frame.message_stamp_sec << ' '
             << ++proximity_sequences_[sensor] << ' '
             << frame.raw_count << ' '
             << frame.tof_m << ' '
             << frame.cap_sigma_mm << ' '
             << frame.cap_distance_m << ' '
             << frame.fused_distance_m << '\n';
      pending.erase(result.first);
    }

    // Bound memory if one topic is temporarily absent. Complete frames normally
    // leave the map immediately because all four eCAN messages share one stamp.
    while (pending.size() > 500u) {
      pending.erase(pending.begin());
    }
  }

  void flush_logs()
  {
    for (auto & stream : ft_streams_) {
      if (stream.is_open()) {
        stream.flush();
      }
    }
    for (auto & stream : proximity_streams_) {
      if (stream.is_open()) {
        stream.flush();
      }
    }
  }

  std::chrono::steady_clock::time_point start_time_;
  std::filesystem::path session_directory_;
  std::array<std::ofstream, kFtSensorCount> ft_streams_;
  std::array<std::ofstream, kProximitySensorCount> proximity_streams_;
  std::array<std::map<std::int64_t, ProximityFrame>, kProximitySensorCount>
  pending_proximity_;
  std::array<std::uint64_t, kProximitySensorCount> proximity_sequences_{};
  std::array<
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr,
    kFtSensorCount> ft_subscriptions_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
    kProximitySensorCount>
  raw_subscriptions_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
    kProximitySensorCount>
  tof_subscriptions_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
    kProximitySensorCount>
  sigma_subscriptions_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
    kProximitySensorCount>
  cap_subscriptions_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
    kProximitySensorCount>
  fused_subscriptions_{};
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataLogger>());
  rclcpp::shutdown();
  return 0;
}
