// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_
#define SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pthread.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sensor_trigger
{
class SensorTrigger : public rclcpp::Node
{
public:
  explicit SensorTrigger(const rclcpp::NodeOptions & node_options);
  ~SensorTrigger();

private:
  // ros::NodeHandle nh_{ "" };
  // ros::NodeHandle private_nh_{ "~" };
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr trigger_time_publisher_;

  // Map from gpio name to chip number and line number
  YAML::Node gpio_mapping_;
  bool get_gpio_chip_and_line();

  // Triggering configuration
  double fps_;
  double phase_;
  std::string gpio_name_;
  unsigned int gpio_chip_;
  unsigned int gpio_line_;
  int cpu_;
  std::mutex iomutex_;
  int64_t pulse_width_ms_;
  jetson_gpio::JetsonGpio gpio_handler_;

  // Trigger thread
  std::unique_ptr<std::thread> trigger_thread_;

  void run();
};
}  // namespace sensor_trigger

#endif  // SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_
