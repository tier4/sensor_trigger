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

#include "jetson_camera_trigger/jetson_camera_trigger.hpp"

namespace jetson_camera_trigger
{
JetsonCameraTrigger::JetsonCameraTrigger(const rclcpp::NodeOptions & node_options)
: Node("jetson_camera_trigger", node_options)
{
  const auto frame_rate = declare_parameter("frame_rate", 10.0);
  const auto phase_degree = declare_parameter("phase_degree", 0.0);
  gpio_pin_ = declare_parameter("gpio_pin", 0);
  const std::map<int, int> pin_gpio_mapping{{51, 408}, {52, 350}, {53, 446}, {54, 445}};
  const auto it = pin_gpio_mapping.find(gpio_pin_);
  if (it == pin_gpio_mapping.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid gpio pin: " << gpio_pin_);
    rclcpp::shutdown();
    return;
  }

  jetson_gpio_ = std::make_unique<JetsonGpio>(pin_gpio_mapping);
  if (
    jetson_gpio_->exportGpioPin(gpio_pin_) ||
    jetson_gpio_->setGpioPinDirection(gpio_pin_, GpioDirection::GPIO_OUTPUT)) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Failed to initialize GPIO trigger. Not using triggering on " << gpio_pin_ << ".");
    rclcpp::shutdown();
    return;
  }

  trigger_time_pub_ = create_publisher<builtin_interfaces::msg::Time>("trigger_time", 1);

  // Timer
  const auto period_ns = rclcpp::Rate(frame_rate).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, [this]() {
    jetson_gpio_->setGpioPinState(gpio_pin_, GpioState::GPIO_HIGH);
    trigger_time_pub_->publish(now());
    jetson_gpio_->setGpioPinState(gpio_pin_, GpioState::GPIO_LOW);
  });
  timer_->cancel();
  while (std::abs(
           period_ns.count() * phase_degree / 360.0 -
           builtin_interfaces::msg::Time{now()}.nanosec) > 1e6) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(500));
  }
  RCLCPP_INFO(get_logger(), "Start triggering");
  timer_->reset();
}

JetsonCameraTrigger::~JetsonCameraTrigger() { jetson_gpio_->unexportGpioPin(gpio_pin_); }

}  // namespace jetson_camera_trigger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_camera_trigger::JetsonCameraTrigger)
