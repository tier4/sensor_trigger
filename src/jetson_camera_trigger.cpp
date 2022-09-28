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

#include <jetson_camera_trigger/jetson_camera_trigger.hpp>
#include <jetson_camera_trigger/jetson_gpio.hpp>

namespace jetson_camera_trigger
{
JetsonCameraTrigger::JetsonCameraTrigger(const rclcpp::NodeOptions & node_options)
: Node("jetson_camera_trigger", node_options)
{
  // Get the triggering parameters
  fps_ = declare_parameter("frame_rate", 10.0);
  phase_ = declare_parameter("phase", 0.0);
  gpio_ = declare_parameter("gpio", 0);

  if (gpio_ <= 0) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No valid trigger GPIO specified. Not using triggering on GPIO " << gpio_ << ".");
    rclcpp::shutdown();
    return;
  }

  if (export_gpio_pin(gpio_) || set_gpio_pin_direction(gpio_, GPIO_OUTPUT)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to initialize GPIO trigger. Not using triggering on GPIO " << gpio_ << ".");
    rclcpp::shutdown();
    return;
  }

  if (fps_ < 1.0) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Unable to trigger slower than 1 fps. Not triggering on GPIO " << gpio_ << ".");
    rclcpp::shutdown();
    return;
  }

  trigger_time_publisher_ = create_publisher<builtin_interfaces::msg::Time>("trigger_time", 1000);
  trigger_thread_ = std::make_unique<std::thread>(&JetsonCameraTrigger::run, this);
}

JetsonCameraTrigger::~JetsonCameraTrigger()
{
  if (trigger_thread_) {
    if (trigger_thread_->joinable()) {
      trigger_thread_->join();
    }
  }
}

void JetsonCameraTrigger::run()
{
  builtin_interfaces::msg::Time trigger_time_msg;

  // Start on the first time after TOS
  int64_t start_nsec;
  int64_t end_nsec;
  int64_t target_nsec;
  int64_t interval_nsec = (int64_t)(1e9 / fps_);
  int64_t pulse_width = 5e6;
  int64_t wait_nsec = 0;
  int64_t now_nsec = 0;
  // Fix this later to remove magic numbers
  if (std::abs(phase_) <= 1e-7) {
    start_nsec = 0;
  } else {
    start_nsec = interval_nsec * (int64_t)(phase_ * 10) / 3600;
  }
  target_nsec = start_nsec;
  end_nsec = start_nsec - interval_nsec + 1e9;

  while (rclcpp::ok()) {
    // Do triggering stuff
    // Check current time - assume ROS uses best clock source
    do {
      now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      if (now_nsec < end_nsec) {
        while (now_nsec > target_nsec) {
          target_nsec = target_nsec + interval_nsec;
        }
        // FIX: what about very small phases and fast framerates giving a negative number?
        wait_nsec = target_nsec - now_nsec - 1e7;
      } else {
        target_nsec = start_nsec;
        wait_nsec = 1e9 - now_nsec + start_nsec - 1e7;
      }
      // Keep waiting for half the remaining time until the last millisecond.
      // This is required as sleep_for tends to oversleep significantly
      if (wait_nsec > 1e7) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_nsec / 2));
      }
    } while (wait_nsec > 1e7);
    // Block the last millisecond
    now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
    if (start_nsec == end_nsec) {
      while (now_nsec > 1e7) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    } else if (now_nsec < end_nsec) {
      while (now_nsec < target_nsec) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    } else {
      while (now_nsec > end_nsec || now_nsec < start_nsec) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    }
    // Trigger!
    set_gpio_pin_state(gpio_, GPIO_HIGH);
    rclcpp::sleep_for(std::chrono::nanoseconds(pulse_width));
    rclcpp::Time now = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
    int64_t now_sec = now.nanoseconds() / 1e9;
    trigger_time_msg.sec = (int32_t)now_sec;
    trigger_time_msg.nanosec = (uint32_t)now_nsec;
    trigger_time_publisher_->publish(trigger_time_msg);
    set_gpio_pin_state(gpio_, GPIO_LOW);
    target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
  }
  // Cleanup
  unexport_gpio_pin(gpio_);
}
}  // namespace jetson_camera_trigger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_camera_trigger::JetsonCameraTrigger)
