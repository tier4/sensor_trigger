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

#include <sensor_trigger/jetson_gpio.hpp>
#include <sensor_trigger/sensor_trigger.hpp>

namespace sensor_trigger
{
SensorTrigger::SensorTrigger(const rclcpp::NodeOptions & node_options)
: Node("sensor_trigger", node_options)
{
  // Get the triggering parameters
  fps_ = declare_parameter("frame_rate", 10.0);
  phase_ = declare_parameter("phase", 0.0);
  gpio_name_ = declare_parameter("gpio_name", "roscube_trigger_1");
  cpu_ = declare_parameter("cpu_core_id", 1);
  pulse_width_ms_ = declare_parameter("pulse_width_ms", 5);
  std::string gpio_mapping_file = declare_parameter("gpio_mapping_file", "gpio_mapping.yaml");

  gpio_mapping_ = YAML::LoadFile(gpio_mapping_file);

  if (!get_gpio_chip_and_line()) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "No valid trigger GPIO specified. Not using triggering on GPIO name " << gpio_name_ << ".");
    rclcpp::shutdown();
    return;
  }

  if (!gpio_handler_.init_gpio_pin(gpio_chip_, gpio_line_, GPIO_OUTPUT)) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to initialize GPIO trigger. Not using triggering on GPIO chip number "
                      << gpio_chip_ << "line number " << gpio_line_ << ".");
    rclcpp::shutdown();
    return;
  }

  if (fps_ < 1.0) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Unable to trigger slower than 1 fps. Not using triggering on GPIO chip number "
                      << gpio_chip_ << "line number " << gpio_line_ << ".");
    rclcpp::shutdown();
    return;
  }

  if (cpu_ < 0 || cpu_ >= static_cast<int>(std::thread::hardware_concurrency())) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Selected CPU core"
        << cpu_
        << " is not available on this architecture. Not using triggering on GPIO chip number "
        << gpio_chip_ << "line number " << gpio_line_ << ".");
  }

  // Set CPU affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_, &cpuset);
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to set CPU affinity: " << strerror(errno) << ".");
  }
  if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to check CPU affinity: " << strerror(errno) << ".");
  }

  // Create thread
  trigger_time_publisher_ = create_publisher<builtin_interfaces::msg::Time>("trigger_time", 1000);
  trigger_thread_ = std::make_unique<std::thread>(&SensorTrigger::run, this);

  // Set thread priority
  sched_param sch;
  int policy;
  pthread_getschedparam(trigger_thread_->native_handle(), &policy, &sch);
  sch.sched_priority = 30;
  if (pthread_setschedparam(trigger_thread_->native_handle(), SCHED_FIFO, &sch)) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Failed to set schedule parameters: " << strerror(errno) << ".");
  }
}

SensorTrigger::~SensorTrigger()
{
  if (trigger_thread_) {
    if (trigger_thread_->joinable()) {
      trigger_thread_->join();
    }
  }
}

void SensorTrigger::run()
{
  builtin_interfaces::msg::Time trigger_time_msg;

  // Start on the first time after TOS
  int64_t start_nsec;
  int64_t end_nsec;
  int64_t target_nsec;
  int64_t interval_nsec = (int64_t)(1e9 / fps_);
  int64_t pulse_width = pulse_width_ms_ * 1e6;  // millisecond -> nanoseconds
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
    std::lock_guard<std::mutex> guard(iomutex_);
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
    bool to_high = gpio_handler_.set_gpio_pin_state(GPIO_HIGH);
    rclcpp::sleep_for(std::chrono::nanoseconds(pulse_width));
    rclcpp::Time now = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
    int64_t now_sec = now.nanoseconds() / 1e9;
    trigger_time_msg.sec = (int32_t)now_sec;
    trigger_time_msg.nanosec = (uint32_t)now_nsec;
    trigger_time_publisher_->publish(trigger_time_msg);
    bool to_low = gpio_handler_.set_gpio_pin_state(GPIO_LOW);
    target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
    if (!(to_high && to_low)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set GPIO status: " << strerror(errno));
      rclcpp::shutdown();
      return;
    }
  }
}

bool SensorTrigger::get_gpio_chip_and_line()
{
  if (
    gpio_mapping_[gpio_name_] && gpio_mapping_[gpio_name_]["chip"] &&
    gpio_mapping_[gpio_name_]["line"]) {
    gpio_chip_ = gpio_mapping_[gpio_name_]["chip"].as<unsigned int>(),
    gpio_line_ = gpio_mapping_[gpio_name_]["line"].as<unsigned int>();
    return true;
  }
  return false;
}
}  // namespace sensor_trigger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_trigger::SensorTrigger)
