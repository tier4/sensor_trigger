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

#ifndef JETSON_CAMERA_TRIGGER__JETSON_CAMERA_TRIGGER_HPP_
#define JETSON_CAMERA_TRIGGER__JETSON_CAMERA_TRIGGER_HPP_

#include "jetson_camera_trigger/jetson_gpio.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace jetson_camera_trigger
{
class JetsonCameraTrigger : public rclcpp::Node
{
public:
  explicit JetsonCameraTrigger(const rclcpp::NodeOptions & node_options);
  ~JetsonCameraTrigger();

private:
  void onTimer();

  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr trigger_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<JetsonGpio> jetson_gpio_;

  int gpio_pin_;
};
}  // namespace jetson_camera_trigger

#endif  // JETSON_CAMERA_TRIGGER__JETSON_CAMERA_TRIGGER_HPP_
