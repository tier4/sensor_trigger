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

#ifndef JETSON_CAMERA_TRIGGER__JETSON_GPIO_HPP_
#define JETSON_CAMERA_TRIGGER__JETSON_GPIO_HPP_

#include <map>

namespace jetson_camera_trigger
{

static constexpr int BUFFER_SIZE = 64;
enum class GpioDirection : uint8_t { GPIO_INPUT = 0, GPIO_OUTPUT };
enum class GpioState : uint8_t { GPIO_LOW = 0, GPIO_HIGH };
class JetsonGpio
{
public:
  explicit JetsonGpio(const std::map<int, int> & pin_gpio_mapping);

  int exportGpioPin(const int gpio_pin);
  int unexportGpioPin(const int gpio_pin);
  int setGpioPinDirection(const int gpio_pin, const GpioDirection direction);
  int setGpioPinState(const int gpio_pin, const GpioState state);

private:
  int exportGpio(const int gpio) const;
  int unexportGpio(const int gpio) const;
  int setGpioDirection(const int gpio, const GpioDirection direction) const;
  int setGpioState(const int gpio, const GpioState state) const;

  std::map<int, int> pin_gpio_mapping_;
};
}  // namespace jetson_camera_trigger

#endif  // JETSON_CAMERA_TRIGGER__JETSON_GPIO_HPP_
