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

#ifndef SENSOR_TRIGGER__JETSON_GPIO_HPP_
#define SENSOR_TRIGGER__JETSON_GPIO_HPP_

#include <gpiod.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <map>
#include <string>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define BUFFER_SIZE 64
#define GPIO_OUTPUT gpiod::line_request::DIRECTION_OUTPUT
#define GPIO_INPUT gpiod::line_request::DIRECTION_INPUT
#define GPIO_HIGH 1
#define GPIO_LOW 0

typedef int gpio_direction;
typedef int gpio_state;

namespace jetson_gpio
{
class JetsonGpio
{
public:
  JetsonGpio() = default;
  ~JetsonGpio();
  bool init_gpio_pin(unsigned int gpio_chip, unsigned int gpio_line, gpio_direction direction);
  bool set_gpio_pin_state(gpio_state state);

protected:
  bool close_gpio();
  bool set_gpio_direction(gpio_direction direction);

  int gpio_;

  gpiod::chip gpio_chip_;
  gpiod::line gpio_line_;
  gpiod::line_request gpio_request_;
};
}  // namespace jetson_gpio
#endif  // SENSOR_TRIGGER__JETSON_GPIO_HPP_
