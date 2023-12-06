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

#include <sys/stat.h>

#include <cstdio>

namespace jetson_gpio
{
JetsonGpio::~JetsonGpio()
{
  // Regardless of the existence of other processes that uses the same GPIO pin
  // (incl. zombie GPIO port opening because of failure exit),
  // this function closes the target GPIO pin anyway.
  // This behavior intends to make next try to use this GPIO pin success.
  close_gpio();
}

bool JetsonGpio::init_gpio_pin(
  unsigned int gpio_chip, unsigned int gpio_line, gpio_direction direction)
{
  std::string gpio_character_device = "/dev/gpiochip" + std::to_string(gpio_chip);

  gpio_chip_ = gpiod::chip(gpio_character_device);
  gpio_line_ = gpio_chip_.get_line(gpio_line);
  gpio_request_ = {
    "sensor_trigger",  // consumer name
    direction,         // request_type
    0                  // flag
  };

  gpio_line_.request(gpio_request_, GPIO_LOW);

  return true;
}

bool JetsonGpio::close_gpio()
{
  gpio_chip_.~chip();

  return true;
}

bool JetsonGpio::set_gpio_pin_state(gpio_state state)
{
  gpio_line_.set_value(state);

  return true;
}
}  // namespace jetson_gpio
