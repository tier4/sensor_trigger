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

#include <cstdio>
#include <sys/stat.h>

namespace jetson_gpio
{
JetsonGpio::~JetsonGpio()
{
  if (state_file_descriptor_ > -1) {
    close(state_file_descriptor_);
  }

  // Regardless of the existence of other processes that uses the same GPIO pin
  // (incl. zombie GPIO port opening because of failure exit),
  // this unexport closes the target GPIO pin anyway.
  // This behavior intends to make next try to use this GPIO pin success.
  unexport_gpio();
}

bool JetsonGpio::init_gpio_pin(int gpio_pin, gpio_direction direction)
{

  gpio_chip_ = gpiod::chip(GPIO_CHARACTER_DEVICE);
  gpio_lines_ = gpio_chip_.get_lines(std::vector<unsigned int>({gpio_pin}));  // XXX: 143 = Anvil misc.I/O GP_Out_1, 108 = PWM_Out_0
  gpio_request_ = {
    "sensor_trigger", // consumer name. XXX: fixed name may conflict for multiple instances
    gpiod::line_request::DIRECTION_OUTPUT,  // request_type
    0                                       // flag
  };

  if (!set_gpio_direction(direction)) {
    return false;
  }

  gpio_lines_.request(gpio_request_, std::vector<int>({GPIO_LOW}));

  return true;
}

bool JetsonGpio::export_gpio()
{
  return true;
}

bool JetsonGpio::unexport_gpio()
{
  gpio_chip_.~chip();

  return true;
}

bool JetsonGpio::set_gpio_direction(gpio_direction direction)
{
  switch (direction) {
    case GPIO_INPUT:
      gpio_request_.request_type = gpiod::line_request::DIRECTION_INPUT;
      break;
    case GPIO_OUTPUT:
      gpio_request_.request_type = gpiod::line_request::DIRECTION_OUTPUT;
      break;
  }

  return true;
}

bool JetsonGpio::set_gpio_pin_state(gpio_state state)
{

  gpio_lines_.set_values(std::vector<int>({state}));

  return true;
}
}  // namespace jetson_gpio
