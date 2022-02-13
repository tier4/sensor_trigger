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

#include "jetson_camera_trigger/jetson_gpio.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace jetson_camera_trigger
{

JetsonGpio::JetsonGpio(const std::map<int, int> & pin_gpio_mapping)
: pin_gpio_mapping_(pin_gpio_mapping)
{
}

int JetsonGpio::exportGpio(const int gpio) const
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  file_descriptor = open("/sys/class/gpio/export", O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  buffer_length = snprintf(buffer, sizeof(buffer), "%d", gpio);
  if (write(file_descriptor, buffer, buffer_length) != buffer_length) {
    return file_descriptor;
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return 0;
}

int JetsonGpio::unexportGpio(const int gpio) const
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  file_descriptor = open("/sys/class/gpio/unexport", O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  buffer_length = snprintf(buffer, sizeof(buffer), "%d", gpio);
  if (write(file_descriptor, buffer, buffer_length) != buffer_length) {
    return file_descriptor;
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return 0;
}

int JetsonGpio::setGpioDirection(const int gpio, const GpioDirection direction) const
{
  int file_descriptor;
  char buffer[BUFFER_SIZE];

  snprintf(buffer, sizeof(buffer), "/sys/class/gpio/gpio%d/direction", gpio);

  file_descriptor = open(buffer, O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  if (static_cast<uint8_t>(direction)) {
    if (write(file_descriptor, "out", 4) != 4) {
      return file_descriptor;
    }
  } else {
    if (write(file_descriptor, "in", 3) != 3) {
      return file_descriptor;
    }
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return 0;
}

int JetsonGpio::setGpioState(const int gpio, const GpioState state) const
{
  int file_descriptor;
  char buffer[BUFFER_SIZE];

  snprintf(buffer, sizeof(buffer), "/sys/class/gpio/gpio%d/value", gpio);

  file_descriptor = open(buffer, O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  if (static_cast<uint8_t>(state)) {
    if (write(file_descriptor, "1", 2) != 2) {
      return file_descriptor;
    }
  } else {
    if (write(file_descriptor, "0", 2) != 2) {
      return file_descriptor;
    }
  }
  close(file_descriptor);
  return 0;
}

int JetsonGpio::exportGpioPin(const int gpio_pin)
{
  return exportGpio(pin_gpio_mapping_[gpio_pin]);
}

int JetsonGpio::unexportGpioPin(const int gpio_pin)
{
  return unexportGpio(pin_gpio_mapping_[gpio_pin]);
}

int JetsonGpio::setGpioPinDirection(const int gpio_pin, const GpioDirection direction)
{
  return setGpioDirection(pin_gpio_mapping_[gpio_pin], direction);
}

int JetsonGpio::setGpioPinState(const int gpio_pin, const GpioState state)
{
  return setGpioState(pin_gpio_mapping_[gpio_pin], state);
}

}  // namespace jetson_camera_trigger
