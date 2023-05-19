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
  gpio_ = pin_gpio_mapping[gpio_pin];
  // Setup GPIO to use
  if (!export_gpio()) {
    return false;
  }
  if (!set_gpio_direction(direction)) {
    return false;
  }

  // Open file for writing state and keep it as a member of the class to avoid iterative open/close
  char buffer[BUFFER_SIZE];
  snprintf(buffer, sizeof(buffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio_);
  switch (direction) {
    case GPIO_OUTPUT:
      state_file_descriptor_ = open(buffer, O_WRONLY);
      break;
    case GPIO_INPUT:
      state_file_descriptor_ = open(buffer, O_RDONLY);
      break;
  }
  if (state_file_descriptor_ < 0) {
    return false;
  }

  return true;
}

bool JetsonGpio::export_gpio()
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  // Check target GPIO is available (not opened) first.
  // If it is unavailable, close the port and try to open it.
    std::string target_gpio = std::string(SYSFS_GPIO_DIR) + std::string("/gpio")
                              + std::to_string(gpio_);
  struct stat return_status;
  if (stat(target_gpio.c_str(), &return_status) == 0) {
       unexport_gpio();
  }

  file_descriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (file_descriptor < 0) {
    return false;
  }

  buffer_length = snprintf(buffer, sizeof(buffer), "%d", gpio_);
  if (write(file_descriptor, buffer, buffer_length) != buffer_length) {
    close(file_descriptor);
    return false;
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return true;
}

bool JetsonGpio::unexport_gpio()
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  file_descriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (file_descriptor < 0) {
    return false;
  }
  buffer_length = snprintf(buffer, sizeof(buffer), "%d", gpio_);
  if (write(file_descriptor, buffer, buffer_length) != buffer_length) {
    close(file_descriptor);
    return false;
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return true;
}

bool JetsonGpio::set_gpio_direction(gpio_direction direction)
{
  int file_descriptor;
  char buffer[BUFFER_SIZE];

  snprintf(buffer, sizeof(buffer), SYSFS_GPIO_DIR "/gpio%d/direction", gpio_);

  file_descriptor = open(buffer, O_WRONLY);
  if (file_descriptor < 0) {
    return false;
  }
  if (direction) {
    if (write(file_descriptor, "out", 4) != 4) {
      close(file_descriptor);
      return false;
    }
  } else {
    if (write(file_descriptor, "in", 3) != 3) {
      close(file_descriptor);
      return false;
    }
  }
  close(file_descriptor);

  // Delay to prevent errors in next sysfs call
  usleep(1000000);

  return true;
}

bool JetsonGpio::set_gpio_pin_state(gpio_state state)
{
  if (state) {
    if (write(state_file_descriptor_, "1", 2) != 2) {
      return false;
    }
  } else {
    if (write(state_file_descriptor_, "0", 2) != 2) {
      return false;
    }
  }
  return true;
}
}  // namespace jetson_gpio
