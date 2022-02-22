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

#include <fcntl.h>
#include <jetson_camera_trigger/jetson_gpio.hpp>
#include <unistd.h>

int export_gpio(int gpio)
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  file_descriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
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

int unexport_gpio(int gpio)
{
  int file_descriptor, buffer_length;
  char buffer[BUFFER_SIZE];

  file_descriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
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

int set_gpio_direction(int gpio, gpio_direction direction)
{
  int file_descriptor;
  char buffer[BUFFER_SIZE];

  snprintf(buffer, sizeof(buffer), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);

  file_descriptor = open(buffer, O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  if (direction) {
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

int set_gpio_state(int gpio, gpio_state state)
{
  int file_descriptor;
  char buffer[BUFFER_SIZE];

  snprintf(buffer, sizeof(buffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

  file_descriptor = open(buffer, O_WRONLY);
  if (file_descriptor < 0) {
    return file_descriptor;
  }
  if (state) {
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

int export_gpio_pin(int gpio_pin) { return export_gpio(pin_gpio_mapping[gpio_pin]); }

int unexport_gpio_pin(int gpio_pin) { return unexport_gpio(pin_gpio_mapping[gpio_pin]); }

int set_gpio_pin_direction(int gpio_pin, gpio_direction direction)
{
  return set_gpio_direction(pin_gpio_mapping[gpio_pin], direction);
}

int set_gpio_pin_state(int gpio_pin, gpio_state state)
{
  return set_gpio_state(pin_gpio_mapping[gpio_pin], state);
}
