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

#include <map>
#include <string>
#include <fcntl.h>
#include <unistd.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define BUFFER_SIZE 64
#define GPIO_OUTPUT 1
#define GPIO_INPUT 0
#define GPIO_HIGH 1
#define GPIO_LOW 0

typedef int gpio_direction;
typedef int gpio_state;

namespace jetson_gpio {
// Mapping of GPIO number to pin number for ROSCubeX
// Note: pin 5->216 is pin 5 on the DB50 connector, run by GPIO chip 216 (starting at GPIO number
// 216)
static std::map<int, int> pin_gpio_mapping{{5, 216}, {51, 408}, {52, 350}, {53, 446}, {54, 445}};

class JetsonGpio
{
 public:
  explicit JetsonGpio()
  : state_file_descriptor_(-1) {}
  ~JetsonGpio();
  bool init_gpio_pin(int gpio_pin, gpio_direction direction);
  bool set_gpio_pin_state(gpio_state state);

 protected:
  bool export_gpio();
  bool unexport_gpio();
  bool set_gpio_direction(gpio_direction direction);

  int state_file_descriptor_;
  int gpio_;
};
}      // namespace jetson_gpio
#endif  // SENSOR_TRIGGER__JETSON_GPIO_HPP_
