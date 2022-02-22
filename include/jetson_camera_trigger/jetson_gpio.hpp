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

#ifndef __JETSON_GPIO_H
#define __JETSON_GPIO_H

#include <map>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define BUFFER_SIZE 64
#define GPIO_OUTPUT 1
#define GPIO_INPUT 0
#define GPIO_HIGH 1
#define GPIO_LOW 0

typedef int gpio_direction;
typedef int gpio_state;

// Mapping of GPIO number to pin number for ROSCubeX
static std::map<int, int> pin_gpio_mapping{{51, 408}, {52, 350}, {53, 446}, {54, 445}};

int export_gpio(int gpio);
int unexport_gpio(int gpio);
int set_gpio_direction(int gpio, gpio_direction direction);
int set_gpio_state(int gpio, gpio_state state);

int export_gpio_pin(int gpio_pin);
int unexport_gpio_pin(int gpio_pin);
int set_gpio_pin_direction(int gpio_pin, gpio_direction direction);
int set_gpio_pin_state(int gpio_pin, gpio_state state);

#endif  // JETSON_GPIO_H
