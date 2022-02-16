/*
 * Copyright 2022 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <jetson_camera_trigger/jetson_gpio.h>
#include <jetson_camera_trigger/jetson_camera_trigger_node.hpp>
#include <std_msgs/Time.h>

JetsonCameraTriggerNode::JetsonCameraTriggerNode()
{
  // Get the triggering parameters
  private_nh_.param("frame_rate", fps_, double(10));
  private_nh_.param("phase", phase_, double(0));
  private_nh_.param("gpio", gpio_, 0);

  if (gpio_ <= 0)
  {
    ROS_WARN_STREAM("No valid trigger GPIO specified. Not using triggering on " << gpio_ << ".");
    use_triggering_ = false;
  }
  else
  {
    use_triggering_ = true;
    InitializeTrigger();
  }
  trigger_time_publisher_ = private_nh_.advertise<std_msgs::Time>("trigger_time", 1000);
}

void JetsonCameraTriggerNode::InitializeTrigger()
{
  if (export_gpio_pin(gpio_) || set_gpio_pin_direction(gpio_, GPIO_OUTPUT))
  {
    ROS_WARN_STREAM("Failed to initialize GPIO trigger. Not using triggering on " << gpio_ << ".");
    use_triggering_ = false;
  }
}

void JetsonCameraTriggerNode::Run()
{
  if (use_triggering_)
  {
    std_msgs::Time trigger_time_msg;
    // Start on the first time after TOS
    int32_t start_nsec;
    int32_t end_nsec;
    int32_t target_nsec;
    int32_t interval_nsec = (int32_t)(1e9 / fps_);
    int32_t offset = 1e3;
    int32_t pulse_width = 5e6;
    int32_t wait_nsec = 0;

    // Fix this later to remove magic numbers
    if (std::abs(phase_) <= 1e-7)
    {
      start_nsec = 0;
    }
    else
    {
      start_nsec = (int32_t)interval_nsec * (phase_ / 360.0);
    }
    target_nsec = start_nsec;
    end_nsec = start_nsec - interval_nsec + 1e9;

    while (ros::ok())
    {
      // Do triggering stuff
      // Check current time - assume ROS uses best clock source
      // Fix this later to remove magic numbers
      ros::Time now = ros::Time::now();
      // Wait until the last 10 millisecond
      if (now.nsec < end_nsec)
      {
        while (now.nsec > target_nsec)
        {
          target_nsec = target_nsec + interval_nsec;
        }
        // FIX: what about very small phases and fast framerates giving a negative number?
        wait_nsec = target_nsec - now.nsec - 1e7;
      }
      else
      {
        target_nsec = start_nsec;
        wait_nsec = 1e9 - now.nsec + start_nsec - 1e7;
      }
      if (wait_nsec > 1e7)
      {
        ros::Duration(0, wait_nsec).sleep();
      }

      // Block the last millisecond
      now = ros::Time::now();
      if (now.nsec < end_nsec)
      {
        while (now.nsec < target_nsec)
        {
          now = ros::Time::now();
        }
      }
      else
      {
        while (now.nsec > end_nsec || now.nsec < start_nsec)
        {
          now = ros::Time::now();
        }
      }
      // Trigger!
      set_gpio_pin_state(gpio_, GPIO_HIGH);
      trigger_time_msg.data = now;
      trigger_time_publisher_.publish(trigger_time_msg);
      ros::Duration(0, pulse_width).sleep();
      set_gpio_pin_state(gpio_, GPIO_LOW);
      target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
    }
  }
  // Cleanup
  unexport_gpio_pin(gpio_);
}
