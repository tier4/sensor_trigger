# sensor_trigger

## Introduction

This repository provides a ROS2 package for generating sensor trigger signals on a Jetson AGX Xavier. It is specifically designed for use with the ADLINK ROSCube RQX-58G, providing pin mapping for the GPIO which are internally connected to the FPGA which controls FSYNC for the built-in GMSL deserializers.

## Requirements

- ECU: Jetson Xavier AGX from NVIDIA Corp.
  - The software in this package is pre-configured for use with ROSCube RQX-58G from ADLINK, although it can be modified to run hardware triggers by using different GPIO on any Jetson Xavier AGX based ECU. The remainder of this document assumes that the user is working with an RQX-58G.
- OS: Ubuntu 18.04 or higher
- ROS2: Galactic Geochelone
  - With current BSP versions, Ubuntu 18.04 must be run on Jetson Xavier AGX within a docker environment. Please refer to [tier4/perception_ecu_container](https://github.com/tier4/perception_ecu_container) for how to prepare the OS and ROS2 environment.

## Installation

1. Create or change into your workspace directory, and execute the following:

   ```bash
   mkdir -p src
   git clone git@github.com:tier4/sensor_trigger.git src
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_trigger
   ```

2. The sensor trigger node requires as close to real-time operation as possible to maintain reliable trigger timing under heavy CPU load. If this is required, it is recommended to allow thread schedule priority setting to the ROS2 user by adding the following line to `/etc/security/limits.conf`:

   ```
   <username>    -   rtprio   98
   ```

   After saving the edited file (as sudo), a reboot will be required.

   Some notes about thread scheduling:

   - If you do not make the settings in (2), the node will run but the timing thread will not be scheduled at any higher priority that other user threads and interruption may occur, resulting in high jitter in the trigger output.
   - When running in a ROS2 docker, the settings in (2) are not required as the docker user is by default the root user.

## Usage

The sensor trigger node will output trigger pulses on the specified GPIO pin at the configured frequency.
The phase parameter is used to control the timing of the pulse relative to Top of Second (ToS), allowing fine-grained control of the trigger timing relative to other sensors and ECUs when all connected devices have been synchronized using Precision Time Protocol (PTP).

Setting the CPU core for operation is recommended to improve timing stability. As CPU 0 is used for system interrupts, a CPU core >0 is recommended. If multiple sensor trigger nodes are instantiated, make sure they each use a different CPU core. You can check how many CPU cores are available on your system with `nproc --all`.

The node can be launched with the default parameters as follows:

```bash
ros2 launch sensor_trigger sensor_trigger.launch
```

## Inputs / Outputs

### Input

This node does not take any inputs.

### Output

| Name             | Type                            | Description                                              |
| ---------------- | ------------------------------- | -------------------------------------------------------- |
| `~/trigger_time` | `builtin_interfaces::msg::Time` | Time that the trigger output was requested (system time) |

## Parameters

### Core Parameters

| Name             | Type   | Description                                                                           |
| ---------------- | ------ | ------------------------------------------------------------------------------------- |
| `gpio`           | int    | Output GPIO pin - see below for assigned pins on RQX-58G                              |
| `phase`          | double | Desired phase of the trigger relative to ToS (Top of Second)                          |
| `frame_rate`     | double | Desired frequency of the trigger in Hz                                                |
| `cpu_core_id`    | int    | Desired CPU core for execution\*                                                      |
| `pulse_width_ms` | int    | Desired pulse width (the time period in milliseconds for the GPIO pin to remain high) |

\*This is indexed from CPU core 0 (which will be CPU 1 in `htop`).

### Included Pin Mappings (RQX-58G)

| Pin | GPIO (sysfs) | Description                                                             |
| --- | ------------ | ----------------------------------------------------------------------- |
| 5   | 216          | Pin 5 on the RQX-58G's DB50 connector, run by GPIO chip 216             |
| 51  | 408          | Pin 51 on the RQX-58G, internally connected to FSYNC on deserializer #1 |
| 52  | 350          | Pin 52 on the RQX-58G, internally connected to FSYNC on deserializer #2 |
| 53  | 446          | Pin 53 on the RQX-58G, internally connected to FSYNC on deserializer #3 |
| 54  | 445          | Pin 54 on the RQX-58G, internally connected to FSYNC on deserializer #4 |

## Related Repositories

- [tier4/perception_ecu_container](https://github.com/tier4/perception_ecu_container)
  - Meta-repository containing `.repos` file to construct perception-ecu workspace
- [tier4/perception_ecu_launch](https://github.com/tier4/perception_ecu_launch.git)
- [tier4/ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera.git)
- [tier4/perception_ecu_individual_params](https://github.com/tier4/perception_ecu_individual_params)
- [autowarefoundation/autoware.universe](https://github.com/autowarefoundation/autoware.universe.git)
