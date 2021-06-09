# AWS DeepRacer webserver package for the Follow the Leader (FTL) sample project

## Overview

The AWS DeepRacer webserver ROS package creates the `web_publisher_node`, which is part of the Follow the Leader (FTL) sample project and launches from the `ftl_launcher`. For more information, see the [Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

This node launches a Flask application as a background thread and creates service clients and subscribers for all the services and topics that are required by the APIs called from the AWS DeepRacer vehicle console. This node acts as an interface between the AWS DeepRacer device console and the backend ROS services. This node was extended to provide more functionalities required for the FTL sample project.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer webserver package for the FTL sample project.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the FTL sample project. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer device, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `webserver_pkg` specifically depends on the following ROS 2 packages as build and run dependencies.

1. `deepracer_interfaces_pkg*`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the FTL sample project.
2. `ctrl_pkg`: The AWS DeepRacer control ROS package creates the `ctrl_node`, which is part of the core AWS DeepRacer application, modified to support the FTL sample project.
3. `sensor_fusion_pkg`: The AWS DeepRacer sensor fusion ROS package creates the `sensor_fusion_node`, which is part of the core AWS DeepRacer application.
4. `deepracer_systems_pkg`: The AWS DeepRacer systems ROS package creates the `software_update_node`, `model_loader_node`, `network_monitor_node`, and `otg_control_node`, which are part of the core AWS DeepRacer application.
5. `device_info_pkg`: The AWS DeepRacer device info optimizer ROS package creates the `device_info_node`, which is part of the core AWS DeepRacer application.
6. `i2c_pkg`: The AWS DeepRacer I2C ROS package creates the `battery_node`, which is part of the core AWS DeepRacer application.


## Downloading and building

Open a terminal on the DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire FTL sample project on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Fetch the unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `webserver_pkg`, `ctrl_pkg`, `sensor_fusion_pkg`, `deepracer_systems_pkg`, `device_info_pkg`, `i2c_pkg`, and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build --packages-select webserver_pkg ctrl_pkg sensor_fusion_pkg deepracer_systems_pkg device_info_pkg i2c_pkg deepracer_interfaces_pkg


## Using the `webserver_publisher_node`

The `webserver_publisher_node` provides basic system-level functionality for the AWS DeepRacer application and the FTL sample project to work. Although the node is built to work with the AWS DeepRacer application and the FTL sample project, it can be run independently for development, testing, and debugging purposes.

### Running the node

To launch the built `webserver_publisher_node` as the root user on the AWS DeepRacer device, open another terminal on the device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash 

1. Launch the `webserver_publisher_node` using the launch script:

        ros2 launch webserver_pkg webserver_pkg_launch.py

## Launch files

The `webserver_publisher_node` provides the core functionality to launch the FLASK server and respond to the FLASK API calls. The `webserver_pkg_launch.py` included in this package provides an example demonstrating how to launch the nodes independently from the core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node


        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='webserver_pkg',
                    namespace='webserver_pkg',
                    executable='webserver_publisher_node',
                    name='webserver_publisher_node'
                )
            ])

## Node details

### `webserver_publisher_node`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/deepracer_systems_pkg/software_update_pct` | `SoftwareUpdatePctMsg` | Message with the latest software update percentage and status. |


#### Published topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/webserver_pkg/calibration_drive` | `ServoCtrlMsg` | Publishes a message with raw PWM values for steering angle and throttle data sent to the servo package to calibrate the car. |
| `/webserver_pkg/manual_drive` | `ServoCtrlMsg` | Publishes a message with steering angle and throttle data sent to the servo package to move the car. |


#### Service clients

| Service name | Service type | Description |
|--------------| ------------ | ----------- |
| `/ctrl_pkg/vehicle_state` | `ActiveStateSrv` | Client to the `vehicle_state` service to deactivate the current vehicle state and prepare the new mode. |
| `/ctrl_pkg/enable_state` | `EnableStateSrv` | Client to the `enable_state` service to activate and deactivate the current vehicle mode. |
| `/ctrl_pkg/get_car_cal` | `GetCalibrationSrv` | Client to the `get_calibration` service to get the current calibration value for the steering or throttle. |
| `/ctrl_pkg/set_car_cal` | `SetCalibrationSrv` | Client to the `set_calibration` service to set the current calibration value for the steering or throttle. |
| `/device_info_pkg/get_device_info` | `GetDeviceInfoSrv` | Client to the `get_device_info` service to get the hardware and software version of the AWS DeepRacer device and packages. |
| `/i2c_pkg/battery_level` | `BatteryLevelSrv` | Client to the `battery_level` service to get the current vehicle battery level ranging from [0 to 11]. |
| `/sensor_fusion_pkg/sensor_data_status` | `SensorStatusCheckSrv` | Client to the `sensor_data_status` service to get the sensor connection status for single camera, stereo camera, and LiDAR. |
| `/ctrl_pkg/set_car_led` | `SetLedCtrlSrv` | Client to the `set_car_led` service to set the tail light LED values. |
| `/ctrl_pkg/get_car_led` | `GetLedCtrlSrv` | Client to the `get_car_led` service to get the tail light LED values. |
| `/ctrl_pkg/get_ctrl_modes` | `GetCtrlModesSrv` | Client to get the available modes of operation for vehicle. |
| `/deepracer_systems_pkg/verify_model_ready` | `VerifyModelReadySrv` | Client to the `verify_model_service` to validate if the extraction and installation of the model was successful. |
| `/sensor_fusion_pkg/configure_lidar` | `LidarConfigSrv` | Client to the `configure_lidar` service to dynamically configure the preprocessing details for the LiDAR data before publishing as part of sensor message. |
| `/ctrl_pkg/model_state` | `ModelStateSrv` | Client to the `model_state` service to execute the load model services in background thread. |
| `/ctrl_pkg/is_model_loading` | `GetModelLoadingStatusSrv` | Client to the `is_model_loading` service to know if there is a load model operation going on right now on the device. |
| `/deepracer_systems_pkg/console_model_action` | `ConsoleModelActionSrv` | Client to the `get_otg_link` state service to get the current connection status of micro-USB cable to the AWS DeepRacer device. |
| `/deepracer_systems_pkg/software_update_check` | `SoftwareUpdateCheckSrv` | Client to the `software_update_check` service to find out if there is a software update available for the `aws-deepracer` packages. |
| `/deepracer_systems_pkg/begin_update` | `BeginSoftwareUpdateSrv` | Client to the `begin_update_service` to trigger the update of the `aws-deepracer` Debian packages to the latest software version available. |
| `/deepracer_systems_pkg/software_update_state` | `SoftwareUpdateStateSrv` | Client to the `software_update_state` service to get the current software update state from the states [ UPDATE_UNKNOWN, UP_TO_DATE, UPDATE_AVAILABLE, UPDATE_PENDING, UPDATE_IN_PROGRESS ]. |
| `/ctrl_pkg/autonomous_throttle` | `NavThrottleSrv` | Client to the `autonomous_throttle` service to set the scale value to multiply to the throttle during autonomous navigation. |
| `/ftl_navigation_pkg/set_max_speed` | `SetMaxSpeedSrv` | Client to set the FTL max speed percentage scale value during FTL navigation. |
| `/deepracer_systems_pkg/get_otg_link_state` | `OTGLinkStateSrv` | Client to the `get_otg_link_state` service to get the current connection status of micro-USB cable to the AWS DeepRacer device. |

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
