# AWS DeepRacer control package for the Follow the Leader (FTL) sample project

## Overview

The AWS DeepRacer control ROS package creates the `ctrl_node`, which is part of the Follow the Leader (FTL) sample project and launches from the `ftl_launcher`. This package was extended and modified from the AWS DeepRacer control ROS package developed for the core application. For more information, see the [Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

This is the main node with services exposed to be used by webserver backend API calls. This node in the AWS DeepRacer application manages the different modes of the device [manual, autonomous, calibration]. It allows you to maintain the device in a single mode at any point in time so that the overlapping functionalities (such as servo messages) do not conflict with each other. In this package, an additional `followtheleader` mode has been added to support the FTL sample project.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer control package for the Follow the Leader (FTL) sample project.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the FTL sample project. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `ctrl_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the FTL sample project.
2. `camera_pkg`: The AWS DeepRacer camera ROS package creates the `camera_node`, which is part of the core AWS DeepRacer application.
3. `servo_pkg`: The AWS DeepRacer servo ROS package creates the `servo_node`, which is part of the core AWS DeepRacer application.
4. `inference_pkg`: The AWS DeepRacer inference ROS package creates the `inference_node`, which is part of the core AWS DeepRacer application.
5. `model_optimizer_pkg`: The AWS DeepRacer model optimizer ROS package creates the `model_optimizer_node`, which is part of the core AWS DeepRacer application.
6. `deepracer_navigation_pkg`: The AWS DeepRacer navigation ROS package creates the `deepracer_navigation_node`, which is part of the core AWS DeepRacer application.



## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

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

1. Build the `ctrl_pkg`, `camera_pkg`, `servo_pkg`, `inference_pkg`, `model_optimizer_pkg`, `deepracer_navigation_pkg`, and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build --packages-select ctrl_pkg camera_pkg servo_pkg inference_pkg model_optimizer_pkg deepracer_navigation_pkg deepracer_interfaces_pkg


## Using the control node

The `ctrl_node` provides basic system level functionality for the AWS DeepRacer application and the FTL sample project to work. Although the node is built to work with the AWS DeepRacer application and the FTL sample project, you can run it independently for development, testing, and debugging purposes.

### Running the node

To launch the built `ctrl_node` as the root user on the AWS DeepRacer device, open another terminal on the device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash 

1. Launch the `ctrl_node` using the launch script:

        ros2 launch ctrl_pkg ctrl_pkg_launch.py

## Launch files

The `ctrl_node` provides the core functionality to manage the device's different operation modes. The package also includes `ctrl_pkg_launch.py`, an example of how to launch the nodes independently from the core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node


        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='ctrl_pkg',
                    namespace='ctrl_pkg',
                    executable='ctrl_node',
                    name='ctrl_node'
                )
            ])

## Node details

### `ctrl_node`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/deepracer_navigation_pkg/auto_drive` | `ServoCtrlMsg` | Message with scaled steering angle and throttle data as per action space values sent to the servo package to move the car in `autonomous` mode. |
| `/webserver_pkg/calibration_drive` | `ServoCtrlMsg` | Message with raw PWM values for steering angle and throttle data sent to the servo package to calibrate the car in `calibration` mode. |
| `/webserver_pkg/manual_drive` | `ServoCtrlMsg` | Message with steering angle and throttle data sent to the servo package to move the car in `manual` mode. |
| `/ftl_navigation_pkg/ftl_drive` | `ServoCtrlMsg` | Message with steering angle and throttle data sent to the servo package to move the car in `followtheleader` mode. |

#### Published topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/ctrl_pkg/raw_pwm` | `ServoCtrlMsg` | Publisher to send a message with raw PWM values for steering angle and throttle data sent to the servo package to calibrate the car. |
| `/ctrl_pkg/servo_msg` | `ServoCtrlMsg` | Publisher to send a message with steering angle and throttle data sent to the servo package to move the car. |


#### Service clients

| Service name | Service type | Description |
|--------------| ------------ | ----------- |
| `/model_optimizer_pkg/model_optimizer_server` | `ModelOptimizeSrv` | Client to the `model_optimizer` service that is called to initiate the model optimizing script for the current model selected in `autonomous` mode. |
| `/inference_pkg/load_model` | `LoadModelSrv` | Client to the `load_model` service from the inference package that is called to set the preprocessing and inference task details in `autonomous` mode. |
| `/inference_pkg/inference_state` | `InferenceStateSrv` | Client to the service that is called to enable or disable inference function on the stream of sensor messages in `autonomous` mode. |
| `/servo_pkg/servo_gpio` | `ServoGPIOSrv` | Client to the service that is called to enable or disable the servo GPIO to allow setting the PWM values on the servo and motor. |
| `/servo_pkg/set_calibration` | `SetCalibrationSrv` | Client to set the calibration value for the steering or throttle in `calibration` mode. |
| `/servo_pkg/get_calibration` | `GetCalibrationSrv` | Client to get the current calibration value for the steering or throttle in `calibration` mode. |
| `/servo_pkg/get_led_state` | `GetLedCtrlSrv` | Client to the `get_car_led` service to get the tail light LED channel PWM values in `calibration` mode. |
| `/servo_pkg/set_led_state` | `SetLedCtrlSrv` | Client to the `set_car_led` service to set the tail light LED channel PWM values in `calibration` mode. |
| `/deepracer_navigation_pkg/navigation_throttle` | `NavThrottleSrv` | Client to service that is called set the scale value for the navigation throttle in `autonomous` mode to control the speed of the device. |
| `/deepracer_navigation_pkg/load_action_space` | `LoadModelSrv` | Client to the `load_action_space` service that is called to set the action space for the model in `deepracer_navigation_node` to be used while mapping the inference results to the action values in `autonomous` mode. |


#### Services

| Service name | Service type | Description |
|--------------| ------------ | ----------- |
| `vehicle_state` | `ActiveStateSrv` | Service that is called to set the vehicle mode by deactivating the current vehicle state and set the new mode. |
| `enable_state` | `EnableStateSrv` | Service that is called to activate and deactivate the current vehicle mode. |
| `get_car_cal` | `GetCalibrationSrv` | Service that is called to get the current calibration PWM values [min, mid, max, polarity] for the steering or throttle. |
| `set_car_cal` | `SetLedCtrlSrv` | Service that is called to set the calibration PWM values  [min, mid, max, polarity] for the steering or throttle. |
| `get_car_led` | GetLedCtrlSrv | Service that is called to get the tail light LED [red, green, blue] channel PWM values. |
| `get_ctrl_modes` | `GetCtrlModesSrv` | Service that is called to get the available modes of operation for vehicle. |
| `model_state` | `ModelStateSrv` | Service that is called to run the load model services in background thread. |
| `autonomous_throttle` | `NavThrottleSrv` | Service that is called to set the scale value to multiply to the throttle during autonomous navigation. |
| `is_model_loading` | `GetModelLoadingStatusSrv` | Service that is called to detect if there is a `load_model` operation going on right now on the device in `autonomous` mode. |

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
