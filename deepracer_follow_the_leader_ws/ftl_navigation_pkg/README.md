# AWS DeepRacer Follow the Leader (FTL) navigation package

## Overview

The AWS DeepRacer Follow the Leader (FTL) navigation ROS package creates the `ftl_navigation_node`, which decides the action and controller messages to send out using the normalized detection error (delta) received from the `object_detection_node`. For more information, see the [AWS DeepRacer Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer Follow the Leader (FTL) navigation package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the FTL sample project. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `ftl_navigation_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the FTL sample project.

### Downloading and building

Open up a terminal on the AWS DeepRacer device and run the following commands as the root user.

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

1. Build the `ftl_navigation_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build --packages-select ftl_navigation_pkg deepracer_interfaces_pkg


## Using the `ftl_navigation_mode`

Although the **ftl_navigation_node** is built to work with the FTL sample project, you can run it independently for development, testing, and debugging purposes.

### Running the node

To launch the built `ftl_navigation_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the RO S2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash 

1. Launch the `ftl_navigation_node` using the launch script:

        ros2 launch ftl_navigation_pkg ftl_navigation_pkg_launch.py

## Launch files

The `ftl_navigation_pkg_launch.py` launch file included in this package gives an example of how to launch the `ftl_navigation_node`.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='ftl_navigation_pkg',
                    namespace='ftl_navigation_pkg',
                    executable='ftl_navigation_node',
                    name='ftl_navigation_node'
                )
            ])


## Node details

### `ftl_navigation`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
|`/object_detection_pkg/object_detection_delta`|`DetectionDeltaMsg`|Message with Object Detection normalized error (delta) of the detected object from the target (reference) position with respect to `x` and `y` axes.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|`ftl_drive`|`ServoCtrlMsg`|This message sends motor throttle and servo steering angle ratios with respect to the device calibration. It can also be used to send raw PWM values for angle and throttle.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`set_max_speed`|`SetMaxSpeedSrv`|Sets the max speed percentage scale for the FTL application.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)