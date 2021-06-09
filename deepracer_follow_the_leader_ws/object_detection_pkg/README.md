# AWS DeepRacer object detection package

## Overview

The object detection ROS package creates the `object_detection_node`, which is responsible for collecting sensor data (camera images) from `sensor_fusion_pkg` and running them through the object-detection model to find a specified object and provide a normalized delta of the found object from the target position. This delta value is published using a ROS publisher as `DetectionDeltaMsg` data. For more information, see the [AWS DeepRacer Follow the Leader(FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer object detection package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the FTL sample project. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer device, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md). 

The `object_detection_pkg` specifically depends on the following ROS 2 packages as build and run dependencies.

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support FTL sample project.

The following are the additional software and hardware requirements to get the `object_detection_node` to work on the AWS DeepRacer device. 

1. **Download and optimize the object-detection model:** Follow the [instructions](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md) to download and optimize the object-detection model and copy it to the required location on the AWS DeepRacer device.

1. **Set up the Intel Neural Compute Stick 2 (optional):** The `object_detection_node` provides functionality to offload the inference to a Intel Neural Compute Stick 2 connected to the AWS DeepRacer device. This is an optional setting that enhances the inference performance of the object-detection model. For more details about running inference on the Movidius NCS (Neural Compute Stick) with OpenVINO™ toolkit, see [this video](https://www.youtube.com/watch?v=XPvMrGobe7I).

Attach the Neural Compute Stick 2 firmly in the back slot of the AWS DeepRacer, open a terminal, and run the following commands as the root user to install the dependencies of the Intel Neural Compute Stick 2 on the AWS DeepRacer device.

1. Switch to the root user:

            sudo su

2. Navigate to the OpenVino installation directory:

            cd /opt/intel/openvino_2021/install_dependencies

3. Set the environment variables required to run the Intel OpenVino scripts:

            source /opt/intel/openvino_2021/bin/setupvars.sh

4. Run the dependency installation script for the Intel Neural Compute Stick:

            ./install_NCS_udev_rules.sh

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

1. Clone the entire FTL sample project on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Fetch the unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `object_detection_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build --packages-select object_detection_pkg deepracer_interfaces_pkg


## Using the `object_detection_node`

Although the **object_detection_node** is built to work with the FTL sample project, you can run it independently for development, testing, and debugging purposes.

### Running the node

Configure the launch file to specify which device to use for inference (for more details, see the extended configuration section below). To launch the built `object_detection_node` as the root user on the AWS DeepRacer device, open another terminal on the device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash and OpenVINO bash script:

        source /opt/ros/foxy/setup.bash 
        source /opt/intel/openvino_2021/bin/setupvars.sh 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash 

1. Launch the `object_detection_node` using the launch script:

        ros2 launch object_detection_pkg object_detection_pkg_launch.py

## Launch files

A launch file called `object_detection_pkg_launch.py` is included in this package that gives an example of how to launch the `object_detection_node`.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='object_detection_pkg',
                    namespace='object_detection_pkg',
                    executable='object_detection_node',
                    name='object_detection_node',
                    parameters=[{
                        'DEVICE': 'MYRIAD/CPU',
                        'PUBLISH_DISPLAY_OUTPUT': True
                    }]
                )
            ])

### Configuration file and parameters

#### `object_detection_node`

| Parameter name   | Description  |
| ---------------- |  ----------- |
| `DEVICE` (optional) | If set as `MYRIAD`, it uses the Intel Compute Stick 2 for inference. Otherwise, it uses CPU for inference by default, even if removed. |
| `PUBLISH_DISPLAY_OUTPUT` | Set to `True` or `False` if the inference output images need to be published to localhost using `web_video_server`.|


## Node details

### `object_detection_node`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/sensor_fusion_pkg/sensor_msg` | `EvoSensorMsg` | This message holds a list of `sensor_msgs` and image objects that are independently collected from different camera sensors. |


#### Published topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `object_detection_delta` | `DetectionDeltaMsg` | Message with object detection normalized error (delta) of the detected object from the target (reference) position with respect to `x` and `y` axes. |
| `detection_display` | Image | Message to display the input stream of images after inference, published to the `local web_video_server`. |

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
* [Instructions for downloading and optimizing the object-detection model](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md)
