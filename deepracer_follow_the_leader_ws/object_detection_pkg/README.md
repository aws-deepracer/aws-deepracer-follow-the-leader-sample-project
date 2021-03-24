# AWS DeepRacer Object Detection Package

## Overview

The Object Detection ROS package creates the object_detection_node which is responsible for collecting sensor data (camera images) from sensor_fusion_pkg and running them through the object detection model to find a specified object and providing normalized delta of the found object from target position. This delta value is published using a ROS publisher as ObjectDetectionDeltaMsg data. For more information about the Follow the Leader(FTL) sample project, see [Follow the Leader(FTL) sample project](https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the Follow the Leader(FTL) sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The object_detection_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support Follow the Leader(FTL) sample project.

The following are the additional software and hardware requirements to get the object_detection_node to work on the AWS DeepRacer device. 

1. **Download and Optimize the object detection model:** Follow the [instructions](https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md) to download and optimize the object detection model and copy it to required location on the AWS DeepRacer device.

1. **Setup Intel Neural Compute Stick 2 (optional):** The object_detection_node provides functionality to offload the inference to a Intel Neural Compute Stick 2 connected to the AWS DeepRacer device. This is an optional setting that is included to enhance the inference performance of the object detection model. More details about running Inference on the Movidius NCS (Neural Compute Stick) with OpenVINO™ toolkit can be found here: https://www.youtube.com/watch?v=XPvMrGobe7I

    Attach the Neural Compute Stick 2 firmly in the back slot of the AWS DeepRacer, and open up a terminal and run the following commands as root user to install the dependencies of the Intel Neural Compute Stick 2 on the AWS DeepRacer device:

    1. Navigate to the OpenVino installation directory:

            cd /opt/intel/openvino_2021.1.110/install_dependencies

    1. Run the dependency installation script for Intel Neural Compute Stick:

            ./install_NCS_udev_rules.sh


## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire Follow the Leader(FTL) sample project on the DeepRacer device.

        git clone https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the object_detection_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build --packages-select object_detection_pkg deepracer_interfaces_pkg


## Usage

Although the **object_detection_node** is built to work with the Follow the Leader(FTL) sample project, it can be run independently for development/testing/debugging purposes.

### Run the node

Configure the launch file to specify which device to use for inference (for more details, see the extended configuration section below). To launch the built object_detection_node as root user on the AWS DeepRacer device, open up another terminal on the device and run the following commands as root user:

1. Navigate to the Follow the Leader(FTL) workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS2 Foxy setup bash and OpenVINO bash script:

        source /opt/ros/foxy/setup.bash 
        source /opt/intel/openvino_2021/bin/setupvars.sh 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash 

1. Launch the object_detection_node using the launch script:

        ros2 launch object_detection_pkg object_detection_pkg_launch.py

## Launch Files

A launch file called object_detection_pkg_launch.py is included in this package that gives an example of how to launch the object_detection_node.

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

### Configuration File and Parameters

#### object_detection_node

| Parameter Name   | Description  |
| ---------------- |  ----------- |
| DEVICE (optional) | If set as MYRIAD, will use the Intel Compute Stick 2 for inference. Else uses CPU for inference by default, even if removed. |
| PUBLISH_DISPLAY_OUTPUT | Set to True/False if the inference output images need to be published to localhost using web_video_server.|


## Node Details

### object_detection_node

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| /sensor_fusion_pkg/sensor_msg | EvoSensorMsg | This message holds a list of sensor_msgs/Image objects that are independently collected from different camera sensors. |


#### Published Topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| object_detection_delta | ObjectDetectionDeltaMsg | Message with Object Detection normalized error (delta) of the detected object from the target (reference) position with respect to x and y axes. |
| detection_display | Image | Message to display the input stream of images after inference, published to the local web_video_server. |

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Follow the Leader(FTL) sample project getting started: [https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
* Instructions to download and optimize the object detection model: [https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md](https://github.com/awsdeepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md)
