# AWS DeepRacer Interfaces Package for Follow Me

## Overview

The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the core AWS DeepRacer application. Here we have extended and modified the package to support Follow Me sample project. These services and messages defined are shared across the packages that are part of the AWS DeepRacer application and Follow Me sample project. For more information about the Follow Me sample project, see [Follow Me sample project](https://github.com/aws-racer/aws-deepracer-follow-me-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the Follow Me sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepracer_inferfaces_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *rosidl_default_generators* - The custom built interfaces rely on rosidl_default_generators for generating language-specific code.
2. *rosidl_default_runtime* - The custom built interfaces rely on rosidl_default_runtime for using the language-specific code.
3. *sensor_msgs* - This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.
4. *std_msgs* - Standard ROS Messages including common message types representing primitive data types and other basic message constructs, such as multiarrays.



## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup script:

        source /opt/ros/foxy/setup.bash

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire Follow Me sample project on the DeepRacer device.

        git clone https://github.com/aws-racer/aws-deepracer-follow-me-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-follow-me-sample-project/deepracer_follow_me_ws/

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-me-sample-project/deepracer_follow_me_ws/ && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-follow-me-sample-project/deepracer_follow_me_ws/ && colcon build --packages-select deepracer_interfaces_pkg