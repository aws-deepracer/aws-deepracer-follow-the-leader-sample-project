# AWS DeepRacer Follow the Leader (FTL) launcher package

## Overview

The AWS DeepRacer Follow the Leader (FTL) sample project is an sample application built on top of the existing AWS DeepRacer application, which uses an object-detection machine learning model through which the AWS DeepRacer device can identify and follow a person. For information, see [Getting Started](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer Follow the Leader (FTL) launcher package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the FTL sample project. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md). The FTL sample project requires the AWS DeepRacer application to be installed on the device, because it leverages most of the packages from the core application.

The following are the additional software and hardware requirements to get the FTL sample project to work on the AWS DeepRacer device.

1. **Download and optimize the object-detection model:** Follow the [instructions](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md) to download and optimize the object-detection model and copy it to the required location on the AWS DeepRacer device.

1. **Calibrate the AWS DeepRacer (optional):** Follow the [instructions](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) to calibrate the mechanics of your AWS DeepRacer vehicle so the vehicle performance is optimal and it behaves as expected.

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

## Downloading and Building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Stop the `deepracer-core.service` that is currently running on the device:

        systemctl stop deepracer-core

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

1. Clone the `async_web_server_cpp`, `web_video_server`, and `rplidar_ros` dependency packages on the AWS DeepRacer device:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && ./install_dependencies.sh

1. Fetch the unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the packages in the workspace

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ && colcon build

## Using the FTL sample application
Follow this procedure to use the FTL sample application.

### Running the node

To launch the FTL sample application as the root user on the AWS DeepRacer device, open another terminal on the device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash

1. Launch the nodes required for the FTL sample project:

        ros2 launch ftl_launcher ftl_launcher.py

Once the FTL sample application is launched, you can follow the steps [here](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-set-up-vehicle-test-drive.html) to open the AWS DeepRacer Vehicle's Device Console and checkout the FTL mode tab which will help you control the vehicle.

### Enabling `followtheleader` mode using the CLI

Once the `ftl_launcher` has been kicked off, open a new terminal as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash

1. Set the mode of the AWS DeepRacer via `ctrl_pkg` to `followtheleader` using the following ROS 2 service call:

        ros2 service call /ctrl_pkg/vehicle_state deepracer_interfaces_pkg/srv/ActiveStateSrv "{state: 3}"

1. Enable `followtheleader` mode using the following ROS 2 service call:

        ros2 service call /ctrl_pkg/enable_state deepracer_interfaces_pkg/srv/EnableStateSrv "{is_active: True}"

### Changing the `MAX_SPEED` scale of the AWS DeepRacer

You can modify the `MAX_SPEED` scale of the AWS DeepRacer using a ROS 2 service call in case the car isn’t moving as expected. This can occur because of the vehicle battery percentage, the surface on which the car is operating, or for other reasons.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the FTL workspace:

        cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash

1. Change the `MAX SPEED` to xx% of the `MAX` Scale:

        ros2 service call /ftl_navigation_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.xx}"

    Example: Change the `MAX SPEED` to 75% of the `MAX` Scale:

        ros2 service call /ftl_navigation_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.75}"


## Launch files

The `ftl_launcher.py`, included in this package, is the main launcher file that launches all the required nodes for the FTL sample project. This launcher file also includes the nodes from the AWS DeepRacer core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            ld = LaunchDescription()
            object_detection_node = Node(
                package='object_detection_pkg',
                namespace='object_detection_pkg',
                executable='object_detection_node',
                name='object_detection_node',
                parameters=[{
                    'DEVICE': 'CPU',
                    'PUBLISH_DISPLAY_OUTPUT': True
                }]
                )
            ftl_navigation_node = Node(
                package='ftl_navigation_pkg',
                namespace='ftl_navigation_pkg',
                executable='ftl_navigation_node',
                name='ftl_navigation_node'
                )
            camera_node = Node(
                package='camera_pkg',
                namespace='camera_pkg',
                executable='camera_node',
                name='camera_node',
                parameters=[
                    {'resize_images': False}
                ]
            )
            ctrl_node = Node(
                package='ctrl_pkg',
                namespace='ctrl_pkg',
                executable='ctrl_node',
                name='ctrl_node'
            )
            deepracer_navigation_node = Node(
                package='deepracer_navigation_pkg',
                namespace='deepracer_navigation_pkg',
                executable='deepracer_navigation_node',
                name='deepracer_navigation_node'
            )
            software_update_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='software_update_node',
                name='software_update_node'
            )
            model_loader_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='model_loader_node',
                name='model_loader_node'
            )
            otg_control_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='otg_control_node',
                name='otg_control_node'
            )
            network_monitor_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='network_monitor_node',
                name='network_monitor_node'
            )
            deepracer_systems_scripts_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='deepracer_systems_scripts_node',
                name='deepracer_systems_scripts_node'
            )
            device_info_node = Node(
                package='device_info_pkg',
                namespace='device_info_pkg',
                executable='device_info_node',
                name='device_info_node'
            )
            battery_node = Node(
                package='i2c_pkg',
                namespace='i2c_pkg',
                executable='battery_node',
                name='battery_node'
            )
            inference_node = Node(
                package='inference_pkg',
                namespace='inference_pkg',
                executable='inference_node',
                name='inference_node'
            )
            model_optimizer_node = Node(
                package='model_optimizer_pkg',
                namespace='model_optimizer_pkg',
                executable='model_optimizer_node',
                name='model_optimizer_node'
            )
            rplidar_node = Node(
                package='rplidar_ros2',
                namespace='rplidar_ros',
                executable='rplidar_scan_publisher',
                name='rplidar_scan_publisher',
                parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': True,
                    }]
            )
            sensor_fusion_node = Node(
                package='sensor_fusion_pkg',
                namespace='sensor_fusion_pkg',
                executable='sensor_fusion_node',
                name='sensor_fusion_node'
            )
            servo_node = Node(
                package='servo_pkg',
                namespace='servo_pkg',
                executable='servo_node',
                name='servo_node'
            )
            status_led_node = Node(
                package='status_led_pkg',
                namespace='status_led_pkg',
                executable='status_led_node',
                name='status_led_node'
            )
            usb_monitor_node = Node(
                package='usb_monitor_pkg',
                namespace='usb_monitor_pkg',
                executable='usb_monitor_node',
                name='usb_monitor_node'
            )
            webserver_publisher_node = Node(
                package='webserver_pkg',
                namespace='webserver_pkg',
                executable='webserver_publisher_node',
                name='webserver_publisher_node'
            )
            web_video_server_node = Node(
                package='web_video_server',
                namespace='web_video_server',
                executable='web_video_server',
                name='web_video_server'
            )
            ld.add_action(object_detection_node)
            ld.add_action(ftl_navigation_node)
            ld.add_action(camera_node)
            ld.add_action(ctrl_node)
            ld.add_action(deepracer_navigation_node)
            ld.add_action(software_update_node)
            ld.add_action(model_loader_node)
            ld.add_action(otg_control_node)
            ld.add_action(network_monitor_node)
            ld.add_action(deepracer_systems_scripts_node)
            ld.add_action(device_info_node)
            ld.add_action(battery_node)
            ld.add_action(inference_node)
            ld.add_action(model_optimizer_node)
            ld.add_action(rplidar_node)
            ld.add_action(sensor_fusion_node)
            ld.add_action(servo_node)
            ld.add_action(status_led_node)
            ld.add_action(usb_monitor_node)
            ld.add_action(webserver_publisher_node)
            ld.add_action(web_video_server_node)
            return ld


### Configuration file and parameters

| Parameter name   | Description  |
| ---------------- |  ----------- |
| `DEVICE` (optional) | If set as `MYRIAD`, uses the Intel Compute Stick 2 for inference. Else, uses the CPU for inference by default, even if it is removed. |
| `PUBLISH_DISPLAY_OUTPUT` | Set to `True` or `False` if the inference output images need to be published to localhost using `web_video_server`.|
| `resize_images` | Set to `True` or `False` depending on if you want to resize the images in camera_pkg |


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
* [Instructions for downloading and optimizing the object-detection model](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md)
* [Instructions for calibrating your AWS DeepRacer](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html)
