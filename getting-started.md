# Follow Me Sample Application

The Follow Me sample project is a sample application built on top of the existing AWS DeepRacer application which uses object detection machine learning model through which the AWS DeepRacer device can identify and follow a person. Explore the Follow Me sample project by cloning the [aws-deepracer-follow-me-sample-project](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project).

The Follow Me application uses many nodes from the AWS DeepRacer core application as is and adds a few specific extension to the shared nodes. This application is built to work alongside the AWS DeepRacer core application so that we can run both of the applications simultaneously.

* **DeepRacer Core Packages used without modification**
    * camera_pkg
    * deepracer_navigation_pkg
    * deepracer_systems_pkg
    * device_info_pkg
    * i2c_pkg
    * inference_pkg
    * model_optimizer_pkg
    * sensor_fusion_pkg
    * servo_pkg
    * status_led_pkg
    * usb_monitor_pkg
* **DeepRacer Core Packages modified to support Follow Me**
    * webserver_pkg
    * ctrl_pkg
    * deepracer_interfaces_pkg
* **Follow Me functionality specific packages**
    * object_detection_pkg
    * follow_me_navigation_pkg


## Hardware Setup

The Follow Me sample project is built to work on **AWS DeepRacer** with a single camera attached to it. Optionally, you can also connect an **Intel Neural Compute Stick 2** to the USB slot at the rear end of the car as depicted to improve the inference performance.

![followme-deepracer](/media/followme-deepracer.png)

## Main Components

There are six packages (ROS Nodes) that are of importance for the Follow Me sample project.
 
1. [Object Detection Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/object_detection_pkg) - Package responsible to detect object (person) from the camera sensor images and calculate the error (delta) in displacement of the object from ideal position to be provided to the follow-me navigation.

1. [Follow Me Navigation Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/follow_me_navigation_pkg) - Package responsible for collecting the delta results from object detection and mapping it to the servo message with throttle and steering angle values.

1. [Follow Me Launcher Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/follow_me_launcher) - The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the core AWS DeepRacer application, but has been modified to support Follow Me sample project.

1. [Control Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/ctrl_pkg) - Package extended from AWS DeepRacer core application and responsible for creating main node with services exposed to be used by webserver backend API calls. This manages the mode of the car: manual, autonomous, calibration or followme.

1. [DeepRacer Interfaces Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/deepracer_interfaces_pkg) - The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the Follow Me sample project.

1. [Webserver Package](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project/tree/main/deepracer_follow_me_ws/webserver_pkg) - Package extended from AWS DeepRacer core application and responsible for creating a collection of FLASK APIs that are called from the front end. These APIs call the required ROS services and return the result to the front end required for Follow Me sample project to interact with the device console.


## Follow me mode:

The Follow Me sample project introduces a new mode (followme mode) of operation in the AWS DeepRacer device apart from the existing modes of operation(autonomous mode, calibration mode and manual mode). More details about the existing modes of operation in the AWS DeepRacer device is found [here](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).  

In the Follow me mode the DeepRacer devices takes the camera image input from the front facing camera connected to the car and runs it through the machine learning model to identify a person and calculate information required to plan its action and follow the person. Similar to the autonomous mode, there is an perception-inference-action step involved here as well, where the inference is done by an object detection model to obtain the bounding box data for a person identified in the image. Each perception-inference-action step involves a pipeline of a series of ROS messages published/subscribed at various nodes, to publish camera image, and then to publish the object detection deltas identifying person’s position and corresponding action data to follow the person.

![followme-flow](/media/followme-flow.png)


### Inference (Decision)

The inference step is handled by the Object Detection ROS package that creates the object_detection_node responsible for collecting sensor data (camera images) from sensor_fusion_pkg and running object detection on specified object. A target center is specified in the object_detection_node that acts as reference to calculate the detection error (delta) whenever an object is detected. As part of this step, the node publishes the normalized delta data from target point as ObjectDetectionDeltaMsg data identifying the person/object’s position. 

For each input image, the node will detect object (person) and get coordinates of center of bounding box and calculate the (x, y) delta of the [current position of detected object - target position] is calculated as shown in the figure below:

![followme-object-detection-bb](/media/followme-object-detection-bb.png)

This delta value is published as the ObjectDetectionDeltaMsg data to /object_detection_pkg/object_detection_delta topic which will be read by the follow me navigation node. If no object is detected in a image, the object_detection_node publishes a zero error (delta) signifying that the DeepRacer is already at the target position and need not move.


### Action (Navigation)

The Follow Me Navigation ROS package creates the follow_me_navigation_node which decides the action / controller message to send out based on the normalized detection error (delta) received from object_detection_node. The node uses a very simple action space to account for the various combinations of the (x, y) delta values that are expected.

![followme-navigation-moves](/media/followme-navigation-moves.png)

Based on the above diagram, we can see that there are 9 different cases to handle with respect to the {delta_x, delta_y} values. These {delta_x, delta_y} values define the difference in the target position to the center of the bounding box in the current image that was run through object detection. We have defined the following simple action space to handle these 9 cases:

|   Case    |    Steering    |    Throttle    |
| --------- | -------------- | -------------- |
|     1     |      Left      |     Forward    |
|     2     |      NULL      |     Forward    |
|     3     |      Right     |     Forward    |
|     4     |      Left      |     NULL       |
|     5     |      NULL      |     NULL       |
|     6     |      Right     |     NULL       |
|     7     |      Right     |     Back       |
|     8     |      NULL      |     Back       |
|     9     |      Left      |     Back       |

It is important to map specific thresholds for the {delta_x, delta_y} values to the actions defined above in order to ensure a safe and meaningful selection of actions. These actual delta values to trigger each of the action from the above action space was defined by empirically collecting the {delta_x, delta_y} value of the object (person standing in front of camera) at different positions with respect to the camera of the DeepRacer device. The grid diagram below shows a top down view of the placement positions with the car with camera placed at the bottom.

![followme-detection-delta-calculation-experiment-placement](/media/followme-detection-delta-calculation-experiment-placement.png)

The average of the changes in x and y (delta_x and delta_y) for over 3 x 20 incidents for each position is shared below. These {delta_x, delta_y} values with respect to the object (person) position from the camera enables us to create a safe distance bracket for valid actions. These brackets are then mapped to the steering and the speed values required by the DeepRacer servo node.

![followme-detection-delta-calculation-experiment-results](/media/followme-detection-delta-calculation-experiment-results.png)

Based on the data collected, we get the following brackets:

**Steering:**

* DEFAULT: No steering, when the object is positioned on the straight line of sight with respect to camera
* SLOW_LEFT: With respect to 15 cm Left
* FAST_LEFT: With respect to 45 cm Left
* SLOW_RIGHT: With respect to 15 cm Right
* FAST_RIGHT: With respect to 45 cm Right

**Speed:**

* DEFAULT: No throttle
* FORWARD
* REVERSE

For every combination of the normalized delta combination in x and y (delta_x and delta_y), based on the above brackets of actions for steering and throttle, an action is planned by the follow_me_navigation_node and published to be picked up by the servo_node when *followme* mode is enabled.

Hence, using this pipeline for Perception - Inference - Action on a loop, the DeepRacer detects a person, plans what action is needed to bring the person at the target position and takes the action for each image it infers on, thus achieving the goal of following a person.


## Possible next steps:

This sample project can be used as a guide to think about more interesting applications by modifying/enhancing the logic and action space used in the follow_me_navigation_node as well as using the following capability to build more applications. This sample project has been designed to give more independence on modifying or adding your own logic and ideas without having to rework everything from scratch.

There are multiple ideas that can be implemented to improve the **“follow”** feature:

* The object detection can be improved by replacing the current model used by a custom model trained to recognize patterns and movements. Low latency and high accuracy models can be trained for specific function of developing a “follow” feature and refine the Perception - Inference - Action pipeline.
* Add-ons like a depth camera can be used to leverage the additional depth information to calculate the distance of object more accurately. The follow_me_navigation_node can be extended to utilize the depth aspect to fine tune the object tracking capability.

The individual nodes used in the Follow Me sample project or the entire sample project can be used to develop something entirely different as well.

* The object detection node can be picked up to run independently as a part of your custom project by changing the object that should to be detected. For example, you can create a security bot which identifies suspicious activity and reports the same by moving safely towards the cause, unlike security cameras which can have blind-spots.
* The follow_me_navigation_node can be modified and picked up to run independently as part of custom project as well to satisfy a different use case, such as to search for an object and notify.
* Another use case to also leverage the Mapping sample project as a part of your custom project can be to imitate a line follower, harnessing capabilities from the object detection and navigation while mapping the surrounding, thus automating the mapping process!


## Summary

The Follow Me sample project leverages most of the concepts used in the AWS DeepRacer application You can learn more about the AWS DeepRacer core application [here](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md).
