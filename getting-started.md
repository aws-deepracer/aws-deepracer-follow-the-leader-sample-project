# Follow the Leader (FTL) sample application

The Follow the Leader (FTL) sample project is a sample application built on top of the existing AWS DeepRacer application. It uses an object-detection machine learning model, through which the AWS DeepRacer device can identify and follow a person. Explore the FTL sample project by cloning the [aws-deepracer-follow-the-leader-sample-project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

The FTL application uses nodes from the AWS DeepRacer core application as is and adds specific extensions to the shared nodes. This application is built to work alongside the AWS DeepRacer core application so you can run both of the applications simultaneously.

* **AWS DeepRacer core packages used without modification**
    * `camera_pkg`
    * `deepracer_navigation_pkg`
    * `deepracer_systems_pkg`
    * `device_info_pkg`
    * `i2c_pkg`
    * `inference_pkg`
    * `model_optimizer_pkg`
    * `sensor_fusion_pkg`
    * `servo_pkg`
    * `status_led_pkg`
    * `usb_monitor_pkg`
* **AWS DeepRacer core packages modified to support FTL**
    * `webserver_pkg`
    * `ctrl_pkg`
    * `deepracer_interfaces_pkg`
* **FTL functionality–specific packages**
    * `object_detection_pkg`
    * `ftl_navigation_pkg`


## Hardware setup

The FTL sample project is built to work on **AWS DeepRacer** with a single camera attached to it. Optionally, you can also connect an **Intel Neural Compute Stick 2** to the USB slot at the rear end of the car as depicted to improve the inference performance.

<p align="center">
<img src="/media/ftl-deepracer.png" height="450" >
</p>

## Main components

There are six packages (ROS Nodes) that are of importance for the FTL sample project.
 
1. [Object detection package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/object_detection_pkg): This package is responsible for detecting an object (person) from the camera sensor images and calculating the error (delta) in displacement of the object from an ideal position to be provided to the `ftl-navigation`.

1. [Follow the Leader (FTL) navigation package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/ftl_navigation_pkg): This package is responsible for launching all the required nodes for the FTL sample project. This launcher file also includes the launch setup for nodes from the AWS DeepRacer core application.

1. [Follow the Leader (FTL) launcher package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/ftl_launcher): The Follow the Leader (FTL) launcher package is responsible for launching all the required nodes for the FTL sample project. This launcher file also includes the launch setup for nodes from the AWS DeepRacer core application.

1. [Control package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/ctrl_pkg): This package is extended from the AWS DeepRacer core application and responsible for creating a main node with exposed services to be used by webserver backend API calls. This manages the mode of the car: `manual`, `autonomous`, `calibration`, or `followtheleader`.

1. [AWS DeepRacer interfaces package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/deepracer_interfaces_pkg): This is a foundational package that creates the custom service and message types that are used in the FTL sample project.

1. [Webserver package](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/webserver_pkg): This package is extended from the AWS DeepRacer core application and responsible for creating a collection of FLASK APIs that are called from the front end. These APIs call the required ROS services and return the result to the front end required for the FTL sample project to interact with the device console.


## `Followtheleader` mode

The FTL sample project introduces a new mode (`followtheleader`) of operation in the AWS DeepRacer device apart from the existing modes of operation (`autonomous` mode, `calibration` mode, and `manual` mode). For more details about the existing modes of operation in the AWS DeepRacer device, see  [Modes of operation](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).

In `followtheleader` mode, the AWS DeepRacer device takes the camera image input from the front-facing camera connected to the car and runs it through the machine learning model to identify a person, calculate information required to plan its action, and follow the person. Similar to `autonomous` mode, `followtheleader` mode has a perception-inference-action step, in which an object-detection model does the inference to obtain the bounding box data for a person identified in the image. Each perception-inference-action step involves a pipeline of a series of ROS messages published and subscribed at various nodes to publish the camera image, and then to publish the object detection deltas identifying the person’s position and corresponding action data to follow the person.

![ftl-flow](/media/ftl-flow.png)


### Inference (decision)

The object detection ROS package handles the inference step, creating the `object_detection_node` responsible for collecting sensor data (camera images) from `sensor_fusion_pkg` and running object detection on the specified object. The `object_detection_node` specifies a target center that acts as reference to calculate the detection error (delta) whenever it detects an object. As part of this step, the node publishes the normalized delta data from target point as `DetectionDeltaMsg` data identifying the person or object’s position. 

For each input image, the node detects an object or person, gets the coordinates of the center of the bounding box, and calculates the (x, y) delta of the [current position of detected object - target position]. The delta is calculated as shown in the following figure:

<p align="center">
<img src="/media/ftl-object-detection-bb.png" height="450" >
</p>

This delta value is published as the `DetectionDeltaMsg` data to the `/object_detection_pkg/object_detection_delta` topic, which the FTL navigation node reads. If no object is detected in a image, the `object_detection_node` publishes a zero error (delta), signifying that the AWS DeepRacer is already at the target position and need not move.


### Action (navigation)

The FTL navigation ROS package creates the `ftl_navigation_node`, which decides the action or controller message to send out based on the normalized detection error (delta) received from the `object_detection_node`. The node uses a very simple action space to account for the various combinations of the expected (`x`, `y`) delta values.

<p align="center">
<img src="/media/ftl-navigation-moves.png" height="450" >
</p>

In the preceding diagram, there are 9 different cases to handle with respect to the {delta_x, delta_y} values. These {delta_x, delta_y} values define the difference in the target position to the center of the bounding box in the current image that was run through object detection. We have defined the following simple action space to handle these 9 cases:

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

It is important to map specific thresholds for the {delta_x, delta_y} values to the actions defined above in order to ensure a safe and meaningful selection of actions. These actual delta values to trigger each of the actions from the preceding action space are defined by empirically collecting the {delta_x, delta_y} value of the object (person standing in front of camera) at different positions with respect to the camera of the AWS DeepRacer device. The following grid diagram shows a top-down view of the placement positions with the car with camera placed at the bottom.

<p align="center">
<img src="/media/ftl-detection-delta-calculation-experiment-placement.png" height="450" >
</p>


The following image shows the average of the changes in `x` and `y` (`delta_x` and `delta_y`) for over 3 x 20 incidents for each position. These {delta_x, delta_y} values, with respect to the object (person) position from the camera, enable us to create a safe distance bracket for valid actions. These brackets are then mapped to the steering and the speed values required by the AWS DeepRacer `servo_node`.

<p align="center">
<img src="/media/ftl-detection-delta-calculation-experiment-results.png" height="450" >
</p>

Based on the data collected, we get the following brackets:

**Steering**

* DEFAULT: No steering, when the object is positioned on the straight line of sight with respect to camera
* SLOW_LEFT: With respect to 15 cm left
* FAST_LEFT: With respect to 45 cm left
* SLOW_RIGHT: With respect to 15 cm right
* FAST_RIGHT: With respect to 45 cm right

**Speed**

* DEFAULT: No throttle
* FORWARD
* REVERSE

For every combination of the normalized delta combination in `x` and `y` (`delta_x` and `delta_y`), based on the preceding brackets of actions for steering and throttle, the `ftl_navigation_node` plans and publishes an action for the `servo_node` to pick up when `followtheleader` mode is enabled.

Using this pipeline for perception-inference-action on a loop, the AWS DeepRacer detects a person, plans what action is needed to bring the person to the target position, and takes the action for each image on which it infers, thus achieving the goal of following a person.


## Demo

<p align="center">
<img src="/media/ftl-demo.gif" height="450" >
</p>


## Possible next steps

This sample project can be used as a guide to think about more interesting applications by modifying or enhancing the logic and action space used in the `ftl_navigation_node`, as well as using the following capability to build more applications. This sample project has been designed to give you more independence in modifying or adding your own logic and ideas without having to rework everything from scratch.

You can implement multiple ideas to improve the **“follow”** feature:

* You can improve the object detection by replacing the current model used by a custom model trained to recognize patterns and movements. You can train low latency and high accuracy models for the specific function of developing a “follow” feature, and refine the perception-inference-action pipeline.
* You can use add-ons like a depth camera to leverage additional depth information to more accurately calculate the distance of an object. You can extend the `ftl_navigation_node` to use the depth aspect to fine-tune the object tracking capability.

You can use the individual nodes used in the FTL sample project or the entire sample project to develop something entirely different as well.

* You can pick up the object detection node to run independently as a part of your custom project by changing the object that should to be detected. For example, you can create a security bot which identifies suspicious activity and reports it by moving safely towards the cause, unlike security cameras which can have blind spots.
* You can modify the `ftl_navigation_node` and pick it up to run independently as part of custom project, or satisfy a different use case, such as searching for an object and notifying when it's found.
* You can leverage the Mapping sample project as a part of your custom project to imitate a line follower, harnessing capabilities from object detection and navigation while mapping the surroundings, thus automating the mapping process.


## Summary

The Follow the Leader (FTL) sample project leverages most of the concepts used in the AWS DeepRacer application. To learn more about the AWS DeepRacer core application, see [Getting started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [AWS DeepRacer device modes of operation](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).
