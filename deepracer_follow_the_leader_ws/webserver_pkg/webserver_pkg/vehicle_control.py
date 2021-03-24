#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
vehicle_control.py

This is the module that holds the APIs and utility functions required to capture
the action performed by user from the UI regarding the throttle, angle etc in the
manual drive and autonomous mode, and trigger appropriate service calls to move
the servo/motor.
"""

import math
from flask import (Blueprint,
                   jsonify,
                   request)

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import (ActiveStateSrv,
                                          EnableStateSrv,
                                          NavThrottleSrv,
                                          SetMaxSpeedSrv,
                                          GetCtrlModesSrv)
from webserver_pkg import constants
from webserver_pkg.utility import (api_fail,
                                   call_service_sync)
from webserver_pkg import webserver_publisher_node


VEHICLE_CONTROL_BLUEPRINT = Blueprint("vehicle_control", __name__)


def get_rescaled_manual_speed(categorized_throttle, max_speed_pct):
    """Return the non linearly rescaled speed value based on the max_speed_pct.

    Args:
        categorized_throttle (float): Float value ranging from -1.0 to 1.0.
        max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                               from maximum speed input.
    Returns:
        float: Categorized value of the input speed.
    """
    """
    Example return values:
    categorized_throttle: -0.3 max_speed_pct: 0.1 mapped_speed: -0.1383742911153119
    categorized_throttle: -0.5 max_speed_pct: 0.1 mapped_speed: -0.224952741020794
    categorized_throttle: -0.8 max_speed_pct: 0.1 mapped_speed: -0.3463137996219282

    categorized_throttle: -0.3 max_speed_pct: 0.5 mapped_speed: -0.208
    categorized_throttle: -0.5 max_speed_pct: 0.5 mapped_speed: -0.33333333333333337
    categorized_throttle: -0.8 max_speed_pct: 0.5 mapped_speed: -0.5013333333333334

    categorized_throttle: -0.3 max_speed_pct: 1.0 mapped_speed: -0.552
    categorized_throttle: -0.5 max_speed_pct: 1.0 mapped_speed: -0.8
    categorized_throttle: -0.8 max_speed_pct: 1.0 mapped_speed: -0.992

    categorized_throttle: 0.3 max_speed_pct: 0.1 mapped_speed: 0.1383742911153119
    categorized_throttle: 0.5 max_speed_pct: 0.1 mapped_speed: 0.224952741020794
    categorized_throttle: 0.8 max_speed_pct: 0.1 mapped_speed: 0.3463137996219282

    categorized_throttle: 0.3 max_speed_pct: 0.5 mapped_speed: 0.208
    categorized_throttle: 0.5 max_speed_pct: 0.5 mapped_speed: 0.33333333333333337
    categorized_throttle: 0.8 max_speed_pct: 0.5 mapped_speed: 0.5013333333333334

    categorized_throttle: 0.3 max_speed_pct: 1.0 mapped_speed: 0.552
    categorized_throttle: 0.5 max_speed_pct: 1.0 mapped_speed: 0.8
    categorized_throttle: 0.8 max_speed_pct: 1.0 mapped_speed: 0.992
    """
    # return 0.0 if categorized_throttle or maximum speed pct is 0.0
    if categorized_throttle == 0.0 or max_speed_pct == 0.0:
        return 0.0

    # get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
    # The lower the update_speed_scale_value parameter, higher the impact on the
    # final mapped_speed.
    # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
    # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
    # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
    #     max_speed_pct = 1.0; update_speed_scale_value = 1
    # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
    # higher possible values.
    #   Example: update_speed_scale_value = 1.0;
    #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
    # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
    # lower possible values.
    #   Example: update_speed_scale_value = 3.0;
    #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

    inverse_max_speed_pct = (1 - max_speed_pct)
    update_speed_scale_value = \
        constants.MANUAL_SPEED_SCALE_BOUNDS[0] + \
        inverse_max_speed_pct * \
        (constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0])
    speed_mapping_coefficients = dict()

    # recreate the mapping coefficeints for the non-linear equation ax^2 + bx based on
    # the update_speed_scale_value.
    # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
    # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
    speed_mapping_coefficients["a"] = \
        (1.0 / update_speed_scale_value**2) * \
        (2.0 * constants.DEFAULT_SPEED_SCALES[0] - 4.0 * constants.DEFAULT_SPEED_SCALES[1])
    speed_mapping_coefficients["b"] = \
        (1.0 / update_speed_scale_value) * \
        (4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0])
    return math.copysign(1.0, categorized_throttle) * \
        (speed_mapping_coefficients["a"] * abs(categorized_throttle)**2 +
         speed_mapping_coefficients["b"] * abs(categorized_throttle))


def get_categorized_manual_throttle(throttle):
    """Return the value of the category in which the input throttle belongs to.

    Args:
        input (float): Float value ranging from -1.0 to 1.0 taken as input from joystick.

    Returns:
        float: Value of the category the input throttle belonged to.
    """
    if abs(throttle) >= 0.8:
        throttle = math.copysign(0.8, throttle)
    elif abs(throttle) >= 0.5:
        throttle = math.copysign(0.5, throttle)
    elif abs(throttle) > 0.0:
        throttle = math.copysign(0.3, throttle)
    return throttle


def get_categorized_manual_angle(angle):
    """Return the value of the category in which the input angle belongs to.

    Args:
        input (float): Float value ranging from -1.0 to 1.0 taken as input from joystick.

    Returns:
        float: Value of the category the input angle belonged to.
    """
    if abs(angle) >= 0.8:
        angle = math.copysign(0.8, angle)
    elif abs(angle) >= 0.5:
        angle = math.copysign(0.5, angle)
    elif abs(angle) > 0:
        angle = math.copysign(0.3, angle)
    return angle


@VEHICLE_CONTROL_BLUEPRINT.route("/api/manual_drive", methods=["PUT", "POST"])
def api_manual_drive():
    """API that publishes control messages to control the angle and throttle in
       manual drive mode.

    Returns:
        dict: Execution status if the API call was successful.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    angle = request.json.get("angle")
    throttle = request.json.get("throttle")
    max_speed = request.json.get("max_speed")

    if angle is None:
        return api_fail("angle is required")
    if throttle is None:
        return api_fail("throttle is required")
    if max_speed is None:
        return api_fail("max_speed is required")

    if angle < -1.0 or angle > 1.0:
        return api_fail("angle out of range")
    if throttle < -1.0 or throttle > 1.0:
        return api_fail("throttle out of range")

    webserver_node.get_logger().info(f"Angle: {angle}  Throttle: {throttle}")

    # Create the servo message.
    msg = ServoCtrlMsg()
    # bound the throttle value based on the categories defined
    msg.angle = -1.0 * get_categorized_manual_angle(angle)
    categorized_throttle = get_categorized_manual_throttle(throttle)
    msg.throttle = -1.0 * get_rescaled_manual_speed(categorized_throttle, max_speed)
    webserver_node.pub_manual_drive.publish(msg)
    return jsonify({"success": True})


@VEHICLE_CONTROL_BLUEPRINT.route("/api/drive_mode", methods=["PUT", "POST"])
def api_set_drive_mode():
    """API to toggle the drive mode between Autonomous/Manual/Follow the Leader(FTL) mode.

    Returns:
        dict: Execution status if the API call was successful and error
              reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    drive_mode = request.json.get("drive_mode")
    if drive_mode is None:
        return jsonify({"success": False, "reason": "drive_mode must be set."})

    webserver_node.get_logger().info(f"Changed the vehicle state to {drive_mode}")
    if drive_mode == "manual":
        drive_mode_state = 0
    elif drive_mode == "auto":
        drive_mode_state = 1
    else:
        # Follow the Leader(FTL) mode.
        drive_mode_state = 3

    try:
        vehicle_state_req = ActiveStateSrv.Request()
        vehicle_state_req.state = drive_mode_state
        vehicle_state_res = call_service_sync(webserver_node.vehicle_state_cli,
                                              vehicle_state_req)
        if vehicle_state_res and (vehicle_state_res.error == 0):
            return jsonify(success=True)
        else:
            webserver_node.get_logger().error("Vehicle state service call failed")
            return jsonify(success=False, reason="Error")

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach vehicle state server: {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to reach vehicle state server."})


@VEHICLE_CONTROL_BLUEPRINT.route("/api/start_stop", methods=["PUT", "POST"])
def api_set_start_stop():
    """API to call the enable_state service to start and stop the vehicle.

    Returns:
        dict: Execution status if the API call was successful and error
              reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    start_stop = request.json.get("start_stop")
    if start_stop is None:
        return jsonify({"success": False, "reason": "start_stop must be set."})

    webserver_node.get_logger().info(f"Changed the enable state to {start_stop}")
    start_stop_state = False if start_stop == "stop" else True
    try:
        enable_state_req = EnableStateSrv.Request()
        enable_state_req.is_active = start_stop_state
        enable_state_res = call_service_sync(webserver_node.enable_state_cli, enable_state_req)
        if enable_state_res and (enable_state_res.error == 0):
            return jsonify({"success": True})
        else:
            return jsonify({"success": False, "reason": "Error"})

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach enable state server: {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to reach enable state server."})


@VEHICLE_CONTROL_BLUEPRINT.route("/api/max_nav_throttle", methods=["PUT", "POST"])
def max_nav_throttle():
    """API to call the navigation_throttle service to set the throttle scale in the
       autonomous mode.

    Returns:
        dict: Execution status if the API call was successful and error
              reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    nav_throttle = request.json.get("throttle")
    if nav_throttle is None:
        return jsonify({"success": False, "reason": "value must be set."})
    webserver_node.get_logger().info(f"Setting max navigation throttle to {nav_throttle}")
    try:
        set_throttle_req = NavThrottleSrv.Request()
        set_throttle_req.throttle = nav_throttle / constants.MAX_AUTO_THROTTLE_VAL
        set_throttle_res = call_service_sync(webserver_node.set_throttle_cli, set_throttle_req)
        if set_throttle_res and (set_throttle_res.error == 0):
            return jsonify({"success": True})
        else:
            return jsonify(success=False, reason="Failed to call the navigation throttle service")
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach navigation throttle server: {ex}")
        return jsonify(success=False, reason="Unable to reach navigation throttle server")


@VEHICLE_CONTROL_BLUEPRINT.route("/api/ftl_max_speed", methods=["PUT", "POST"])
def max_ftl_speed():
    """API to call the SetMaxSpeedSrv service to set the throttle scale in the
       followtheleader mode.

    Returns:
        dict: Execution status if the API call was successful and error
              reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    ftl_speed = request.json.get("throttle")
    if ftl_speed is None:
        return jsonify({"success": False, "reason": "value must be set."})
    webserver_node.get_logger().info(f"Setting max ftl speed to {ftl_speed}%")
    try:
        set_speed_req = SetMaxSpeedSrv.Request()
        set_speed_req.max_speed_pct = float(ftl_speed / 100)
        set_speed_res = call_service_sync(webserver_node.set_ftl_max_speed_cli, set_speed_req)
        if set_speed_res and (set_speed_res.error == 0):
            return jsonify({"success": True})
        else:
            return jsonify(success=False, reason="Failed to call the SetMaxSpeed service")
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach SetMaxSpeed server: {ex}")
        return jsonify(success=False, reason="Unable to reach SetMaxSpeed server")


@VEHICLE_CONTROL_BLUEPRINT.route("/api/control_modes_available", methods=["GET"])
def control_modes_available():
    """API to call the GetCtrlModesCountSrv service to get the list of available modes
       in ctrl_pkg (autonomous/manual/calibration/followtheleader).

    Returns:
        dict: Execution status if the API call was successful, list of available modes
              and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Providing the number of available modes")
    try:
        get_ctrl_modes_req = GetCtrlModesSrv.Request()
        get_ctrl_modes_res = call_service_sync(webserver_node.get_ctrl_modes_cli,
                                               get_ctrl_modes_req)

        control_modes_available = list()
        for mode in get_ctrl_modes_res.modes:
            control_modes_available.append(constants.MODE_DICT[mode])

        data = {
            "control_modes_available": control_modes_available,
            "success": True
        }
        return jsonify(data)

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach get ctrl modes service: {ex}")
        return jsonify(success=False, reason="Error")
