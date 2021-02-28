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
calibration.py

This is the module that holds the APIs and utility functions required to set the
vehicle in calibration mode, capture the action performed by user from the UI in
the calibration mode, and trigger appropriate service calls to move the servo/motor,
get and set the calibration values from the file.
"""

from flask import (Blueprint,
                   jsonify,
                   request)

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import (ActiveStateSrv,
                                          EnableStateSrv,
                                          SetCalibrationSrv,
                                          GetCalibrationSrv)
from webserver_pkg.constants import (PWM_ANGLE_CONVERSION,
                                     PWM_THROTTLE_CONVERSION,
                                     PWM_OFFSET,
                                     CALIBRATION_MODE)
from webserver_pkg.utility import (api_fail,
                                   call_service_sync)
from webserver_pkg import webserver_publisher_node


CALIBRATION_BLUEPRINT = Blueprint("calibration", __name__)


@CALIBRATION_BLUEPRINT.route("/api/set_calibration_mode", methods=["GET"])
def set_calibration_mode():
    """API to call the service to activate calibration mode.

    Returns:
        dict: Execution status if the API call was successful.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        vehicle_state_req = ActiveStateSrv.Request()
        vehicle_state_req.state = CALIBRATION_MODE
        vehicle_state_res = call_service_sync(webserver_node.vehicle_state_cli,
                                              vehicle_state_req)

        enable_state_req = EnableStateSrv.Request()
        enable_state_req.is_active = True
        enable_state_res = call_service_sync(webserver_node.enable_state_cli,
                                             enable_state_req)

        success = (vehicle_state_res and
                   vehicle_state_res.error == 0 and
                   enable_state_res and
                   enable_state_res.error == 0)
        if not success:
            webserver_node.get_logger().error("Vehicle state service call failed")
            return jsonify(success=False, reason="Error")
        return jsonify(success=True)
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach vehicle state server: {ex}")
        return jsonify(success=False, reason="Error")


@CALIBRATION_BLUEPRINT.route("/api/get_calibration/<cali_type>", methods=["GET"])
def api_get_calibration(cali_type):
    """API to call the service to get the calibration from file with the current
       calibration.

    Args:
        cali_type (int): Calibration type identifying steering/throttle calibration.

    Returns:
        dict: Execution status if the API call was successful.
    """
    state = 0 if cali_type == "angle" else 1
    conversion = PWM_ANGLE_CONVERSION if state == 0 else PWM_THROTTLE_CONVERSION
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        get_cal_req = GetCalibrationSrv.Request()
        get_cal_req.cal_type = state
        get_cal_res = call_service_sync(webserver_node.get_cal_cli, get_cal_req)
        if get_cal_res is None or get_cal_res.error != 0:
            return api_fail("Unable to reach get vehicle calibration server")
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach get vehicle calibration server: {ex}")
        return api_fail("Unable to reach get vehicle calibration server")

    converted_min = (get_cal_res.min - PWM_OFFSET) / conversion
    converted_max = (get_cal_res.max - PWM_OFFSET) / conversion
    data = {
        "mid": (get_cal_res.mid - PWM_OFFSET) / conversion,
        "min": (converted_min if (get_cal_res.polarity) else converted_max),
        "max": (converted_max if (get_cal_res.polarity) else converted_min),
        "polarity": get_cal_res.polarity,
        "success": True
    }
    webserver_node.get_logger().info(f"Current calibration state: {state} "
                                     f"MID: {data['mid']} "
                                     f"MIN: {data['min']} "
                                     f"MAX: {data['max']} "
                                     f"Polarity: {data['polarity']}")
    return jsonify(data)


@CALIBRATION_BLUEPRINT.route("/api/set_calibration/<cali_type>", methods=["PUT", "POST"])
def api_set_calibration(cali_type):
    """API to call the service to store the calibration to file and
       set the current calibration.

    Args:
        cali_type (int): Calibration type identifying steering/throttle calibration.

    Returns:
        dict: Execution status if the API call was successful.
    """
    data = request.json
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info(f"set_calibration: {data}")
    state = 0 if cali_type == "angle" else 1
    conversion = PWM_ANGLE_CONVERSION if state == 0 else PWM_THROTTLE_CONVERSION
    set_mid = int(data["mid"]) * conversion + PWM_OFFSET
    set_min = int(data["min"]) * conversion + PWM_OFFSET
    set_max = int(data["max"]) * conversion + PWM_OFFSET
    set_polarity = int(data["polarity"])
    webserver_node.get_logger().info(f"State: {state} "
                                     f"mid: {set_mid} "
                                     f"min: {set_min} "
                                     f"max: {set_max} "
                                     f"polarity: {set_polarity}")
    try:
        set_cal_req = SetCalibrationSrv.Request()
        set_cal_req.cal_type = state
        set_cal_req.min = set_min
        set_cal_req.max = set_max
        set_cal_req.mid = set_mid
        set_cal_req.polarity = set_polarity
        set_cal_res = call_service_sync(webserver_node.set_cal_cli, set_cal_req)
        if set_cal_res and set_cal_res.error == 0:
            return jsonify({"success": True})
        else:
            return jsonify({"success": False})
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach set vehicle calibration server: {ex}")
        return api_fail("Unable to reach set vehicle calibration")


@CALIBRATION_BLUEPRINT.route("/api/adjust_calibrating_wheels/<cali_type>",
                             methods=["PUT", "POST"])
def api_adjust_calibrating_wheels(cali_type):
    """API to call the service to publishes a message withPWM measurement to adjust
       wheels while calibrating.

    Args:
        cali_type (int): Calibration type identifying steering/throttle calibration.

    Returns:
        dict: Execution status if the API call was successful.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    form_data = request.json
    if cali_type == "angle":
        angle = float(int(form_data["pwm"]) * PWM_ANGLE_CONVERSION + PWM_OFFSET)
        throttle = -1.0
    else:
        angle = -1.0
        throttle = float(int(form_data["pwm"]) * PWM_THROTTLE_CONVERSION + PWM_OFFSET)
    webserver_node.get_logger().info(f"Angle: {angle}  Throttle: {throttle}")
    msg = ServoCtrlMsg()
    msg.angle = angle
    msg.throttle = throttle
    webserver_node.pub_calibration_drive.publish(msg)
    return jsonify({"success": True})
