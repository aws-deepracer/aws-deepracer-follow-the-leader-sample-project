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
led_api.py

This is the module that holds the APIs required to get/set the tail light LED.
"""

from flask import (Blueprint,
                   jsonify,
                   request)

from deepracer_interfaces_pkg.srv import (SetLedCtrlSrv,
                                          GetLedCtrlSrv)
from webserver_pkg.constants import LED_SCALING_FACTOR
from webserver_pkg import webserver_publisher_node
from webserver_pkg.utility import call_service_sync

LED_API_BLUEPRINT = Blueprint("led_api", __name__)


@LED_API_BLUEPRINT.route("/api/set_led_color", methods=["POST"])
def set_led_color():
    """API to call the service to set the tail light LED color
       from the red, green, blue values from front end.

    Returns:
        dict: Execution status if the API call was successful and
              error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        data = request.json
        if int(data["red"]) < 0 or int(data["red"]) > 255 \
           or int(data["green"]) < 0 or int(data["green"]) > 255 \
           or int(data["blue"]) < 0 or int(data["blue"]) > 255:
            return jsonify(success=False, reason="Input is not valid")
        # Convert to PWM
        red = int(data["red"]) * LED_SCALING_FACTOR
        green = int(data["green"]) * LED_SCALING_FACTOR
        blue = int(data["blue"]) * LED_SCALING_FACTOR
        webserver_node.get_logger().info("Set LED Color: "
                                         f"Red: {red} "
                                         f"Green: {green} "
                                         f"Blue: {blue}")

        set_led_color_req = SetLedCtrlSrv.Request()
        set_led_color_req.red = red
        set_led_color_req.green = green
        set_led_color_req.blue = blue
        set_led_color_res = call_service_sync(webserver_node.set_led_color_cli,
                                              set_led_color_req)
        if set_led_color_res and set_led_color_res.error == 0:
            data = {"success": True}
        else:
            webserver_node.get_logger().error("Set led color service call failed")
            data = {
                "reason": "Error",
                "success": False
            }
        return jsonify(data)

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach set led color server: {ex}")
        return jsonify(success=False, reason="Error")


@LED_API_BLUEPRINT.route("/api/get_led_color", methods=["GET"])
def get_led_color():
    """API to call the service to get red, green, blue values for the tail
       light LED color that is currently active.

    Returns:
        dict: Execution status if the API call was successful, channel values
              for the tail light LED and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()

    webserver_node.get_logger().info("Providing r, b, g values as response")
    try:
        get_led_color_req = GetLedCtrlSrv.Request()
        get_led_color_res = call_service_sync(webserver_node.get_led_color_cli,
                                              get_led_color_req)
        if get_led_color_res:
            converted_red = get_led_color_res.red // LED_SCALING_FACTOR
            converted_green = get_led_color_res.green // LED_SCALING_FACTOR
            converted_blue = get_led_color_res.blue // LED_SCALING_FACTOR

            data = {
                "red": converted_red,
                "green": converted_green,
                "blue": converted_blue,
                "success": True
            }
            webserver_node.get_logger().info("Current LED RGB values: "
                                             f"Red: {data['red']} "
                                             f"Green: {data['green']} "
                                             f"Blue: {data['blue']}")
            return jsonify(data)
        else:
            return jsonify(success=False, reason="Error")
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach get vehicle led server: {ex}")
        return jsonify(success=False, reason="Error")
