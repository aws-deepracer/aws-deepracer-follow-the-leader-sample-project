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
device_info.py

This is the module that holds the APIs required to get the hardware/software version,
sensor status and battery level information.
"""

from flask import Blueprint
from flask import jsonify

from deepracer_interfaces_pkg.srv import (GetDeviceInfoSrv,
                                          BatteryLevelSrv,
                                          SensorStatusCheckSrv)
from webserver_pkg import webserver_publisher_node
from webserver_pkg.utility import call_service_sync

DEVICE_INFO_API_BLUEPRINT = Blueprint("device_info_api", __name__)


@DEVICE_INFO_API_BLUEPRINT.route("/api/get_device_info", methods=["GET"])
def get_device_info():
    """API to call the service to get the current hardware version of the
       DeepRacer vehicle and the software version of aws-deepracer-core package.

    Returns:
        dict: Execution status if the API call was successful, hardware and
              software version details and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Providing hardware and software revision "
                                     "details as response")
    try:
        get_revision_info_req = GetDeviceInfoSrv.Request()
        get_revision_info_res = call_service_sync(webserver_node.get_revision_info_cli,
                                                  get_revision_info_req)
        if get_revision_info_res and get_revision_info_res.error == 0:
            data = {
                "hardware_version": get_revision_info_res.hardware_version,
                "software_version": get_revision_info_res.software_version,
                "success": True
            }
            webserver_node.get_logger().info(f"Hardware version: {data['hardware_version']}, "
                                             f"Software version: {data['software_version']}")
        else:
            webserver_node.get_logger().error("Get device info service call failed")
            data = {
                "reason": "Error",
                "success": False
            }
        return jsonify(data)

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach revision info server: {ex}")
        return jsonify(success=False, reason="Error")


@DEVICE_INFO_API_BLUEPRINT.route("/api/get_battery_level", methods=["GET"])
def get_battery_level():
    """API to call the service to get the current vehicle battery level
       information.

    Returns:
        dict: Execution status if the API call was successful, vehicle
              battery level details and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()

    try:
        battery_level_req = BatteryLevelSrv.Request()
        battery_level_res = call_service_sync(webserver_node.battery_level_cli,
                                              battery_level_req)
        if battery_level_res:
            data = {
                "battery_level": battery_level_res.level,
                "success": True
            }
            webserver_node.get_logger().info(f"Battery Level: {data['battery_level']}")
            return jsonify(data)
        else:
            return jsonify(success=False, reason="Error")
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach battery level server: {ex}")
        return jsonify(success=False, reason="Error")


@DEVICE_INFO_API_BLUEPRINT.route("/api/get_sensor_status", methods=["GET"])
def get_sensor_status():
    """API to call the service to get the sensor data status for camera and
       LiDAR sensors connected to the vehicle.

    Returns:
        dict: Execution status if the API call was successful, sensor status
              information and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    data = {
        "camera_status": "checking",
        "stereo_status": "checking",
        "lidar_status": "checking",
        "success": True
    }
    try:
        sensor_status_req = SensorStatusCheckSrv.Request()
        sensor_status_res = call_service_sync(webserver_node.sensor_status_cli, sensor_status_req)
        if sensor_status_res and sensor_status_res.error == 0:
            data["camera_status"] = \
                "not_connected" if sensor_status_res.single_camera_status == 1 else "connected"
            data["stereo_status"] = \
                "not_connected" if sensor_status_res.stereo_camera_status == 1 else "connected"
            data["lidar_status"] = \
                "not_connected" if sensor_status_res.lidar_status == 1 else "connected"
        else:
            webserver_node.get_logger().error("Get sensor status service call failed")
            data["camera_status"] = "error"
            data["stereo_status"] = "error"
            data["lidar_status"] = "error"
    except Exception as ex:
        webserver_node.get_logger().error("Unable to reach sensor status server: {ex}")
        data["camera_status"] = "error"
        data["stereo_status"] = "error"
        data["lidar_status"] = "error"
    finally:
        webserver_node.get_logger().info(f"Camera status: {data['camera_status']}, "
                                         f"Stereo status: {data['stereo_status']}, "
                                         f"Lidar status: {data['lidar_status']}")
        return jsonify(data)
