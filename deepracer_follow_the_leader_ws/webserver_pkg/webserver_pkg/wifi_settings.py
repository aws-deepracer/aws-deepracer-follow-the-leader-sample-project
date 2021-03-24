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
wifi_settings.py

This is the module that holds the APIs and utility functions required to manage the
WiFi connection, get network details, check for ethernet-over-usb connection.
"""

import math
from flask import (Blueprint,
                   jsonify,
                   request)

from deepracer_interfaces_pkg.srv import OTGLinkStateSrv
from webserver_pkg import utility
from webserver_pkg.constants import REMOVE_IPS
from webserver_pkg import webserver_publisher_node


WIFI_SETTINGS_BLUEPRINT = Blueprint("wifi_settings", __name__)


def check_usb_connection():
    """Call the OTGLinkState service to find out if OTG is connected.

    Returns:
        dict: Execution status if the service call was successful and the response.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    is_usb_connected = False
    try:
        otg_link_state_req = OTGLinkStateSrv.Request()
        otg_link_state_res = utility.call_service_sync(webserver_node.otg_link_state_cli, otg_link_state_req)
        is_usb_connected = True if otg_link_state_res and otg_link_state_res.link_state else False
    except Exception as arg:
        webserver_node.get_logger().info(f"USB connection check exception: {arg}")

    webserver_node.get_logger().info(f"Check OTG Link State: {is_usb_connected}")
    return {"success": True, "is_usb_connected": is_usb_connected}


def get_static_ip_address():
    """Returns the static ip address of the vehicle.

    Returns:
        str: Comma seperated ip address values.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        _, ips = utility.execute("hostname -I", shell=True)
        req_ips = list()
        for ip_address in ips.split():
            ip_norm = ip_address.strip()
            if ip_norm in REMOVE_IPS or not ip_norm:
                continue
            req_ips.append(ip_norm)
        return ", ".join(req_ips)
    except Exception as ex:
        webserver_node.get_logger().info(f"Failed to fetch the static IP address: {ex}")
        return ""


def get_connected_ssid():
    """Get the current SSID if the vehicle is connected to the suppied SSID.

    Returns:
        str: SSID details.
    """
    cur_ssid = utility.execute(r'/usr/bin/nmcli -t -f active,ssid -e no dev wifi | egrep "^yes" | cut -d\: -f2',
                               shell=True)[1]
    return cur_ssid.strip() if cur_ssid.strip() and cur_ssid else ""


@WIFI_SETTINGS_BLUEPRINT.route("/api/is_usb_connected", methods=["GET"])
def is_usb_connected():
    """API to return information if the DeepRacer vehicle is connected the micro-USB cable to
       laptop/computer.

    Returns:
        [type]: True if the vehicle is connected to USB
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    is_usb_connected = False
    conn_res = check_usb_connection()
    usb_connected = True if conn_res["success"] and conn_res["is_usb_connected"] else False
    if request.headers["Referer"].find("deepracer.aws") != -1 and usb_connected:
        is_usb_connected = True

    webserver_node.get_logger().info(f"host: {request.headers['Referer']} "
                                     f"otg_connected: {'connected' if usb_connected else 'not connected'} "
                                     f"is_usb_connected: {'connected' if is_usb_connected else 'not connected'}")

    return jsonify({
        "success": True,
        "is_usb_connected": is_usb_connected,
    })


@WIFI_SETTINGS_BLUEPRINT.route("/api/get_network_details", methods=["GET"])
def get_network_details():
    """API to get connected network details such as SSID, IP address, USB connection
       status.

    Returns:
        dict: JSON object with teh network details and execution status.
    """
    conn_res = check_usb_connection()
    usb_connected = True if conn_res["success"] and conn_res["is_usb_connected"] else False
    return jsonify({
        "success": True,
        "is_usb_connected": usb_connected,
        "SSID": get_connected_ssid(),
        "ip_address": get_static_ip_address()
    })


@WIFI_SETTINGS_BLUEPRINT.route("/api/available_wifi_info", methods=["GET"])
def api_wifi_information():
    """API to get the list of available WiFi networks. This will populate the
       wifi drop down in the UI.

    Returns:
        list: List of dict objects with WiFi network details.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Providing wifi information as response")
    ssid_set = set()
    cmd = "/usr/bin/nmcli -t -f ssid,signal,security -e no device wifi list"
    wifi_list = list()
    wifi_lines = utility.execute(cmd, shlex_split=True)[1].split("\n")
    for wifi in wifi_lines:
        words = wifi.split(":")
        if words[0] in ssid_set or words[0] == "--" or len(words[0]) == 0:
            continue
        ssid_set.add(words[0])

        wifi_data = {
            "ssid": words[0],
            "strength": math.floor(1.0 + float(words[1])/30.0),
            "security_info": words[2]}
        wifi_list.append(wifi_data)
    return jsonify(wifi_list)


@WIFI_SETTINGS_BLUEPRINT.route("/api/wifi_reset", methods=["POST"])
def api_wifi_reset():
    """API used to reset the wifi ssid and password. This will send success or failure message.
       Connects the vehicle to the provide Wi-Fi credentials.

    Returns:
        dict: Execution status if the reset was successful and the static ip address.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("User requesting for resetting the wifi credentials")
    form_data = request.json
    wifi_name = form_data["wifi_name"]
    wifi_password = form_data["wifi_password"]

    if utility.is_network_connected(wifi_name):
        ip_address = get_static_ip_address()
        return jsonify({"success": True, "ip_address": ip_address})

    if utility.is_network_inactive(wifi_name):
        utility.execute(['sudo', '/usr/bin/nmcli', 'con', 'del', wifi_name])

    utility.execute(['sudo', '/usr/bin/nmcli', 'device', 'wifi', 'con', wifi_name,
                     'password', wifi_password, 'ifname', 'mlan0'])

    if not utility.is_network_connected(wifi_name):
        webserver_node.get_logger().info("Wifi not changed successfully, clean up.")
        utility.execute(['sudo', '/usr/bin/nmcli', 'con', 'del', wifi_name])
        return jsonify({"success": False,
                        "reason": "Could not connect to the Wi-Fi network.\
                         Check your network ID and password and try again."})
    ip_address = get_static_ip_address()
    return jsonify({"success": True, "ip_address": ip_address})
