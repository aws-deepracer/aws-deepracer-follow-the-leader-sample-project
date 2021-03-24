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
software_update.py

This is the module that holds the APIs and utility functions required to manage
the software udpates system. It contains APIs to fetch the current software
update status, execute software update, check if the device has rebooted and webserver
is back up online.
"""

import itertools
import time
import os
import json
from flask import (Blueprint,
                   jsonify,
                   request,
                   Response)

from deepracer_interfaces_pkg.srv import (SoftwareUpdateCheckSrv,
                                          BeginSoftwareUpdateSrv,
                                          SoftwareUpdateStateSrv)
from webserver_pkg.constants import (SOFTWARE_UPDATE_STATUS_PATH,
                                     SOFTWARE_UPDATE_FETCH_FREQUENCY,
                                     SLEEP_TIME_BEFORE_REBOOT)
from webserver_pkg.utility import (execute,
                                   call_service_sync)
from webserver_pkg import webserver_publisher_node


SOFTWARE_UPDATE_BLUEPRINT = Blueprint("software_update", __name__)


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/get_mandatory_update_status", methods=["GET"])
def get_mandatory_update_status():
    """API to read the saved mandatory update status from the software update status file
       and return the value to the client.

    Returns:
        dict: Execution status if the API call was successful with mandatory update status
              and error reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Getting mandatory software update status.")
    software_update_status = {"update_completed": False}
    try:
        # if software_update_status file not exists, create one.
        if not os.path.isfile(SOFTWARE_UPDATE_STATUS_PATH):
            with open(SOFTWARE_UPDATE_STATUS_PATH, "w") as software_update_status_file:
                json.dump(software_update_status, software_update_status_file)
        else:
            with open(SOFTWARE_UPDATE_STATUS_PATH, "r") as software_update_status_file:
                software_update_status = json.load(software_update_status_file)
            if "update_completed" not in software_update_status:
                return jsonify({"success": False,
                                "reason": "Incorrect software update status object"
                                })
        return jsonify({"success": True,
                        "status": software_update_status["update_completed"]
                        })
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to read from software update status file {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to read from software update status file"})


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/set_mandatory_update_status", methods=["PUT", "POST"])
def set_mandatory_update_status():
    """API to save the mandatory update status to the software update status file.

    Returns:
        dict: Execution status if the API call was successful.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    update_completed_value = request.json.get("update_completed")
    if update_completed_value is None:
        return jsonify({"success": False, "reason": "Status must be a boolean"})
    webserver_node.get_logger().info("Setting mandatory software update status to "
                                     f"{update_completed_value}")

    software_update_status = {"update_completed": update_completed_value}
    try:
        with open(SOFTWARE_UPDATE_STATUS_PATH, "w") as software_update_status_file:
            json.dump(software_update_status, software_update_status_file)
        return jsonify({"success": True})
    except IOError:
        webserver_node.get_logger().error("Unable to set software update status: "
                                          f"{software_update_status}")
        return jsonify({"success": False})


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/is_software_update_available", methods=["GET"])
def is_software_update_available():
    """API to call the service to check if the software update is available.

    Returns:
        dict: Execution status if the API call was successful with the software update
              status and error reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        sw_update_state_req = SoftwareUpdateCheckSrv.Request()
        sw_update_state_req.force_update_check = False
        sw_update_state_res = call_service_sync(webserver_node.sw_update_state_cli,
                                                sw_update_state_req,
                                                timeout=180)
        if sw_update_state_res:
            webserver_node.get_logger().info("Status returned from software_update_get_state: "
                                             f"{sw_update_state_res.software_update_state}")
            # Software update status == 0 -> up to date,
            # 1 -> update available,
            # others -> error/pending/progress state
            return jsonify({"success": True,
                            "status": sw_update_state_res.software_update_state == 1
                            })
        else:
            return jsonify({"success": False,
                            "reason": "Unable to reach software update state server"
                            })
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach software update state server: {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to reach software update state server"})


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/begin_software_update", methods=["GET", "POST"])
def begin_software_update():
    """API to call the service to begin the software update process.

    Returns:
        dict: Execution status if the API call was successful and error reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        webserver_node.get_logger().info("Started software update.")
        begin_sw_update_req = BeginSoftwareUpdateSrv.Request()
        begin_sw_update_req.sleep_time_before_reboot = SLEEP_TIME_BEFORE_REBOOT
        begin_sw_update_res = call_service_sync(webserver_node.begin_sw_update_cli,
                                                begin_sw_update_req)
        if begin_sw_update_res and begin_sw_update_res.response_status:
            return jsonify({"success": True})
        else:
            webserver_node.get_logger().error("Begin software update service call failed")
            return jsonify({"success": False,
                            "reason": "Update service call failed"
                            })
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach begin update server: {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to reach begin update server"
                        })


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/update_status", methods=["GET"])
def get_software_update_status():
    """API to stream the software update progress percentage and the current state.

    Returns:
        flask.Response: Flask response object with the content_type set to text/event-stream.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Inside the update status function")

    def events():
        webserver_node.get_logger().info("Running software update event source")
        for i, c in enumerate(itertools.cycle("\|/-")):
            try:
                pct_dict = webserver_node.pct_dict_db.get_nowait()
                percentage_completion = pct_dict["update_pct"]
                result = f"status:{pct_dict['status']}|update_pct:{percentage_completion}"
                yield "data: %s %d\n\n" % (result, i)
                if(percentage_completion == 100):
                    webserver_node.get_logger().info("software update event source result: "
                                                     f"{result}")
                    request = True
                    sw_update_status_req = SoftwareUpdateStateSrv.Request()
                    sw_update_status_req.request = request
                    sw_update_status_res = call_service_sync(webserver_node.sw_update_status_cli,
                                                             sw_update_status_req)
                    if sw_update_status_res:
                        # Check if software update state is set.
                        if sw_update_status_res.update_state == 0:
                            # Write to software udpate status as atleast one
                            # update has completed successfully.
                            with open(SOFTWARE_UPDATE_STATUS_PATH, "w") \
                             as software_update_status_file:
                                software_update_status = {"update_completed": True}
                                json.dump(software_update_status, software_update_status_file)

                            break
                    else:
                        webserver_node.get_logger().error("Unable to reach update status service: "
                                                          f"{sw_update_status_res}")
                        result = "status:complete|update_pct:100"
                        yield f"data: {result} {1}\n\n"

                # The sleep is introduced here so as to fetch the next message from
                # the software_update_status service. This is rate at which the UI shows
                # the change in the status querying the service. So this will provide us
                # to control the rate at which we would like to see the software update
                # information on the browser. For now its set to 1 seconds.
                time.sleep(SOFTWARE_UPDATE_FETCH_FREQUENCY)
            except Exception as ex:
                webserver_node.get_logger().error(f"Unable to reach update status service: {ex}")
                result = "status:checking|update_pct:0"
                yield f"data: {result} {1}\n\n"
                break
    return Response(events(), content_type="text/event-stream")


@SOFTWARE_UPDATE_BLUEPRINT.route("/api/server_ready", methods=["GET"])
def isServerReady():
    """API to check if the server is back up after reboot.

    Returns:
        dict: A successful call to this API will return a success status.
    """
    return jsonify({"success": True, "status": True})
