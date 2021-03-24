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
vehicle_logs.py

This is the module that holds the API required to fetch the logs and pass it over
to the UI.
"""

from flask import (Blueprint,
                   jsonify)

from webserver_pkg import constants
from webserver_pkg import webserver_publisher_node

VEHICLE_LOGS_BLUEPRINT = Blueprint("vehicle_logs", __name__)


@VEHICLE_LOGS_BLUEPRINT.route("/api/logs/<log_type>/<int:num_lines>", methods=["GET"])
def api_get_logs(log_type, num_lines):
    """API to return the logs of appropriate type and logging window based on
       user selection.

    Args:
        log_type (str): Log type attribute name pointing to the log file location.
                        Defined in the constants file.
        num_lines (int): Number of latest log lines to return.

    Returns:
        dict: Execution status if the API call was successful and the
              log file read response.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    num_lines = 1000 if num_lines > 1000 else num_lines
    log_content = ""
    try:
        with VEHICLE_LOGS_BLUEPRINT.open_resource(getattr(constants, log_type)) as logfile:
            lines = logfile.readlines()[-num_lines:]
            for i in range(0, len(lines)):
                log_content = log_content + lines[i].decode("utf-8")
    except AttributeError:
        webserver_node.get_logger().error("Type of log does not exist!")
        log_content = "Type of log does not exist!"
    except IOError:
        webserver_node.get_logger().error("File Not Found!")
        log_content = "File Not Found!"
    return jsonify({"success": True, "data": log_content})
