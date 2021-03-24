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
ssh_api.py

This is the module that holds the APIs and utility functions required to manage the
SSH connection and get current ssh connection status.
"""

import re
import pam
from flask import (Blueprint,
                   jsonify,
                   request)

from webserver_pkg.constants import (DEFAULT_USER,
                                     DEFAULT_SSH_PASSWORD)
from webserver_pkg import utility
from webserver_pkg import webserver_publisher_node

SSH_API_BLUEPRINT = Blueprint("ssh_api", __name__)


@SSH_API_BLUEPRINT.route("/api/isSshEnabled", methods=["GET"])
def is_ssh_enabled():
    """API called to execute commands to check if SSH is enabled.
       This will populate the ssh enabled radio button in the UI.

    Returns:
        dict: Execution status if the API call was successful, flag to indicate
              if ssh is enabled and with error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Providing ssh enabled as response")
    try:
        # Check SSH status
        if utility.execute("/bin/systemctl --no-pager status ssh", shlex_split=True)[1].find("active (running)") > -1:
            # Check UFW status
            stdout = utility.execute("/usr/sbin/ufw status", shlex_split=True)[1]
            if re.search("22.*ALLOW", stdout):
                return jsonify(success=True,
                               isSshEnabled=True,
                               reason="Ssh is enabled.")
            else:
                return jsonify(success=True,
                               isSshEnabled=False,
                               reason="Ssh not enabled.")
        else:
            return jsonify(success=True,
                           isSshEnabled=False,
                           reason="Ssh not enabled.")
    except Exception as ex:
        webserver_node.get_logger().error(f"Failed check if ssh is enabled: {ex}")
        return jsonify(success=False, isSshEnabled=False, reason="Error")


@SSH_API_BLUEPRINT.route("/api/enableSsh", methods=["GET"])
def enable_ssh():
    """API called to execute commands to enable the ssh.

    Returns:
        dict: Execution status if the API call was successful, flag to indicate
              if ssh is enabled and with error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Enabling SSH")
    try:
        # Start SSH
        utility.execute("/usr/sbin/service ssh start", shlex_split=True)
        # Allow SSH
        utility.execute("/usr/sbin/ufw allow ssh", shlex_split=True)
        return jsonify(success=True, isSshEnabled=True, reason="Ssh enabled.")
    except Exception as ex:
        webserver_node.get_logger().error(f"Failed to enable ssh: {ex}")
        return jsonify(success=False, isSshEnabled=False, reason="Error")


@SSH_API_BLUEPRINT.route("/api/disableSsh", methods=["GET"])
def disable_ssh():
    """API called to execute commands to disable the ssh.

    Returns:
        dict: Execution status if the API call was successful, flag to indicate
              if ssh is still enabled and with error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Disabling SSH")
    try:
        # Stop SSH
        utility.execute("/usr/sbin/service ssh stop", shlex_split=True)
        return jsonify(success=True, isSshEnabled=False, reason="Ssh disabled.")
    except Exception as ex:
        webserver_node.get_logger().error(f"Failed to disable ssh: {ex}")
        return jsonify(success=False, isSshEnabled=True, reason="Error")


@SSH_API_BLUEPRINT.route("/api/isSshDefaultPasswordChanged", methods=["GET"])
def is_ssh_default_password_changed():
    """API called to check if the default password for SSH is changed.
       This will populate the ssh enabled radio button in the UI.

    Returns:
        dict: Execution status if the API call was successful, flag to indicate
              if default ssh password is changed and with error reason if
              call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Providing ssh enabeld as response")
    try:
        # Authenticate against default password
        if pam.pam().authenticate(DEFAULT_USER, DEFAULT_SSH_PASSWORD):
            return jsonify(success=True,
                           isDefaultSshPasswordChanged=False,
                           reason="Default password not changed")
        else:
            return jsonify(success=True,
                           isDefaultSshPasswordChanged=True,
                           reason="Default password changed")
    except Exception as ex:
        webserver_node.get_logger().error(f"Failed check if the ssh password is default: {ex}")
        return jsonify(success=False,
                       isDefaultSshPasswordChanged=False,
                       reason="Error")


@SSH_API_BLUEPRINT.route("/api/resetSshPassword", methods=["POST", "PUT"])
def ssh_reset():
    """API called to execute commands to reset the ssh password with the new one.

    Returns:
        dict: Execution status if the API call was successful and with error
              reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("User requesting for resetting the ssh credentials")
    try:
        old_ssh_password = request.json["oldPassword"]
        new_ssh_password = request.json["newPassword"]

        # Check if old password is correct
        if not pam.pam().authenticate(DEFAULT_USER, old_ssh_password):
            return jsonify(success=False,
                           reason="The password is incorrect. "
                                  "Provide your current password.")

        # Check if the default password is updated
        passwd_status_cmd = f"/usr/bin/passwd --status {DEFAULT_USER}"
        passwd_status_output = utility.execute(passwd_status_cmd, shlex_split=True)[1]

        # Execute passwd command based on the status of the password obtained above
        if passwd_status_output.split()[1] == "P":
            webserver_node.get_logger().info("Usable Password present")
            if (utility.execute(f"/usr/bin/sudo -u {DEFAULT_USER} /usr/bin/passwd",
                                input_str=(f"{old_ssh_password}"
                                           f"\n{new_ssh_password}"
                                           f"\n{new_ssh_password}"),
                                shlex_split=True)[1]
                    .find("successfully")) == -1:
                return jsonify(success=False, reason="The password update was unsuccessful.")
            else:
                return jsonify(success=True)
        elif passwd_status_output.split()[1] == "NP":
            webserver_node.get_logger().info("No Password present")
            if (utility.execute(f"/usr/bin/passwd {DEFAULT_USER}",
                                input_str=(f"{new_ssh_password}\n"
                                           f"{new_ssh_password}"),
                                shlex_split=True)[1]
                    .find("successfully")) > -1:
                return jsonify(success=True)
        else:
            return jsonify(success=False, reason="Unable to change the SSH password")
    except Exception:
        return jsonify(success=False, reason="Error")
