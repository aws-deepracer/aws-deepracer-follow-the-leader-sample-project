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
utility.py

This module holds the utility functions and classes used across the package.
"""

from subprocess import Popen, PIPE, STDOUT
import shlex
import threading
import time
from flask import jsonify

from webserver_pkg import webserver_publisher_node


def call_service_sync(cli, req, timeout=10, sleep_time=0.01):
    """A wrapper function to call the services and wait for the results until timeout.

    Args:
        cli (rclpy.client.Client): Client object using which we call the service.
        req (Request): Service request object.
        timeout (int, optional): Time in seconds to keep checking for service call to
                                 return result before removing the request. Defaults to 60.
        sleep_time (float, optional): Time in seconds to sleep before each check for
                                      returned result from service call. Defaults to 0.01.

    Returns:
        Response: The service response.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    if cli.service_is_ready():
        webserver_node.get_logger().info(f"Service call initiated: {cli.srv_name}")
        future = cli.call_async(req)
        sequence = -1
        for seq, req_future in cli._pending_requests.items():
            if req_future == future:
                sequence = seq
                break
        webserver_node.get_logger().info(f"New request: {sequence} {cli.srv_name}")
        elapsed_time = 0
        while not future.done():
            if elapsed_time == int(elapsed_time):
                webserver_node.get_logger().info(f"Service call not finished: {sequence} {cli.srv_name}")
            time.sleep(sleep_time)
            elapsed_time += sleep_time
            if elapsed_time >= timeout:
                webserver_node.get_logger().info("Service call was not completed before timeout: "
                                                 f"{sequence} {cli.srv_name} {timeout}")
                future.cancel()
                if future.cancelled():
                    webserver_node.get_logger().error(f"Service was cancelled: {sequence} {cli.srv_name}")
                return None
        webserver_node.get_logger().info(f"Service call finished: {sequence} {cli.srv_name}")
        if future.exception() is not None:
            webserver_node.get_logger().error(f"Error while calling service: {sequence} - "
                                              f"{cli.srv_name} - {future.exception()}")
        return future.result()
    else:
        webserver_node.get_logger().info(f"Service is not ready: {cli.srv_name}")
        return None


def execute(cmd, input_str=None, shell=False, shlex_split=False):
    """Execute the commands on shell terminal.

    Args:
        cmd (str): Command to be executed.
        input_str (str, optional): Input string that has to be passed
                                   to the shell input. Defaults to None.
        shell (bool, optional): Specifies whether to use the shell as the
                                program to execute. Defaults to False.
        shlex_split(bool, optional): Specifies whether to split the command.
                                     Defaults to False.
    Returns:
        tuple: A tuple of return code and the output of the command.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info(f"Command executing: {cmd}")
    if shlex_split:
        cmd = shlex.split(cmd)
    proc = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=STDOUT,
                 universal_newlines=True, shell=shell)
    stdout = proc.communicate(input=input_str)[0]

    webserver_node.get_logger().info(f"{cmd} : execute output: {stdout}")

    return proc.returncode, stdout


def is_network_connected(ssid):
    """Check if the vehicle is connected to the supplied SSID.

    Args:
        ssid (str): SSID value.

    Returns:
        bool: True if the network is connected to the SSID else False.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info(f"Checking if ssid: {ssid} is connected")
    cur_ssid = execute(r'/usr/bin/nmcli -t -f active,ssid -e no dev wifi | egrep "^yes" | cut -d\: -f2',
                       shell=True)[1].split('\n')

    if cur_ssid[0] == ssid:
        return True

    return False


def is_network_inactive(ssid):
    """Check if the supplied SSID is currently not connected, but previously connected.

    Args:
        ssid (str): SSID value.

    Returns:
        bool: True if the SSID is inactive else False.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info(f"Checking if ssid: {ssid} is inactive")

    is_present = False

    stdout = execute("/usr/bin/nmcli -t -f name -e no con show",
                     shlex_split=True)[1]
    for line in stdout.splitlines():
        if line == ssid:
            is_present = True
            break

    if is_present and not is_network_connected(ssid):
        return True

    return False


def api_fail(reason):
    """Helper method to create failed json return with reason.

    Args:
        reason (str): Reason message.

    Returns:
        dict: JSON object with success and reason keys.
    """
    return jsonify({"success": False, "reason": reason})


#########################################################################################
# Double Buffer.


class DoubleBuffer():
    """Object type which helps to thread-safely read and write from the buffer.
    """
    def __init__(self, clear_data_on_get=True):
        """Create a DoubleBuffer object.

        Args:
            clear_data_on_get (bool, optional): Flag to clear data from the queue after its read.
                                                Defaults to True.
        """
        self.read_buffer = None
        self.write_buffer = None
        self.clear_data_on_get = clear_data_on_get
        self.cv = threading.Condition()

    def clear(self):
        """Helper method to clear the buffer.
        """
        with self.cv:
            self.read_buffer = None
            self.write_buffer = None

    def put(self, data):
        """Helper method to safely store the data in the buffer.

        Args:
            data (Any): The object that is to be stored.
        """
        with self.cv:
            self.write_buffer = data
            self.write_buffer, self.read_buffer = self.read_buffer, self.write_buffer
            self.cv.notify()

    def get(self, block=True):
        """Helper method to safely read the data from the buffer.

        Args:
            block (bool, optional): Flag set to wait for the new data if read_buffer is empty.
                                    Defaults to True.

        Raises:
            DoubleBuffer.Empty: Exception if returning empty read buffer without waiting for
                                new data.

        Returns:
            Any: Data stored in the buffer.
        """
        with self.cv:
            if not block:
                if self.read_buffer is None:
                    raise DoubleBuffer.Empty
            else:
                while self.read_buffer is None:
                    self.cv.wait()
            data = self.read_buffer
            if self.clear_data_on_get:
                self.read_buffer = None
            return data

    def get_nowait(self):
        """Wrapper method to return the data stored in buffer without waiting for new data.

        Returns:
            Any: Data stored in the buffer.
        """
        return self.get(block=False)

    class Empty(Exception):
        """Custom Exception to identify if an attempt was made to read from an empty buffer.

        Args:
            Exception (Exception): Inherits from Exception class.
        """
        pass
