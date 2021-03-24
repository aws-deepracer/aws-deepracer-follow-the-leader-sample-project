#!/usr/bin/env python

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
webserver_publisher_node.py

This module creates the webserver_publisher_node which launches a Flask application
as a background thread and creates service clients and subscribers for all the services
and topics that are required by the APIs called from the DeepRacer vehicle console.
"""

import threading
from flask import g
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import (ReentrantCallbackGroup,
                                   MutuallyExclusiveCallbackGroup)
from rclpy.qos import (QoSReliabilityPolicy,
                       QoSProfile,
                       QoSHistoryPolicy)

# TODO: Figure out a way to avoid global variable for webserver node shared across Flask threads
webserver_node = None

from deepracer_interfaces_pkg.srv import (ActiveStateSrv,
                                          EnableStateSrv,
                                          GetCalibrationSrv,
                                          SetCalibrationSrv,
                                          GetDeviceInfoSrv,
                                          BatteryLevelSrv,
                                          SensorStatusCheckSrv,
                                          SetLedCtrlSrv,
                                          GetLedCtrlSrv,
                                          GetModelLoadingStatusSrv,
                                          VerifyModelReadySrv,
                                          LidarConfigSrv,
                                          ModelStateSrv,
                                          ConsoleModelActionSrv,
                                          SoftwareUpdateCheckSrv,
                                          BeginSoftwareUpdateSrv,
                                          SoftwareUpdateStateSrv,
                                          NavThrottleSrv,
                                          SetMaxSpeedSrv,
                                          GetCtrlModesSrv,
                                          OTGLinkStateSrv)
from deepracer_interfaces_pkg.msg import (ServoCtrlMsg,
                                          SoftwareUpdatePctMsg)
from webserver_pkg.webserver import app
from webserver_pkg.utility import DoubleBuffer
from webserver_pkg.constants import (VEHICLE_STATE_SERVICE,
                                     ENABLE_STATE_SERVICE,
                                     GET_CAR_CAL_SERVICE,
                                     SET_CAR_CAL_SERVICE,
                                     GET_DEVICE_INFO_SERVICE,
                                     BATTERY_LEVEL_SERVICE,
                                     SENSOR_DATA_STATUS_SERVICE,
                                     SET_CAR_LED_SERVICE,
                                     GET_CAR_LED_SERVICE,
                                     VERIFY_MODEL_READY_SERVICE,
                                     CONFIGURE_LIDAR_SERVICE,
                                     MODEL_STATE_SERVICE,
                                     IS_MODEL_LOADING_SERVICE,
                                     CONSOLE_MODEL_ACTION_SERVICE,
                                     SOFTWARE_UPDATE_CHECK_SERVICE_NAME,
                                     BEGIN_UPDATE_SERVICE,
                                     SOFTWARE_UPDATE_STATE_SERVICE,
                                     AUTONOMOUS_THROTTLE_SERVICE,
                                     FTL_SET_MAX_SPEED_SERVICE,
                                     GET_CTRL_MODES_SERVICE,
                                     GET_OTG_LINK_STATE_SERVICE,
                                     CAL_DRIVE_TOPIC,
                                     MANUAL_DRIVE_TOPIC,
                                     SOFTWARE_UPDATE_PCT_TOPIC)


class WebServerNode(Node):
    """Node responsible for launching a Flask application as a seperate thread
       and creating service clients and subscribers.
    """

    def __init__(self):
        """Create a WebServerNode and launch a flask server with default host and port details.
        """
        super().__init__("webserver_publisher_node")
        self.get_logger().info("webserver_publisher_node started")
        HOST_DEFAULT = "0.0.0.0"
        PORT_DEFAULT = "5001"
        self.get_logger().info(f"Running the flask server on {HOST_DEFAULT}:{PORT_DEFAULT}")

        # Run the Flask webserver as a background thread.
        self.get_logger().info("Running webserver")
        self.server_thread = threading.Thread(target=app.run,
                                              daemon=True,
                                              kwargs={
                                                  "host": HOST_DEFAULT,
                                                  "port": PORT_DEFAULT,
                                                  "use_reloader": False,
                                                  "threaded": True}
                                              )
        self.server_thread.start()

        # Create service clients.

        # Create a reentrant callback group to set the vehicle mode.
        vehicle_mode_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create vehicle state service client: {VEHICLE_STATE_SERVICE}")
        self.vehicle_state_cli = self.create_client(ActiveStateSrv,
                                                    VEHICLE_STATE_SERVICE,
                                                    callback_group=vehicle_mode_cb_group)
        self.wait_for_service_availability(self.vehicle_state_cli)

        # Create a reentrant callback group to activate the state.
        enable_state_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create enable state service client: {ENABLE_STATE_SERVICE}")
        self.enable_state_cli = self.create_client(EnableStateSrv,
                                                   ENABLE_STATE_SERVICE,
                                                   callback_group=enable_state_cb_group)
        self.wait_for_service_availability(self.enable_state_cli)

        # Create a reentrant callback group to get the calibration value.
        get_calibration_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create get_cal service client: {GET_CAR_CAL_SERVICE}")
        self.get_cal_cli = self.create_client(GetCalibrationSrv,
                                              GET_CAR_CAL_SERVICE,
                                              callback_group=get_calibration_cb_group)
        self.wait_for_service_availability(self.get_cal_cli)

        # Create a mutually exclusive callback group to set the calibration value.
        set_calibration_cb_group = MutuallyExclusiveCallbackGroup()
        self.get_logger().info(f"Create set_cal service client: {SET_CAR_CAL_SERVICE}")
        self.set_cal_cli = self.create_client(SetCalibrationSrv,
                                              SET_CAR_CAL_SERVICE,
                                              callback_group=set_calibration_cb_group)
        self.wait_for_service_availability(self.set_cal_cli)

        # Create a reentrant callback group to get the device info values.
        get_device_info_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create device info service client: {GET_DEVICE_INFO_SERVICE}")
        self.get_revision_info_cli = self.create_client(GetDeviceInfoSrv,
                                                        GET_DEVICE_INFO_SERVICE,
                                                        callback_group=get_device_info_cb_group)
        self.wait_for_service_availability(self.get_revision_info_cli)

        self.get_logger().info(f"Create battery level service client: {BATTERY_LEVEL_SERVICE}")
        self.battery_level_cli = self.create_client(BatteryLevelSrv,
                                                    BATTERY_LEVEL_SERVICE,
                                                    callback_group=get_device_info_cb_group)
        self.wait_for_service_availability(self.battery_level_cli)

        self.get_logger().info(f"Create sensor status service client: {SENSOR_DATA_STATUS_SERVICE}")
        self.sensor_status_cli = self.create_client(SensorStatusCheckSrv,
                                                    SENSOR_DATA_STATUS_SERVICE,
                                                    callback_group=get_device_info_cb_group)
        self.wait_for_service_availability(self.sensor_status_cli)

        # Create a mutually exclusive callback group to set the tail light LED color values.
        set_led_color_cb_group = MutuallyExclusiveCallbackGroup()

        self.get_logger().info(f"Create set led color service client: {SET_CAR_LED_SERVICE}")
        self.set_led_color_cli = self.create_client(SetLedCtrlSrv,
                                                    SET_CAR_LED_SERVICE,
                                                    callback_group=set_led_color_cb_group)
        self.wait_for_service_availability(self.set_led_color_cli)

        # Create a reentrant callback group to set the tail light LED color values.
        get_led_color_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create get led color service client: {GET_CAR_LED_SERVICE}")
        self.get_led_color_cli = self.create_client(GetLedCtrlSrv,
                                                    GET_CAR_LED_SERVICE,
                                                    callback_group=get_led_color_cb_group)
        self.wait_for_service_availability(self.get_led_color_cli)

        # Create a reentrant callback group to call load model services.
        model_load_cb_group = ReentrantCallbackGroup()

        self.get_logger().info(f"Create verify model ready service client: {VERIFY_MODEL_READY_SERVICE}")
        self.verify_model_ready_cli = self.create_client(VerifyModelReadySrv,
                                                         VERIFY_MODEL_READY_SERVICE,
                                                         callback_group=model_load_cb_group)
        self.wait_for_service_availability(self.verify_model_ready_cli)

        self.get_logger().info(f"Create configure LiDAR service client: {CONFIGURE_LIDAR_SERVICE}")
        self.configure_lidar_cli = self.create_client(LidarConfigSrv,
                                                      CONFIGURE_LIDAR_SERVICE,
                                                      callback_group=model_load_cb_group)
        self.wait_for_service_availability(self.configure_lidar_cli)

        self.get_logger().info(f"Create model state service client: {MODEL_STATE_SERVICE}")
        self.model_state_cli = self.create_client(ModelStateSrv,
                                                  MODEL_STATE_SERVICE,
                                                  callback_group=model_load_cb_group)
        self.wait_for_service_availability(self.model_state_cli)

        # Create a reentrant callback group to call model loading status service.
        is_model_loading_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create is model loading service client: {IS_MODEL_LOADING_SERVICE}")
        self.is_model_loading_cli = self.create_client(GetModelLoadingStatusSrv,
                                                       IS_MODEL_LOADING_SERVICE,
                                                       callback_group=is_model_loading_cb_group)
        self.wait_for_service_availability(self.is_model_loading_cli)

        # Create a mutually exclusive callback group to call model action services.
        model_action_cb_group = MutuallyExclusiveCallbackGroup()

        self.get_logger().info(f"Create upload model service client: {CONSOLE_MODEL_ACTION_SERVICE}")
        self.model_action_cli = self.create_client(ConsoleModelActionSrv,
                                                   CONSOLE_MODEL_ACTION_SERVICE,
                                                   callback_group=model_action_cb_group)
        self.wait_for_service_availability(self.model_action_cli)

        # Create a reentrant callback group to call software update check service.
        sw_update_state_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create sw update state check service client: "
                               f"{SOFTWARE_UPDATE_CHECK_SERVICE_NAME}")
        self.sw_update_state_cli = self.create_client(SoftwareUpdateCheckSrv,
                                                      SOFTWARE_UPDATE_CHECK_SERVICE_NAME,
                                                      callback_group=sw_update_state_cb_group)
        self.wait_for_service_availability(self.sw_update_state_cli)

        # Create a reentrant callback group to call software update services.
        sw_update_cb_group = ReentrantCallbackGroup()

        self.get_logger().info(f"Create begin sw update service client: {BEGIN_UPDATE_SERVICE}")
        self.begin_sw_update_cli = self.create_client(BeginSoftwareUpdateSrv,
                                                      BEGIN_UPDATE_SERVICE,
                                                      callback_group=sw_update_cb_group)
        self.wait_for_service_availability(self.begin_sw_update_cli)

        self.get_logger().info("Create sw update status service client: "
                               f"{SOFTWARE_UPDATE_CHECK_SERVICE_NAME}")
        self.sw_update_status_cli = self.create_client(SoftwareUpdateStateSrv,
                                                       SOFTWARE_UPDATE_STATE_SERVICE,
                                                       callback_group=sw_update_cb_group)
        self.wait_for_service_availability(self.sw_update_status_cli)

        # Create a reentrant exclusive callback group for the software progress subscriber.
        # Guaranteed delivery is needed to send messages to late-joining subscription.
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sw_update_pct_sub_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create sw update status service client: "
                               f"{SOFTWARE_UPDATE_STATE_SERVICE}")
        self.sw_update_pct_sub = self.create_subscription(SoftwareUpdatePctMsg,
                                                          SOFTWARE_UPDATE_PCT_TOPIC,
                                                          self.sw_update_pct_sub_cb,
                                                          callback_group=self.sw_update_pct_sub_cb_group,
                                                          qos_profile=qos_profile)

        # Create a reentrant callback group to call autonomous throttle update service.
        set_throttle_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create set throttle service client: "
                               f"{AUTONOMOUS_THROTTLE_SERVICE}")
        self.set_throttle_cli = self.create_client(NavThrottleSrv,
                                                   AUTONOMOUS_THROTTLE_SERVICE,
                                                   callback_group=set_throttle_cb_group)
        self.wait_for_service_availability(self.set_throttle_cli)

        # Create a reentrant callback group to call ftl max speed update service.
        set_ftl_max_speed_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create ftl set max speed service client: "
                               f"{FTL_SET_MAX_SPEED_SERVICE}")
        self.set_ftl_max_speed_cli = self.create_client(SetMaxSpeedSrv,
                                                        FTL_SET_MAX_SPEED_SERVICE,
                                                        callback_group=set_ftl_max_speed_cb_group)
        self.wait_for_service_availability(self.set_ftl_max_speed_cli)

        # Create a reentrant callback group to call Get Ctrl Modes service.
        get_ctrl_modes_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create Get Ctrl Modes service client: "
                               f"{GET_CTRL_MODES_SERVICE}")
        self.get_ctrl_modes_cli = self.create_client(GetCtrlModesSrv,
                                                     GET_CTRL_MODES_SERVICE,
                                                     callback_group=get_ctrl_modes_cb_group)
        self.wait_for_service_availability(self.get_ctrl_modes_cli)

        # Create a reentrant callback group to call otg link state service.
        otg_link_state_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create otg link state service client: "
                               f"{GET_OTG_LINK_STATE_SERVICE}")
        self.otg_link_state_cli = self.create_client(OTGLinkStateSrv,
                                                     GET_OTG_LINK_STATE_SERVICE,
                                                     callback_group=otg_link_state_cb_group)
        self.wait_for_service_availability(self.otg_link_state_cli)

        # Create a reentrant callback group to publish manual drive messages.
        manual_pub_drive_msg_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create manual drive publisher: {MANUAL_DRIVE_TOPIC}")
        self.pub_manual_drive = self.create_publisher(ServoCtrlMsg,
                                                      MANUAL_DRIVE_TOPIC,
                                                      1,
                                                      callback_group=manual_pub_drive_msg_cb_group)

        # Create a reentrant callback group to publish calibration drive messages.
        cal_pub_drive_msg_cb_group = ReentrantCallbackGroup()
        self.get_logger().info(f"Create calibration drive publisher: {CAL_DRIVE_TOPIC}")
        self.pub_calibration_drive = self.create_publisher(ServoCtrlMsg,
                                                           CAL_DRIVE_TOPIC,
                                                           1,
                                                           callback_group=cal_pub_drive_msg_cb_group)

        # Double buffer object to safely updat the latest status and installation percentage
        # during software update setup.
        self.pct_dict_db = DoubleBuffer(clear_data_on_get=False)
        self.pct_dict_db.put({"status": "unknown",
                              "update_pct": 0.0})

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def sw_update_pct_sub_cb(self, pct_dict):
        """Callback for the software_update_pct topic.

        Args:
            pct_dict (SoftwareUpdatePctMsg): Message with the current software update
                                             progress percentage and the status.
        """
        self.pct_dict_db.put({"status": pct_dict.status,
                              "update_pct": int(pct_dict.update_pct)})

    def wait_for_service_availability(self, client):
        """Helper function to wait for the service to which the client subscribes to is alive.

        Args:
            client (rclpy.client.Client): Client object for the service to wait for.
        """
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{client.srv_name} service not available, waiting again...")


def get_webserver_node():
    """Getter function to get the global webserver node object.

    Returns:
        Node: Global webserver node object.
    """
    global webserver_node
    if 'db' not in g:
        g.webserver_node = webserver_node

    return g.webserver_node


def main(args=None):
    global webserver_node
    rclpy.init(args=args)
    webserver_node = WebServerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(webserver_node, executor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webserver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
