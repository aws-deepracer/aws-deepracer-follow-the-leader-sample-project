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
constants.py

All the file location paths and constants vaules are specified here.
"""

import os
from enum import Enum

BASE_PATH = "/opt/aws/deepracer/"

# Service and topic names.

# webserver_pkg
CAL_DRIVE_TOPIC = "calibration_drive"
MANUAL_DRIVE_TOPIC = "manual_drive"

# camera_pkg
CAMERA_PKG_NS = "/camera_pkg"
MEDIA_STATE_SERVICE = f"{CAMERA_PKG_NS}/media_state"
DISPLAY_MJPEG_TOPIC = f"{CAMERA_PKG_NS}/display_mjpeg"
VIDEO_MJPEG_TOPIC = f"{CAMERA_PKG_NS}/video_mjpeg"

# ctrl_pkg
CTRL_PKG_NS = "/ctrl_pkg"
AUTO_THROTTLE_SERVICE = f"{CTRL_PKG_NS}/autonomous_throttle"
VEHICLE_STATE_SERVICE = f"{CTRL_PKG_NS}/vehicle_state"
ENABLE_STATE_SERVICE = f"{CTRL_PKG_NS}/enable_state"
MODEL_STATE_SERVICE = f"{CTRL_PKG_NS}/model_state"
IS_MODEL_LOADING_SERVICE = f"{CTRL_PKG_NS}/is_model_loading"
GET_CAR_CAL_SERVICE = f"{CTRL_PKG_NS}/get_car_cal"
SET_CAR_CAL_SERVICE = f"{CTRL_PKG_NS}/set_car_cal"
GET_CAR_LED_SERVICE = f"{CTRL_PKG_NS}/get_car_led"
SET_CAR_LED_SERVICE = f"{CTRL_PKG_NS}/set_car_led"
AUTONOMOUS_THROTTLE_SERVICE = f"{CTRL_PKG_NS}/autonomous_throttle"
GET_CTRL_MODES_SERVICE = f"{CTRL_PKG_NS}/get_ctrl_modes"

# deepracer_navigation_pkg
DEEPRACER_NAVIGATION_PKG_NS = "/deepracer_navigation_pkg"
LOAD_ACTION_SPACE_SERVICE = f"{DEEPRACER_NAVIGATION_PKG_NS}/load_action_space"
NAVIGATION_THROTTLE_SERVICE = f"{DEEPRACER_NAVIGATION_PKG_NS}/navigation_throttle"

# ftl_navigation_pkg
FTL_NAVIGATION_PKG_NS = "/ftl_navigation_pkg"
FTL_SET_MAX_SPEED_SERVICE = f"{FTL_NAVIGATION_PKG_NS}/set_max_speed"

# deepracer_systems_pkg
DEEPRACER_SYSTEMS_PKG_NS = "/deepracer_systems_pkg"
BEGIN_UPDATE_SERVICE = f"{DEEPRACER_SYSTEMS_PKG_NS}/begin_update"
SOFTWARE_UPDATE_CHECK_SERVICE_NAME = f"{DEEPRACER_SYSTEMS_PKG_NS}/software_update_check"
SOFTWARE_UPDATE_STATE_SERVICE = f"{DEEPRACER_SYSTEMS_PKG_NS}/software_update_state"
CONSOLE_MODEL_ACTION_SERVICE = f"{DEEPRACER_SYSTEMS_PKG_NS}/console_model_action"
VERIFY_MODEL_READY_SERVICE = f"{DEEPRACER_SYSTEMS_PKG_NS}/verify_model_ready"
GET_OTG_LINK_STATE_SERVICE = f"{DEEPRACER_SYSTEMS_PKG_NS}/get_otg_link_state"
SOFTWARE_UPDATE_PCT_TOPIC = f"{DEEPRACER_SYSTEMS_PKG_NS}/software_update_pct"

# device_info_pkg
DEVICE_INFO_PKG_NS = "/device_info_pkg"
GET_DEVICE_INFO_SERVICE = f"{DEVICE_INFO_PKG_NS}/get_device_info"

# i2c_pkg
I2C_PKG_NS = "/i2c_pkg"
BATTERY_LEVEL_SERVICE = f"{I2C_PKG_NS}/battery_level"

# inference_pkg
INFERENCE_PKG_NS = "/inference_pkg"
INFERENCE_STATE_SERVICE = f"{INFERENCE_PKG_NS}/inference_state"
LOAD_MODEL_SERVICE = f"{INFERENCE_PKG_NS}/load_model"
RL_RESULTS_TOPIC = f"{INFERENCE_PKG_NS}/rl_results"

# model_optimizer_pkg
MODEL_OPTIMIZER_PKG_NS = "/model_optimizer_pkg"
MODEL_OPTIMIZER_SERVER_SERVICE = f"{MODEL_OPTIMIZER_PKG_NS}/model_optimizer_server"

# sensor_fusion_pkg
SENSOR_FUSION_PKG_NS = "/sensor_fusion_pkg"
CONFIGURE_LIDAR_SERVICE = f"{SENSOR_FUSION_PKG_NS}/configure_lidar"
SENSOR_DATA_STATUS_SERVICE = f"{SENSOR_FUSION_PKG_NS}/sensor_data_status"
OVERLAY_MSG_TOPIC = f"{SENSOR_FUSION_PKG_NS}/overlay_msg"
SENSOR_MSG_TOPIC = f"{SENSOR_FUSION_PKG_NS}/sensor_msg"

# servo_pkg
SERVO_PKG_NS = "/servo_pkg"
GET_CAL_SERVICE = f"{SERVO_PKG_NS}/get_cal"
GET_LED_STATE_SERVICE = f"{SERVO_PKG_NS}/get_led_state"
SERVO_CAL_SERVICE = f"{SERVO_PKG_NS}/servo_cal"
SERVO_GPIO_SERVICE = f"{SERVO_PKG_NS}/servo_gpio"
SERVO_STATE_SERVICE = f"{SERVO_PKG_NS}/servo_state"
SET_LED_STATE_SERVICE = f"{SERVO_PKG_NS}/set_led_state"
SET_RAW_PWM_SERVICE = f"{SERVO_PKG_NS}/set_raw_pwm"

# status_led_pkg
STATUS_LED_PKG_NS = "/status_led_pkg"
LED_BLINK_SERVICE = f"{STATUS_LED_PKG_NS}/led_blink"
LED_SOLID_SERVICE = f"{STATUS_LED_PKG_NS}/led_solid"

# Logging constants.
SYS = "/var/log/syslog"
SEVER_LOG = "SERVER"

# Models constants.
MODEL_DIRECTORY_PATH = os.path.join(BASE_PATH, "artifacts/")
MODEL_FILE_TYPE = ".pb"

# parameters to load machine learning model by
MODEL_WIDTH = 160
MODEL_HEIGHT = 120

# 0 for reinforcement learning model, 1 for object detection model (not implemented)
MODEL_TASK_TYPE = 0
# These parameters are likely not to change.
MODEL_CHANNELS = 1
# 0 for mxnet, 1 for tf, and 2 for caffe.
MODEL_PLATFORM = 1
# 0 for BGR downsample, 1 for Greyscale down sample
MODEL_PRE_PROCESS = 1
# Ignored if channels are equal to 1.
MODEL_FORMAT = "RGB"

# Login constants.
TOKEN_PATH = os.path.join(BASE_PATH, "token.txt")
PASSWORD_PATH = os.path.join(BASE_PATH, "password.txt")
DEEPRACER_TOKEN = "deepracer_token"
DESTINATION_PATH = "/opt/aws/deepracer/password.txt"
DEFAULT_PASSWORD_PATH = "/sys/class/dmi/id/chassis_asset_tag"

# Control vehicle constants.
AUTONOMOUS_MODE = 1
# Max autonomous throttle value allowed on the front end.
MAX_AUTO_THROTTLE_VAL = 100.0

# Calibration constants.
PWM_ANGLE_CONVERSION = 10000
PWM_THROTTLE_CONVERSION = 4500
PWM_OFFSET = 1500000
CALIBRATION_MODE = 2

# Default values for action space mapped to on the vehicle for speed values of 0.8 and 0.4
DEFAULT_SPEED_SCALES = [1.0, 0.8]
# Speed scale bounds to pick from while calculating the coefficients
MANUAL_SPEED_SCALE_BOUNDS = [1.0, 5.0]

# Led constants.
# Scaling factor to convert r,g,b channel values to pwm values
# int(10000000 / 255)
LED_SCALING_FACTOR = 39215

# SSH constants.
DEFAULT_USER = "deepracer"
DEFAULT_SSH_PASSWORD = "deepracer"

# Wifi Settings constants.
SERVER_NAME = "deepracer.aws"
# Every one second ping the server
PING_SERVER_FREQUENCY = 1
REMOVE_IPS = ["10.0.0.1", "10.0.1.1"]

# Day 0 / Mandatory Software Update status
SOFTWARE_UPDATE_STATUS_PATH = os.path.join(BASE_PATH, "software_update_status.json")
# 1 second
SOFTWARE_UPDATE_FETCH_FREQUENCY = 1

# The sleep time is 1 more than that of the software update frequency. This will make sure
# that 100% of software completion is passed back to the browser before rebooting the vehicle.
SLEEP_TIME_BEFORE_REBOOT = SOFTWARE_UPDATE_FETCH_FREQUENCY + 1

INVALID_ENUM_VALUE = 0

# Mapping the various modes for the vehicle in ctrl_pkg
MODE_DICT = {
    0: "manual",
    1: "autonomous",
    2: "calibration",
    3: "followtheleader"
}


class SensorInputKeys(Enum):
    """Enum mapping sensor inputs keys(str) passed in model_metadata.json to int values,
       as we add sensors we should add inputs. This is also important for networks
       with more than one input.
    """
    observation = 1
    LIDAR = 2
    SECTOR_LIDAR = 3
    LEFT_CAMERA = 4
    FRONT_FACING_CAMERA = 5
    STEREO_CAMERAS = 6

    @classmethod
    def has_member(cls, input_key):
        """Check if the input_key passed as parameter is one of the class members.

        Args:
            input_key (str): String containing sensor input key to check.

        Returns:
            bool: Returns True if the sensor input key is supported, False otherwise.
        """
        return input_key in cls.__members__


# Mapping between sensor input keys and their names used for front end display
# As we add action space type we should add names here.
SENSOR_INPUT_NAME_MAPPING = {
    SensorInputKeys.observation: "Camera",
    SensorInputKeys.LIDAR: "LiDAR",
    SensorInputKeys.SECTOR_LIDAR: "LiDAR",
    SensorInputKeys.LEFT_CAMERA: "Left camera",
    SensorInputKeys.FRONT_FACING_CAMERA: "Front-facing camera",
    SensorInputKeys.STEREO_CAMERAS: "Stereo camera",
    INVALID_ENUM_VALUE: "--"

}


class TrainingAlgorithms(Enum):
    """Enum mapping training algorithm value passed in model_metadata.json to int values.
    """
    clipped_ppo = 1
    sac = 2

    @classmethod
    def has_member(cls, training_algorithm):
        """Check if the training_algorithm passed as parameter is one of the class members.

        Args:
            training_algorithm (str): String containing training algorithm to check.

        Returns:
            bool: Returns True if the training algorithm is supported, False otherwise.
        """
        return training_algorithm in cls.__members__


# Mapping between training algorithm and their names used for front end display
# As we add training algorithms we should add names here.
TRAINING_ALGORITHM_NAME_MAPPING = {
    TrainingAlgorithms.clipped_ppo: "Clipped PPO",
    TrainingAlgorithms.sac: "Soft actor critic",
    INVALID_ENUM_VALUE: "--"
}


class ActionSpaceTypes(Enum):
    """Enum mapping action space type value passed in model_metadata.json to int values.
    """
    discrete = 1
    continuous = 2

    @classmethod
    def has_member(cls, action_space_type):
        """Check if the action_space_type passed as parameter is one of the class members.

        Args:
            action_space_type (str): String containing action space type to check.

        Returns:
            bool: Returns True if the action space type is supported, False otherwise.
        """
        return action_space_type in cls.__members__


# Mapping between action space type and their names used for front end display
# As we add action space type we should add names here.
ACTION_SPACE_TYPE_NAME_MAPPING = {
    ActionSpaceTypes.continuous: "Continuous",
    ActionSpaceTypes.discrete: "Discrete",
    INVALID_ENUM_VALUE: "--"
}


class ModelMetadataKeys():
    """Class with keys in the model metadata.json
    """
    SENSOR = "sensor"
    LIDAR_CONFIG = "lidar_config"
    USE_LIDAR = "use_lidar"
    NUM_LIDAR_VALUES = "num_values"
    MIN_LIDAR_ANGLE = "min_angle"
    MAX_LIDAR_ANGLE = "max_angle"
    MIN_LIDAR_DIST = "min_distance"
    MAX_LIDAR_DIST = "max_distance"
    LIDAR_CLIPPING_DIST = "clipping_distance"
    NUM_LIDAR_SECTORS = "num_sectors"
    TRAINING_ALGORITHM = "training_algorithm"
    ACTION_SPACE_TYPE = "action_space_type"
    ACTION_SPACE = "action_space"
    CONTINUOUS_HIGH = "high"
    CONTINUOUS_LOW = "low"
    STEERING = "steering_angle"
    SPEED = "speed"


LIDAR_PREPROCESS_KEY = "preprocess_type"

# Default Lidar configuration values
DEFAULT_LIDAR_CONFIG = {
    ModelMetadataKeys.MIN_LIDAR_ANGLE: -150.0,
    ModelMetadataKeys.MAX_LIDAR_ANGLE: 150.0,
    ModelMetadataKeys.MIN_LIDAR_DIST: 0.15,
    ModelMetadataKeys.MAX_LIDAR_DIST: 0.5,
    ModelMetadataKeys.LIDAR_CLIPPING_DIST: 1.0,
    # Number of lidar values to select from the min-to-max angle range
    ModelMetadataKeys.NUM_LIDAR_VALUES: 64,
    # Number of lidar sectors to feed into network
    ModelMetadataKeys.NUM_LIDAR_SECTORS: 64,
    LIDAR_PREPROCESS_KEY: 0
}

# Default Sector Lidar configuration values
DEFAULT_SECTOR_LIDAR_CONFIG = {
    ModelMetadataKeys.MIN_LIDAR_ANGLE: -150.0,
    ModelMetadataKeys.MAX_LIDAR_ANGLE: 150.0,
    ModelMetadataKeys.MIN_LIDAR_DIST: 0.15,
    ModelMetadataKeys.MAX_LIDAR_DIST: 0.5,
    ModelMetadataKeys.LIDAR_CLIPPING_DIST: 0.5,
    # Number of lidar values to select from the min-to-max angle range
    ModelMetadataKeys.NUM_LIDAR_VALUES: 64,
    # Number of lidar sectors to feed into network
    ModelMetadataKeys.NUM_LIDAR_SECTORS: 8,
    LIDAR_PREPROCESS_KEY: 1
}
