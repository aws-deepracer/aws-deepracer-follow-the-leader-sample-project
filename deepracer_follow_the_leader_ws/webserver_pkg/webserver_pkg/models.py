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
models.py

This is the module that holds the APIs and utility functions required to manage
the AWS DeepRacer reinforcement models. It contains APIs to fetch the details about the
models, execute upload of tar.gz files, delete models, load models before running
inference.
"""
import itertools
import time
import fnmatch
import os
import shutil
import json
from werkzeug.utils import secure_filename
from flask import (Blueprint,
                   jsonify,
                   request,
                   Response)

from deepracer_interfaces_pkg.srv import (ModelStateSrv,
                                          ConsoleModelActionSrv,
                                          GetModelLoadingStatusSrv,
                                          VerifyModelReadySrv,
                                          LidarConfigSrv,
                                          SensorStatusCheckSrv)
from webserver_pkg import utility
from webserver_pkg.utility import call_service_sync
from webserver_pkg import constants
from webserver_pkg import webserver_publisher_node


MODELS_BLUEPRINT = Blueprint("models", __name__)


def get_file_and_folder_info(path):
    """Helper function to get the file and folder information for the model at the
       model path sent as parameter.

    Args:
        path (str): Model directory path.

    Returns:
        dict: Dictonary with the relevant details about the model.
    """
    training_algorithm_display_name = \
        constants.TRAINING_ALGORITHM_NAME_MAPPING[constants.INVALID_ENUM_VALUE]
    action_space_type_display_name = \
        constants.ACTION_SPACE_TYPE_NAME_MAPPING[constants.INVALID_ENUM_VALUE]
    model_metadata_sensors_display_names = \
        [constants.SENSOR_INPUT_NAME_MAPPING[constants.INVALID_ENUM_VALUE]]

    size = utility.execute(["du", "-sh", path])[1].split("\t")[0]
    err_code, err_msg, model_metadata_content = \
        read_model_metadata_file(os.path.join(path, "model_metadata.json"))
    if err_code == 0:
        err_code, err_msg, model_metadata_sensors = get_sensors(model_metadata_content)
        if err_code == 0:
            model_metadata_sensors_display_names = [constants.SENSOR_INPUT_NAME_MAPPING[
                                                    constants.SensorInputKeys(sensor)]
                                                    for sensor in model_metadata_sensors]
        err_code, err_msg, training_algorithm = get_training_algorithm(model_metadata_content)
        if err_code == 0:
            training_algorithm_display_name = \
                constants.TRAINING_ALGORITHM_NAME_MAPPING[
                    constants.TrainingAlgorithms(training_algorithm)]
        err_code, err_msg, action_space_type = get_action_space_type(model_metadata_content)
        if err_code == 0:
            action_space_type_display_name = \
                constants.ACTION_SPACE_TYPE_NAME_MAPPING[
                    constants.ActionSpaceTypes(action_space_type)]
    data = {
        "name": os.path.basename(path),
        "size": size,
        "creation_time": os.path.getmtime(path),
        "status": "Ready",
        "training_algorithm": training_algorithm_display_name,
        "action_space_type": action_space_type_display_name,
        "sensors": ", ".join(model_metadata_sensors_display_names)
    }
    return data


def get_sensor_status():
    """Helper function to call the service to get the status of the sensor connected to the vehicle.

    Returns:
        tuple: A tuple with error code and SensorStatusCheckSrv.Response object.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        sensor_status_req = SensorStatusCheckSrv.Request()
        sensor_status_res = call_service_sync(webserver_node.sensor_status_cli,
                                              sensor_status_req)
        if sensor_status_res and sensor_status_res.error == 0:
            webserver_node.get_logger().info("Verify required sensor status: "
                                             f"Camera status: {sensor_status_res.single_camera_status}, "
                                             f"Stereo status: {sensor_status_res.stereo_camera_status}, "
                                             f"Lidar status: {sensor_status_res.lidar_status}")
            return 0, sensor_status_res
        else:
            webserver_node.get_logger().error("Failed to call the sensor status service")
            return 1, {}
    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to reach sensor status server: {ex}")
        return 1, {}


def verify_sensor_connection(sensors, sensor_resp):
    """Helper function to verify if the required sensors that the model trained with are
       connected to the vehicle.

    Args:
        sensors (list): List of SensorInputKeys enum values
        sensor_resp (SensorStatusCheckSrv.Response): Sensor data status for camera and
                                                     LiDAR sensors connected to the vehicle.

    Returns:
        tuple: A tuple of error code and error message.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    required_sensors_not_connected = []
    try:
        if (constants.SensorInputKeys.SECTOR_LIDAR in sensors
           or constants.SensorInputKeys.LIDAR in sensors) and sensor_resp.lidar_status == 1:
            required_sensors_not_connected.append(
                constants.SENSOR_INPUT_NAME_MAPPING[constants.SensorInputKeys.LIDAR])

        if constants.SensorInputKeys.STEREO_CAMERAS in sensors \
           and sensor_resp.stereo_camera_status == 1:
            required_sensors_not_connected.append(
                constants.SENSOR_INPUT_NAME_MAPPING[constants.SensorInputKeys.STEREO_CAMERAS])

        if (constants.SensorInputKeys.FRONT_FACING_CAMERA in sensors
           or constants.SensorInputKeys.observation in sensors
           or constants.SensorInputKeys.LEFT_CAMERA in sensors) \
           and sensor_resp.single_camera_status == 1:
            required_sensors_not_connected.append(
                constants.SENSOR_INPUT_NAME_MAPPING[constants.SensorInputKeys.FRONT_FACING_CAMERA])

    except Exception as ex:
        webserver_node.get_logger().error(f"Unable to verify sensor connections: {ex}")
        return 1, "Unable to verify sensor connections."
    finally:
        if len(required_sensors_not_connected) > 0:
            return 1, (f"Check the connections for the {', '.join(required_sensors_not_connected)} "
                       f"{'sensors' if len(required_sensors_not_connected) > 1 else 'sensor'}")
        return 0, ""


def read_model_metadata_file(model_metatdata_file):
    """Helper method that reads the model metadata file for the model selected and validates
       the sensor configuration.

    Args:
        model_metatdata_file (str): Path to the model_metadata file.

    Returns:
        tuple: A tuple of error code, error message and the JSON data
               with the content read from model_metadata.json of the model.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        if(not os.path.isfile(model_metatdata_file)):
            webserver_node.get_logger().error(f"No model_metadata_file: {model_metatdata_file}.")
            # return 1 which means model_metadata file is not found
            return 1, "No model_metadata_file for the model selected", {}
        with open(model_metatdata_file) as json_file:
            data = json.load(json_file)

        return 0, "", data
    except Exception as ex:
        webserver_node.get_logger().error(f"Error while reading model_metadata.json: {ex}")
        return 1, "Error while reading model_metadata.json", {}


def validate_action_space(action_space, action_space_type):
    """Helper method that validates the action space for specific action space type.

    Args:
        action_space (dict): Dictionary with the action space information.
        action_space_type (ActionSpaceTypes): ActionSpaceTypes enum value indicating
                                              action space type.

    Returns:
        bool: True if the action space and action space type combinatino is valid
              else False.
    """
    valid_action_space = True
    if action_space_type == constants.ActionSpaceTypes.discrete:
        for action_dict in action_space:
            if (constants.ModelMetadataKeys.STEERING not in action_dict
               or constants.ModelMetadataKeys.SPEED not in action_dict):
                valid_action_space = False
                break
    elif action_space_type == constants.ActionSpaceTypes.continuous:
        if (constants.ModelMetadataKeys.STEERING not in action_space
           or constants.ModelMetadataKeys.SPEED not in action_space):
            valid_action_space = False
        else:
            steering_action_dict = action_space[constants.ModelMetadataKeys.STEERING]
            speed_action_dict = action_space[constants.ModelMetadataKeys.SPEED]
            if (constants.ModelMetadataKeys.CONTINUOUS_HIGH not in steering_action_dict
               or constants.ModelMetadataKeys.CONTINUOUS_LOW not in steering_action_dict
                or constants.ModelMetadataKeys.CONTINUOUS_HIGH not in speed_action_dict
                or constants.ModelMetadataKeys.CONTINUOUS_LOW not in speed_action_dict
                or steering_action_dict[constants.ModelMetadataKeys.CONTINUOUS_HIGH]
                <= steering_action_dict[constants.ModelMetadataKeys.CONTINUOUS_LOW]
               or speed_action_dict[constants.ModelMetadataKeys.CONTINUOUS_HIGH]
               <= speed_action_dict[constants.ModelMetadataKeys.CONTINUOUS_LOW]):
                valid_action_space = False
    return valid_action_space


def get_sensors(model_metatdata_json):
    """Helper method that returns the SensorInputKeys enum values from model_metadata json
       of the model selected.

    Args:
        model_metatdata_json (dict): JSON data with the content read from
                                     model_metadata.json of the model.

    Returns:
        tuple: A tuple of error code, error message and the list of SensorInputKeys enum values.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        sensors = None
        if constants.ModelMetadataKeys.SENSOR in model_metatdata_json:
            sensor_names = set(model_metatdata_json[constants.ModelMetadataKeys.SENSOR])
            if all([constants.SensorInputKeys.has_member(sensor_name)
                    for sensor_name in sensor_names]):
                sensors = [constants.SensorInputKeys[sensor_name]
                           for sensor_name in sensor_names]
            else:
                webserver_node.get_logger().error("The sensor configurations of your vehicle "
                                                  "and trained model must match")
                return (2,
                        "The sensor configurations of your vehicle and "
                        "trained model must match",
                        [])
        else:
            # To handle DeepRacer models with no sensor key
            webserver_node.get_logger().info("No sensor key in model_metadata_file. "
                                             "Defaulting to observation.")
            sensors = [constants.SensorInputKeys.observation]

        return 0, "", sensors
    except Exception as ex:
        webserver_node.get_logger().error("Error while getting sensor names "
                                          f"from model_metadata.json: {ex}")
        return 1, "Error while getting sensor names from model_metadata.json", []


def get_training_algorithm(model_metatdata_json):
    """Helper method that returns the training algorithm from model_metadata json of the
       model selected.

    Args:
        model_metatdata_json (dict): JSON data with the content read from
                                     model_metadata.json of the model.

    Returns:
        tuple: A tuple of error code, error message and the TrainingAlgorithms enum value.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        training_algorithm = None
        if constants.ModelMetadataKeys.TRAINING_ALGORITHM in model_metatdata_json:
            training_algorithm_value = \
                model_metatdata_json[constants.ModelMetadataKeys.TRAINING_ALGORITHM]
            if constants.TrainingAlgorithms.has_member(training_algorithm_value):
                training_algorithm = constants.TrainingAlgorithms[training_algorithm_value]
            else:
                webserver_node.get_logger().error("The training algorithm value is incorrect")
                return 2, "The training algorithm value is incorrect", ""
        else:
            # To handle DeepRacer models with no training_algorithm key
            webserver_node.get_logger().info("No training algorithm key in model_metadata_file. "
                                             "Defaulting to clipped_ppo.")
            training_algorithm = constants.TrainingAlgorithms.clipped_ppo

        return 0, "", training_algorithm
    except Exception as ex:
        webserver_node.get_logger().error("Error while getting training algorithm from "
                                          f"model_metadata.json: {ex}")
        return 1, "Error while getting training algorithm from model_metadata.json", ""


def get_action_space_type(model_metatdata_json):
    """Helper method that returns the action space type from model_metadata json of the
       model selected.

    Args:
        model_metatdata_json (dict): JSON data with the content read from
                                     model_metadata.json of the model.

    Returns:
        tuple: A tuple of error code, error message and the ActionSpaceTypes enum value.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        action_space_type = None
        if constants.ModelMetadataKeys.ACTION_SPACE_TYPE in model_metatdata_json:
            action_space_type_value = \
                model_metatdata_json[constants.ModelMetadataKeys.ACTION_SPACE_TYPE]
            if constants.ActionSpaceTypes.has_member(action_space_type_value):
                action_space_type = constants.ActionSpaceTypes[action_space_type_value]
            else:
                webserver_node.get_logger().error("The action space type value is incorrect")
                return 2, "The action space type value is incorrect", ""
        else:
            # To handle DeepRacer models with no action space type key
            webserver_node.get_logger().info("No action space type key in model_metadata_file. "
                                             "Defaulting to discrete.")
            action_space_type = constants.ActionSpaceTypes.discrete

        return 0, "", action_space_type
    except Exception as ex:
        webserver_node.get_logger().error("Error while getting action space type from "
                                          f"model_metadata.json: {ex}")
        return 1, "Error while getting action space type from model_metadata.json", ""


def get_action_space(model_metatdata_json):
    """Helper method that returns the action space from model_metadata json of the
       model selected.

    Args:
        model_metatdata_json (dict): JSON data with the content read from
                                     model_metadata.json of the model.

    Returns:
        tuple: A tuple of error code, error message and the action space data.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        action_space = None
        if constants.ModelMetadataKeys.ACTION_SPACE in model_metatdata_json:
            action_space = model_metatdata_json[constants.ModelMetadataKeys.ACTION_SPACE]
        else:
            return 2, "No action space value found in model_metadata.json", {}
        return 0, "", action_space
    except Exception as ex:
        webserver_node.get_logger().error("Error while getting action space "
                                          " frommodel_metadata.json: {ex}")
        return 1, "Error while getting action space model_metadata.json", {}


def load_lidar_configuration(sensors, data):
    """Helper method to load the LiDAR configuration data if present in the model_metadata.json
       file of the data.

    Args:
        sensors (list): List of SensorInputKeys enum objects which the model was trained on.
        data (dict): JSON data with the content read from model_metadata.json of the model.

    Returns:
        tuple: A tuple of error code, error message and the LiDAR configuration dictionary.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    try:
        # Set default values in case the "lidar configuration" is not
        # defined in model_metadata.json.
        model_lidar_config = constants.DEFAULT_LIDAR_CONFIG
        # Set default values for SECTOR_LIDAR if this sensor is used
        if constants.SensorInputKeys.SECTOR_LIDAR in sensors:
            model_lidar_config = constants.DEFAULT_SECTOR_LIDAR_CONFIG
        model_lidar_config[
            constants.ModelMetadataKeys.USE_LIDAR
        ] = sensors and (constants.SensorInputKeys.LIDAR in sensors
                         or constants.SensorInputKeys.SECTOR_LIDAR in sensors)
        # Load the lidar configuration if the model uses lidar and has custom
        # lidar configurations.
        if model_lidar_config[constants.ModelMetadataKeys.USE_LIDAR] \
           and constants.ModelMetadataKeys.LIDAR_CONFIG in data:
            lidar_config = data[constants.ModelMetadataKeys.LIDAR_CONFIG]
            model_lidar_config[
                constants.ModelMetadataKeys.NUM_LIDAR_VALUES
            ] = lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_VALUES]
            model_lidar_config[
                constants.ModelMetadataKeys.MIN_LIDAR_ANGLE
            ] = lidar_config[constants.ModelMetadataKeys.MIN_LIDAR_ANGLE]
            model_lidar_config[
                constants.ModelMetadataKeys.MAX_LIDAR_ANGLE
            ] = lidar_config[constants.ModelMetadataKeys.MAX_LIDAR_ANGLE]
            model_lidar_config[
                constants.ModelMetadataKeys.MIN_LIDAR_DIST
            ] = lidar_config[constants.ModelMetadataKeys.MIN_LIDAR_DIST]
            model_lidar_config[
                constants.ModelMetadataKeys.MAX_LIDAR_DIST
            ] = lidar_config[constants.ModelMetadataKeys.MAX_LIDAR_DIST]
            model_lidar_config[
                constants.ModelMetadataKeys.LIDAR_CLIPPING_DIST
            ] = lidar_config[constants.ModelMetadataKeys.LIDAR_CLIPPING_DIST]
            model_lidar_config[
                constants.ModelMetadataKeys.NUM_LIDAR_SECTORS
            ] = lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_SECTORS]
        return 0, "", model_lidar_config
    except Exception as ex:
        webserver_node.get_logger().error("Unable to connect to vehicle with current "
                                          f"LiDAR configuration: {ex}")
        return 1, "Unable to connect to vehicle with current LiDAR configuration", {}


@MODELS_BLUEPRINT.route("/api/models", methods=["GET"])
def api_list_models():
    """API to get all the models and their details, and sensor status to populate the
       model list drop down.

    Returns:
        dict: List of models and their status based on connected sensor information.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    data = {
        "models": [],
        "sensor_status": {"camera_status": "error",
                          "stereo_status": "error",
                          "lidar_status": "error"}
        }
    try:
        ext_length = len(constants.MODEL_FILE_TYPE)
        disabled_models = list()
        sensor_status_info = dict()
        sensor_status_code, sensor_resp = get_sensor_status()
        if sensor_status_code == 0:
            sensor_status_info["camera_status"] = \
                "not_connected" if sensor_resp.single_camera_status == 1 else "connected"
            sensor_status_info["stereo_status"] = \
                "not_connected" if sensor_resp.stereo_camera_status == 1 else "connected"
            sensor_status_info["lidar_status"] = \
                "not_connected" if sensor_resp.lidar_status == 1 else "connected"
            data["sensor_status"] = sensor_status_info
        for root, _, filenames in os.walk(constants.MODEL_DIRECTORY_PATH):
            for file_name in fnmatch.filter(filenames, "*" + constants.MODEL_FILE_TYPE):
                model_folder_name = os.path.basename(root)
                if model_folder_name:
                    err_code, err_msg, model_metadata_content = \
                        read_model_metadata_file(os.path.join(
                                                    os.path.join(constants.MODEL_DIRECTORY_PATH,
                                                                 model_folder_name),
                                                    "model_metadata.json"))
                    model_metadata_sensors = None
                    training_algorithm = ""
                    action_space_type = ""
                    if err_code == 0:
                        err_code, err_msg, model_metadata_sensors = \
                            get_sensors(model_metadata_content)
                    if err_code == 0:
                        err_code, err_msg, training_algorithm = \
                            get_training_algorithm(model_metadata_content)
                    if err_code == 0:
                        err_code, err_msg, action_space_type = \
                            get_action_space_type(model_metadata_content)
                    if err_code == 0:
                        model_disabled = False
                        if sensor_status_code == 1:
                            model_disabled = True
                        else:
                            sensor_status, _ = verify_sensor_connection(model_metadata_sensors,
                                                                        sensor_resp)
                            model_disabled = sensor_status == 1
                        model = {"model_name": file_name[:-ext_length],
                                 "model_folder_name": model_folder_name,
                                 "model_training_algorithm":
                                 constants.TRAINING_ALGORITHM_NAME_MAPPING[
                                    constants.TrainingAlgorithms(training_algorithm)],
                                 "model_action_space_type":
                                 constants.ACTION_SPACE_TYPE_NAME_MAPPING[
                                    constants.ActionSpaceTypes(action_space_type)],
                                 "model_sensors":
                                 [constants.SENSOR_INPUT_NAME_MAPPING[
                                    constants.SensorInputKeys(sensor)]
                                     for sensor in model_metadata_sensors],
                                 "is_select_disabled": model_disabled}
                        if not model_disabled:
                            data["models"].append(model)
                        else:
                            disabled_models.append(model)
        data["models"] += disabled_models
    except AttributeError:
        webserver_node.get_logger().error("Model folder not found")
    except OSError:
        webserver_node.get_logger().error("Model not found")
    return jsonify(data)


@MODELS_BLUEPRINT.route("/api/models/<model_folder_name>/<model_name>", methods=["PUT", "POST"])
def api_load_model(model_folder_name, model_name):
    """API to call the service to load model for inference after verifying if its in valid state.

    Returns:
        dict: Execution status if the API call was successful and the error reason if failed.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info("Loading {}/{}".format(model_folder_name, model_name))
    try:
        verify_model_ready_req = VerifyModelReadySrv.Request()
        verify_model_ready_req.model_name = model_folder_name
        model_verification_res = call_service_sync(webserver_node.verify_model_ready_cli,
                                                   verify_model_ready_req)
        webserver_node.get_logger().info("Model verification completed result "
                                         f"{model_verification_res.response_status}")
        if model_verification_res is None or not model_verification_res.response_status:
            return jsonify({"success": False, "reason": "Model optimization failed!"})

        err_code, err_msg, model_metadata_content = \
            read_model_metadata_file(os.path.join(
                                        os.path.join(constants.MODEL_DIRECTORY_PATH,
                                                     model_folder_name),
                                        "model_metadata.json"))

        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})
        err_code, err_msg, model_metadata_sensors = get_sensors(model_metadata_content)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})
        err_code, err_msg, training_algorithm = get_training_algorithm(model_metadata_content)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})

        err_code, err_msg, action_space_type = get_action_space_type(model_metadata_content)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})

        err_code, err_msg, action_space = get_action_space(model_metadata_content)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})

        if not validate_action_space(action_space, action_space_type):
            return jsonify({"success": False,
                            "reason": "Incorrect values in model_metadata.json"})

        err_code, sensor_resp = get_sensor_status()
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": "Error getting sensor status"})

        err_code, err_msg = verify_sensor_connection(model_metadata_sensors, sensor_resp)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})

        err_code, err_msg, model_lidar_config = load_lidar_configuration(model_metadata_sensors,
                                                                         model_metadata_content)
        if err_code > 0:
            return jsonify({"success": False,
                            "reason": err_msg})

        # Configure lidar service. Call the service to load model specific lidar configuration
        # or to reset the lidar configuration to default.
        configure_lidar_req = LidarConfigSrv.Request()
        configure_lidar_req.use_lidar = \
            model_lidar_config[constants.ModelMetadataKeys.USE_LIDAR]
        configure_lidar_req.min_angle = \
            model_lidar_config[constants.ModelMetadataKeys.MIN_LIDAR_ANGLE]
        configure_lidar_req.max_angle = \
            model_lidar_config[constants.ModelMetadataKeys.MAX_LIDAR_ANGLE]
        configure_lidar_req.num_values = \
            model_lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_VALUES]
        configure_lidar_req.min_distance = \
            model_lidar_config[constants.ModelMetadataKeys.MIN_LIDAR_DIST]
        configure_lidar_req.max_distance = \
            model_lidar_config[constants.ModelMetadataKeys.MAX_LIDAR_DIST]
        configure_lidar_req.clipping_distance = \
            model_lidar_config[constants.ModelMetadataKeys.LIDAR_CLIPPING_DIST]
        configure_lidar_req.num_sectors = \
            model_lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_SECTORS]
        configure_lidar_req.preprocess_type = \
            model_lidar_config[constants.LIDAR_PREPROCESS_KEY]
        configure_lidar_res = call_service_sync(webserver_node.configure_lidar_cli,
                                                configure_lidar_req)
        webserver_node.get_logger().info("Setting lidar configuration is completed: "
                                         f"{configure_lidar_res.error}")
        if configure_lidar_res is None or configure_lidar_res.error != 0:
            return jsonify({"success": False, "reason": "Setting lidar configuration failed!"})

        model_path = model_folder_name + "/" + model_name

        model_state_req = ModelStateSrv.Request()
        model_state_req.model_name = model_path
        model_state_req.model_metadata_sensors = [sensor.value for sensor
                                                  in model_metadata_sensors]
        model_state_req.training_algorithm = training_algorithm.value
        model_state_req.action_space_type = action_space_type.value
        model_state_req.img_format = constants.MODEL_FORMAT
        model_state_req.width = constants.MODEL_WIDTH
        model_state_req.height = constants.MODEL_HEIGHT
        model_state_req.num_channels = constants.MODEL_CHANNELS
        model_state_req.lidar_channels = \
            model_lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_SECTORS]
        model_state_req.platform = constants.MODEL_PLATFORM
        model_state_req.task_type = constants.MODEL_TASK_TYPE
        model_state_req.pre_process_type = constants.MODEL_PRE_PROCESS

        model_state_res = call_service_sync(webserver_node.model_state_cli,
                                            model_state_req)
        if model_state_res and model_state_res.error == 0:
            webserver_node.get_logger().info("Model Loaded")
            return jsonify({"success": True})
        else:
            webserver_node.get_logger().error("Model optimizer error")
            return jsonify({"success": False,
                            "reason": "Model load failed! Please check the ROS logs"})
    except Exception as ex:
        webserver_node.get_logger().error("Unable to reach model state server: {ex}")
        return jsonify({"success": False,
                        "reason": "Unable to reach model state server"})


@MODELS_BLUEPRINT.route("/api/uploaded_model_list", methods=["GET"])
def get_uploaded_models_list():
    """API to get all the available models to display as table.

    Returns:
        list: List of models and their details.
    """
    model_list = list()
    for cur_dir in os.listdir(constants.MODEL_DIRECTORY_PATH):
        model_list.append(
            get_file_and_folder_info(
                os.path.join(constants.MODEL_DIRECTORY_PATH,
                             cur_dir)))
    return jsonify(model_list)


@MODELS_BLUEPRINT.route("/api/uploadModels", methods=["POST", "PUT"])
def model_file_upload():
    """API to call the service to upload models to the artifacts directory.

    Returns:
        dict: Execution status if the API call was successful and the response message.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    #
    # Check if the file received is a tar.gz
    #
    file_obj = request.files["file"]
    file_name = file_obj.filename
    secured_file = secure_filename(file_name)
    if file_name.endswith(".tar.gz"):
        folder_name = file_name[:-7]
    else:
        return jsonify({"success": False,
                        "message": "Failed to upload the model. Not a .tar.gz file"})

    # Always create a new directory. If the folder already exists.
    # Delete and Create a new one.
    dir_path = os.path.join(constants.MODEL_DIRECTORY_PATH, folder_name)
    if os.path.exists(dir_path):
        shutil.rmtree(dir_path)
    os.makedirs(dir_path)

    # Save the uploaded file in the artifacts directory.
    webserver_node.get_logger().info("Uploaded model file: {}".format(file_name))
    file_obj.save(os.path.join(dir_path, secured_file))

    # Optimizing the model once the file is uploaded.
    # Converting the .tar.gz to optimized inference model.
    upload_model_req = ConsoleModelActionSrv.Request()
    upload_model_req.model_path = dir_path

    # action=1 (For upload the model) & action=0 for deleting the model
    upload_model_req.action = 1
    upload_model_res = call_service_sync(webserver_node.model_action_cli, upload_model_req)
    if upload_model_res:
        webserver_node.get_logger().info(f"Uploaded model status return {upload_model_res.status}")
        if upload_model_res.status == "done-upload":
            return jsonify({"success": True,
                            "message": "Model uploaded successfully to your vehicle"})
    return jsonify({"success": False,
                    "message": "Failed to upload & optimize the model"})


@MODELS_BLUEPRINT.route("/api/deleteModels", methods=["POST", "PUT"])
def delete_model_folder():
    """API to call the service to delete models passed as parameter. Takes the folder
       path as parameter and deletes all the files in that directory.

    Returns:
        dict: Execution status if the API call was successful and the response message.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    req = request.json
    filenames = req["filenames"]
    webserver_node.get_logger().info(f"Deleting Models {filenames}")
    deleted_all_models = True

    # Loop through the filenames and delete the models.
    for filename in filenames:
        delete_model_req = ConsoleModelActionSrv.Request()
        delete_model_req.model_path = os.path.join(constants.MODEL_DIRECTORY_PATH, filename)

        # action=1 (For upload the model) & action=0 for deleting the model.
        delete_model_req.action = 0
        delete_model_res = call_service_sync(webserver_node.model_action_cli, delete_model_req)
        if delete_model_res:
            webserver_node.get_logger().info("Delete model status return "
                                             f"{delete_model_res.status}")
            if delete_model_res.status != "done-delete":
                deleted_all_models = False
        else:
            deleted_all_models = False
    if deleted_all_models:
        return jsonify({"success": True,
                        "message": "Models deleted successfully from your vehicle"})
    return jsonify({"success": False,
                    "message": "Failed to delete the selected models"})


@MODELS_BLUEPRINT.route("/api/is_model_installed", methods=["GET"])
def is_model_installed():
    """API to call the service to verify if the extraction and installation of the model
       was successful.

    Returns:
        dict: Execution status if the API call was successful and the response message.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    modelname = request.args.get("filename")

    webserver_node.get_logger().info(f"Check if model {modelname} already installed")
    for cur_dir in os.listdir(constants.MODEL_DIRECTORY_PATH):
        if cur_dir == modelname:
            return jsonify({"success": True,
                            "message": "Model is installed"})

    return jsonify({"success": False,
                    "message": "Model is not installed"})


@MODELS_BLUEPRINT.route("/api/isModelLoading", methods=["GET"])
def is_model_loading():
    """API to stream the model loading status.

    Returns:
        flask.Response: Flask response object with the content_type set to text/event-stream.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    is_model_loading_req = GetModelLoadingStatusSrv.Request()
    is_model_loading_res = call_service_sync(webserver_node.is_model_loading_cli,
                                             is_model_loading_req)
    model_loading_status = "error"
    if is_model_loading_res is not None and is_model_loading_res.error == 0:
        model_loading_status = is_model_loading_res.model_loading_status
    return jsonify({"success": True,
                    "isModelLoading": model_loading_status})
