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
webserver.py

This is the main module used to create the flask application object. All the blueprints
are imported and registered with the application object.
"""

import os
from flask_cors import CORS
from flask import Flask
from flask_wtf.csrf import CSRFProtect

from webserver_pkg.calibration import CALIBRATION_BLUEPRINT
from webserver_pkg.device_info_api import DEVICE_INFO_API_BLUEPRINT
from webserver_pkg.login import LOGIN_BLUEPRINT
from webserver_pkg.led_api import LED_API_BLUEPRINT
from webserver_pkg.models import MODELS_BLUEPRINT
from webserver_pkg.software_update import SOFTWARE_UPDATE_BLUEPRINT
from webserver_pkg.ssh_api import SSH_API_BLUEPRINT
from webserver_pkg.vehicle_logs import VEHICLE_LOGS_BLUEPRINT
from webserver_pkg.vehicle_control import VEHICLE_CONTROL_BLUEPRINT
from webserver_pkg.wifi_settings import WIFI_SETTINGS_BLUEPRINT

template_dir = os.path.abspath('/opt/aws/deepracer/lib/device_console/templates')
# Create the Flask application object.
app = Flask(__name__, template_folder=template_dir)
CORS(app)
csrf = CSRFProtect()
# Initialize the application with CSRF and register all the API blueprints.
csrf.init_app(app)
app.register_blueprint(VEHICLE_LOGS_BLUEPRINT)
app.register_blueprint(VEHICLE_CONTROL_BLUEPRINT)
app.register_blueprint(WIFI_SETTINGS_BLUEPRINT)
app.register_blueprint(LOGIN_BLUEPRINT)
app.register_blueprint(SOFTWARE_UPDATE_BLUEPRINT)
app.register_blueprint(CALIBRATION_BLUEPRINT)
app.register_blueprint(SSH_API_BLUEPRINT)
app.register_blueprint(LED_API_BLUEPRINT)
app.register_blueprint(DEVICE_INFO_API_BLUEPRINT)
app.register_blueprint(MODELS_BLUEPRINT)

app.config.update(
    DEBUG=True,
    SECRET_KEY='secret_',
    SESSION_COOKIE_SECURE=True,
    REMEMBER_COOKIE_SECURE=True)
