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
login.py

This is the module that manages the authentication, routing and login/logout
functionality for the console.
"""

import datetime
import hmac
import os
import uuid
import hashlib
from flask import (Blueprint,
                   request,
                   make_response,
                   render_template,
                   flash,
                   jsonify)

from webserver_pkg.constants import (TOKEN_PATH,
                                     PASSWORD_PATH,
                                     DEEPRACER_TOKEN,
                                     DESTINATION_PATH,
                                     DEFAULT_PASSWORD_PATH)

from webserver_pkg import webserver_publisher_node


LOGIN_BLUEPRINT = Blueprint("login", __name__)


@LOGIN_BLUEPRINT.route("/", methods=["GET"])
@LOGIN_BLUEPRINT.route("/home", methods=["GET"])
def home_page():
    """API to load home page when if authenticated else redirect to login page.

    Returns:
        flask.Response: Response object with login page or home page details.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()

    status, _ = check_authentication()
    if status:
        webserver_node.get_logger().info("Authenticated")
        return render_template("index.html")
    else:
        resp = make_response(render_template("login.html"), 200)
        return resp


@LOGIN_BLUEPRINT.route("/login", methods=["GET", "POST"])
def login():
    """API to load login page when requested as GET request and authenticate
       with password passed as parameter when requested as POST request.

    Returns:
        flask.Response: Response object with login page or home page details.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    webserver_node.get_logger().info(f"Called /login with {request.method}")
    if request.method == "POST":
        response = make_response(jsonify({"redirect": "/home"}), 200)
        try:
            with open(PASSWORD_PATH, "r") as file_ptr:
                pwd = file_ptr.readline()
            user_provided_pwd_hash = \
                str(compute_password_digest(request.form["password"].encode("utf-8")))
            if hmac.compare_digest(user_provided_pwd_hash, str(pwd)):
                webserver_node.get_logger().info("Password check passed")
                token = str(uuid.uuid4())
                try:
                    with open(TOKEN_PATH, "w") as token_file:
                        token_file.write(token)
                    webserver_node.get_logger().info("Token set")
                    response.set_cookie("deepracer_token",
                                        token,
                                        max_age=datetime.timedelta(hours=1),
                                        secure=True)
                    return response
                except IOError:
                    webserver_node.get_logger().error("Token path incorrect")
                    flash("Token path incorrect")
                    return "failure"
            else:
                webserver_node.get_logger().error("Incorrect password. Try again.")
                flash("Incorrect password. Try again.")
            return "failure"
        except IOError:
            webserver_node.get_logger().error("File not found")
            reset_default()
            flash("Password has been reset to default. "
                  "Please look at your the bottom of your vehicle for the default password.")
            return "failure"
    resp = make_response(render_template("login.html"), 200)
    return resp


@LOGIN_BLUEPRINT.route("/auth", methods=["POST", "GET"])
def auth():
    """Main authorization API which is always called from nginx to check the
       authentication for all requests. This uses the cookie as token to provide access.

    Returns:
        tuple: A tuple of response object and return status code.
    """
    if request.content_type == "application/json":
        resp = jsonify("")
    else:
        resp = make_response("")
    status, cookie_val = check_authentication()
    if not status:
        return resp, 401
    resp.set_cookie("deepracer_token",
                    cookie_val,
                    max_age=datetime.timedelta(hours=1),
                    secure=True)
    return resp, 200


@LOGIN_BLUEPRINT.route("/logout", methods=["GET"])
@LOGIN_BLUEPRINT.route("/redirect_login", methods=["GET", "POST"])
def logout():
    """API to redirect to login page with reset deepracer_token in the cookie.

    Returns:
        flask.Response: Response object with login page details.
    """
    response = make_response(render_template("login.html"), 200)
    response.set_cookie("deepracer_token", expires=0)
    return response


@LOGIN_BLUEPRINT.route("/api/password", methods=["POST", "GET"])
def update_password_api():
    """API that updates the console password to the new one sent as a parameter.

    Returns:
        dict: Execution status if the API call was successful, flag to indicate
              if password update was successful and error reason if call fails.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()

    old_password = request.json.get("old_password").encode("utf-8")
    new_password = request.json.get("new_password").encode("utf-8")

    try:
        with open(PASSWORD_PATH, "r") as pwd_file:
            password_digest = pwd_file.readline()

        if hmac.compare_digest(compute_password_digest(old_password), password_digest):
            with open(PASSWORD_PATH, "w") as pwd_file:
                pwd_file.write(compute_password_digest(new_password))
                pwd_file.close()
            return jsonify({"success": True})
        else:
            return jsonify({"success": False,
                            "reason": "The password is incorrect. Provide your current password."})
    except IOError:
        webserver_node.get_logger().error("Password.txt not found in given path")
        return jsonify({"success": False, "reason": "Password.txt not found."})


def check_authentication():
    """Helper function to validate the token sent in the request object.

    Returns:
        tuple: A tuple of validation flag and cookie value.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()
    if not os.path.exists(TOKEN_PATH):
        webserver_node.get_logger().error("Token path not found")
        return False, None

    token = ""
    with open(TOKEN_PATH, "r") as file_ptr:
        token = file_ptr.readline()

    cookie_val = str(request.cookies.get(DEEPRACER_TOKEN))
    status = True if(hmac.compare_digest(cookie_val, token)) else False
    webserver_node.get_logger().info(f"Cookie compare status: {status}")
    return status, cookie_val


def compute_password_digest(message):
    """Helper method to compute the message digest for the given string.
    """
    return hashlib.sha224(message).hexdigest()


def reset_default():
    """Helper method to reset the password to the default password found on vehicle.
    """
    webserver_node = webserver_publisher_node.get_webserver_node()

    if os.path.exists(DEFAULT_PASSWORD_PATH):
        webserver_node.get_logger().info("Default password file found")
        with open(DEFAULT_PASSWORD_PATH, "r") as pwd_file:
            default_pass = pwd_file.readline().strip()
    else:
        default_pass = "deepracer"
        webserver_node.get_logger().info("Default password file not found")
    with open(DESTINATION_PATH, "w") as pwd_file:
        pwd_file.write(compute_password_digest(default_pass))
    webserver_node.get_logger().info("Password reset to default")
