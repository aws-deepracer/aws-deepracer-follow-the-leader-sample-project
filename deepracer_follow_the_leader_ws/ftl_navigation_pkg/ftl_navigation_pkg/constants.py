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

ACTION_PUBLISH_TOPIC = "ftl_drive"
SET_MAX_SPEED_SERVICE_NAME = "set_max_speed"

OBJECT_DETECTION_PKG_NS = "/object_detection_pkg"
OBJECT_DETECTION_DELTA_TOPIC = f"{OBJECT_DETECTION_PKG_NS}/object_detection_delta"


class DeltaValueMap():
    """Class with the delta values mapping to action brackets
       Impact of the deltas on actions can be understood from the README.
       TODO: Link the README here.

       Experiment results with object placed at different positions wrt camera:

                               <-Reverse->    <-No Action->   <-Forward->

       LR/        |   Ahead->        30 cm    60 cm   90 cm   120 cm  150 cm
       --------------------------------------------------------------------
       45 cm      |              |
       Left       | delta_x      |    N/A     -0.37   -0.33   -0.25   -0.22
                  | delta_y      |    N/A     -0.06   -0.11   -0.14   -0.15
       15 cm      |              |
       Left       | delta_x      |   -0.25    -0.20   -0.13   -0.12   -0.09
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       0 cm       |              |
       Left/Right | delta_x      |       0        0       0       0       0
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       15 cm      |              |
       Right      | delta_x      |    0.25     0.20    0.13    0.12    0.09
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       45 cm      |              |
       Right      | delta_x      |    N/A      0.37    0.33    0.25    0.22
                  | delta_y      |    N/A     -0.06   -0.11   -0.14   -0.15

    """
    REVERSE_DELTA_Y = 0.11
    FORWARD_DELTA_Y = -0.14
    REVERSE_RIGHT_DELTA_X = -0.25
    REVERSE_LEFT_DELTA_X = 0.25
    FORWARD_RIGHT_DELTA_X = 0.13
    FORWARD_FAST_RIGHT_DELTA_X = 0.33
    FORWARD_LEFT_DELTA_X = -0.13
    FORWARD_FAST_LEFT_DELTA_X = -0.33


class ActionSpaceKeys():
    """Class with keys for the action space.
    """
    ACTION = "action"
    ANGLE = "angle"
    THROTTLE = "throttle"
    CATEGORY = "category"


class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    FORWARD = 0.3
    REVERSE = -0.3
    FAST_LEFT = 0.9
    SLOW_LEFT = 0.5
    FAST_RIGHT = -0.9
    SLOW_RIGHT = -0.5
    DEFAULT = 0.0


# Action Space configuration.
ACTION_SPACE = {
    1: {
        ActionSpaceKeys.ACTION: "No Action",
        ActionSpaceKeys.ANGLE: ActionValues.DEFAULT,
        ActionSpaceKeys.THROTTLE: ActionValues.DEFAULT,
        ActionSpaceKeys.CATEGORY: 1
    },
    2: {
        ActionSpaceKeys.ACTION: "Forward",
        ActionSpaceKeys.ANGLE: ActionValues.DEFAULT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 2
    },
    3: {
        ActionSpaceKeys.ACTION: "Slow Left, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.SLOW_LEFT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 3
    },
    4: {
        ActionSpaceKeys.ACTION: "Fast Left, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_LEFT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 4
    },
    5: {
        ActionSpaceKeys.ACTION: "Slow Right, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.SLOW_RIGHT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 5
    },
    6: {
        ActionSpaceKeys.ACTION: "Fast Right, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_RIGHT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 6
    },
    7: {
        ActionSpaceKeys.ACTION: "Reverse",
        ActionSpaceKeys.ANGLE: ActionValues.DEFAULT,
        ActionSpaceKeys.THROTTLE: ActionValues.REVERSE,
        ActionSpaceKeys.CATEGORY: 7
    },
    8: {
        ActionSpaceKeys.ACTION: "Fast Left, Reverse",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_LEFT,
        ActionSpaceKeys.THROTTLE: ActionValues.REVERSE,
        ActionSpaceKeys.CATEGORY: 8
    },
    9: {
        ActionSpaceKeys.ACTION: "Fast Right, Reverse",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_RIGHT,
        ActionSpaceKeys.THROTTLE: ActionValues.REVERSE,
        ActionSpaceKeys.CATEGORY: 9
    },
}

# Max speed percentage on a scale between 0.0 and 1.0.
# The maximum speed value is used to non linearly map the raw value obtained for the forward
# and reverse throttle to the PWM values of the servo/motor.
# We use the maximum speed % to map to a range of [1.0, 5.0] speed scale values using the
# calculated coefficients of the equation y = ax^2 + bx.
# This allows us to recalculate the curve for each maximum speed % value and use that to
# map the speed values. The idea behind this mapping is a lower percentage of maximum speed %
# should map to a higher speed scale value while calculating the coefficients so that the curve
# is more flatter and the impact of actual speed values is less for lower max speed %.
MAX_SPEED_PCT = 0.74

# Action space mapped to on the vehicle for speed values of 0.8 and 0.4.
DEFAULT_SPEED_SCALES = [1.0, 0.8]
# Speed scale bounds to pick from while calculating the coefficients.
MANUAL_SPEED_SCALE_BOUNDS = [1.0, 5.0]

# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.08
