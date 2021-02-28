///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef CTRL_STATE_BASE_HPP
#define CTRL_STATE_BASE_HPP

#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"

namespace SysCtrl {
    class CtrlStateBase
    {
    /// This class defines the interface for the states of the control node.
    public:
        /// Call back that will manage servo request, each state is supposed to subscribe to the relevant
        /// topic to receive the correct messages.
        /// @param msg Generic servo message to be sent by desired publisher.
        virtual void servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) = 0;
        /// Each state may receive a load model request, this method should handle the request.
        /// @returns True if model loaded successfully.
        /// @param requestSeqNum Sequence number of the request, used to prioritize latest requests.
        /// @param modelName Name of the model to be loaded, this name must match the name of the model artifacts.
        /// @param modelMetadataSensors List of sensors in the model metadata file.
        /// @param trainingAlgorithm Training algorithm passed in the model metadata file.
        /// @param actionSpaceType Action space type passed in the model metadata file.
        /// @param imgFormat Format of the images to be feed to the inference engine, only valid if numChannels is 3.
        /// @param width Width of the images to be feed to the inference engine.
        /// @param height Height of the images to be feed to the inference engine.
        /// @param numChannels Number of color channels of the images to be feed to the inference engine, set to 1 for grey scale.
        /// @param lidarChannels Number of lidar values to be fed to the inference engine, set to 64 for LIDAR and 8 for SECTOR_LIDAR sensor.
        /// @param platform Platform that the model was trained in. [mxnet, tf, caffe]
        /// @param task Inference task for which the model is intended [object detection or reinforcement learning]
        /// @param preProcess Preprocessing algorithm to be used on input images.
        virtual bool loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors,
                                  int trainingAlgorithm, int actionSpaceType, std::string imgFormat,
                                  int width, int height, int numChannels, 
                                  int lidarChannels, int platform, int task, int preProcess) = 0;
        /// Method in charge of activating/deactivating the state.
        /// @param isActive True to start the state, false to deactivate the state.
        virtual void setStateActive(bool isActive) = 0;
        /// Method in charge of getting the latest load model request status.
        /// @returns Load model request status.
        virtual std::string getLoadModelStatus() = 0;
        /// Retrieves the calibration by reference for a desired servo type.
        /// @param type Servo type for which the calibration is to be retrieved.
        /// @param cal Vector containing the calibration data for the desired servo type.
        virtual void getCalibration(int type, std::vector<int> &cal) = 0;
        /// Sets the calibrations of a desired servo type.
        /// @param type Servo type for which the calibration is to be set.
        /// @param cal Vector containing the calibration data to be set.
        virtual void setCalibration(int type, const std::vector<int> &cal) = 0;
        /// Sets the throttle to a constant value.
        /// @param throttle Desired throttle value.
        virtual void setConstantThrottle(float throttle) = 0;
        /// Get the LED value that is stored in the file.
        /// @param ledValuesMap Vector to be filled with the rgb channels pwm values.
        virtual void getLedValue(std::vector<int> &ledValuesMap) = 0;
        /// Sets the LED value to a desired value.
        /// @param redPwm Desired red channel pwm value.
        /// @param greenPwm Desired green channel pwm value.
        /// @param bluePwm Desired blue channel pwm value.
        virtual void setLedValue(int redPwm, int greenPwm, int bluePwm) = 0;
    };
}
#endif