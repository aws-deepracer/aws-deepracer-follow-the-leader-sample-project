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

#ifndef CTRL_STATE_HPP
#define CTRL_STATE_HPP

#include "ctrl_pkg/ctrl_state_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "deepracer_interfaces_pkg/srv/servo_gpio_srv.hpp"
#include "deepracer_interfaces_pkg/srv/model_optimize_srv.hpp"
#include "deepracer_interfaces_pkg/srv/load_model_srv.hpp"
#include "deepracer_interfaces_pkg/srv/inference_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/nav_throttle_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_led_ctrl_srv.hpp"

namespace SysCtrl {
    /// Enum for the calibration keys.
    enum ServoCalType {
        max,
        mid,
        min,
        polarity,
        total
    };

    /// Enum with the servo led channels.
    enum ServoLedChannels {
        red,
        green,
        blue,
        numChannels
    };

    class AutoDriveCtrl : public CtrlStateBase
    {
    /// Control state for autonomous driving.
    public:
        /// @param nodeHdl Reference to the node handler of the control node.
        /// @param subName Topic to subscribe to for servo messages.
        AutoDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName);
        virtual ~AutoDriveCtrl() = default;

        virtual void servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) override;
        virtual bool loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors,
                                  int trainingAlgorithm, int actionSpaceType, std::string imgFormat, int width, int height, int numChannels, 
                                  int lidarChannels, int platform, int task, int preProcess) override;
        virtual void setStateActive(bool isActive) override;
        virtual std::string getLoadModelStatus() override;
        virtual void getCalibration(int type, std::vector<int> &cal) override;
        virtual void setCalibration(int type, const std::vector<int> & cal) override;
        virtual void setConstantThrottle(float throttle) override;
        virtual void getLedValue(std::vector<int> &ledValuesMap) override;
        virtual void setLedValue(int redPwm, int greenPwm, int bluePwm) override;

    private:
        /// Control node object
        std::shared_ptr<rclcpp::Node> ctrlNode;
        /// Helper method that starts and stops inference.
        /// @param taskType Inference task to set as active inactive.
        /// @param isActive True if inference should be running.
        bool setInferState(int taskType, bool isActive);
        /// ROS subscriber object to the desired servo topic.
        rclcpp::Subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoSub_;
        /// ROS publisher object to the publish the servo messages that will handle the actuation.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoPub_;
        /// ROS callback group object to be passed to the servoGPIOClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGPIOClientCbGrp_;
        /// ROS client that sets the GPIO pin, this is required to be set when user
        /// starts servo.
        rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr servoGPIOClient_;
        /// ROS callback group object to be passed to the modelOptimizerClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr  modelOptimizerClientCbGrp_;
        /// ROS client to the model optimizer.
        rclcpp::Client<deepracer_interfaces_pkg::srv::ModelOptimizeSrv>::SharedPtr modelOptimizerClient_;
        /// ROS callback group object to be passed to the loadModelClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr loadModelClientCbGrp_;
        /// ROS client to the inference engine model loading server.
        rclcpp::Client<deepracer_interfaces_pkg::srv::LoadModelSrv>::SharedPtr loadModelClient_;
        /// ROS callback group object to be passed to the loadActionSpaceClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr loadActionSpaceClientCbGrp_;
        // ROS client to the load action space service.
        rclcpp::Client<deepracer_interfaces_pkg::srv::LoadModelSrv>::SharedPtr loadActionSpaceClient_;
        /// ROS callback group object to be passed to the inferStateClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr inferStateClientCbGrp_;
        /// ROS client to inference engine.
        rclcpp::Client<deepracer_interfaces_pkg::srv::InferenceStateSrv>::SharedPtr inferStateClient_;
        /// ROS callback group object to be passed to the navigationThrottleClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr navigationThrottleClientCbGrp_;
        /// ROS client to the navigation throttle control.
        rclcpp::Client<deepracer_interfaces_pkg::srv::NavThrottleSrv>::SharedPtr navigationThrottleClient_;
        /// State variable that tracks whether or not the state is active.
        bool isActive_;
        /// Status to indicate if the model loading operation is in progress.
        std::string modelLoadingStatus_ = "error";
        /// Latest model load request sequence number.
        int latestLoadModelRequestSeq_ = -1;
        // Mutex to lock model load service calls.
        std::mutex loadModelMutex_;
    };

    class ManualDriveCtrl : public CtrlStateBase
    {
    /// Control state for manual driving.
    public:
        /// @param nodeHdl Reference to the node handler of the control node.
        /// @param subName Topic to subscribe to for servo messages.
        ManualDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName);
        virtual ~ManualDriveCtrl() = default;

        virtual void servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) override;
        virtual bool loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors,
                                  int trainingAlgorithm, int actionSpaceType, std::string imgFormat, int width, int height, int numChannels, 
                                  int lidarChannels, int platform, int task, int preProcess) override;
        virtual void setStateActive(bool isActive) override;
        virtual std::string getLoadModelStatus() override;
        virtual void getCalibration(int type, std::vector<int> &cal) override;
        virtual void setCalibration(int type, const std::vector<int> & cal) override;
        virtual void setConstantThrottle(float throttle) override;
        virtual void getLedValue(std::vector<int> &ledValuesMap) override;
        virtual void setLedValue(int redPwm, int greenPwm, int bluePwm) override;

    private:
        /// Control node object
        std::shared_ptr<rclcpp::Node> ctrlNode;
        /// ROS subscriber object to the desired servo topic.
        rclcpp::Subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoSub_;
        /// ROS publisher object to the publish the servo messages that will handle the actuation.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoPub_;
        /// ROS callback group object to be passed to the servoGPIOClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGPIOClientCbGrp_;
        /// ROS client that sets the GPIO pin, this is required to be set when user
        /// starts servo.
        rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr servoGPIOClient_;
        /// State variable that tracks whether or not the state is active
        bool isActive_;
    };

    class CalibrationCtrl : public CtrlStateBase
    {
    public:
        /// @param nodeHdl Reference to the node handler of the control node.
        /// @param subName Topic to subscribe to for servo messages.
        CalibrationCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName);
        virtual ~CalibrationCtrl() = default;

        virtual void servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) override;
        virtual bool loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors,
                                  int trainingAlgorithm, int actionSpaceType, std::string imgFormat, int width, int height, int numChannels, 
                                  int lidarChannels, int platform, int task, int preProcess) override;
        virtual void setStateActive(bool isActive) override;
        virtual std::string getLoadModelStatus() override;
        virtual void getCalibration(int type, std::vector<int> &cal) override;
        virtual void setCalibration(int type, const std::vector<int> & cal) override;
        virtual void setConstantThrottle(float throttle) override;
        virtual void getLedValue(std::vector<int> &ledValuesMap) override;
        virtual void setLedValue(int redPwm, int greenPwm, int bluePwm) override;

    private:
        /// Control node object
        std::shared_ptr<rclcpp::Node> ctrlNode;
        /// ROS subscriber object to the desired servo topic.
        rclcpp::Subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoSub_;
        /// ROS client to the servo server that will handle the actuation, should be a raw pwm
        /// client for calibration.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr calibrationPub_;
        /// ROS callback group object to be passed to the servoGPIOClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGPIOClientCbGrp_;
        /// ROS client that sets the GPIO pin, this is required to be set when user
        /// starts servo.
        rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr servoGPIOClient_;
        /// ROS callback group object to be passed to the servoGetCalClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGetCalClientCbGrp_;
        /// ROS client for setting the calibration.
        rclcpp::Client<deepracer_interfaces_pkg::srv::GetCalibrationSrv>::SharedPtr servoGetCalClient_;
        /// ROS callback group object to be passed to the servoSetCalClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoSetCalClientCbGrp_;
        /// ROS client for getting the calibration.
        rclcpp::Client<deepracer_interfaces_pkg::srv::SetCalibrationSrv>::SharedPtr servoSetCalClient_;
        /// ROS callback group object to be passed to the servoGetLedClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGetLedClientCbGrp_;
        /// ROS client for getting the LED PWM values.
        rclcpp::Client<deepracer_interfaces_pkg::srv::GetLedCtrlSrv>::SharedPtr servoGetLedClient_;
        /// ROS callback group object to be passed to the servoSetLedClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoSetLedClientCbGrp_;
        /// ROS client for setting the LED PWM values.
        rclcpp::Client<deepracer_interfaces_pkg::srv::SetLedCtrlSrv>::SharedPtr servoSetLedClient_;
        /// State variable that tracks whether or not the state is active.
        bool isActive_;
    };

    class FTLDriveCtrl : public CtrlStateBase
    {
    /// Control state for follow the leader driving mode.
    public:
        /// @param nodeHdl Reference to the node handler of the control node.
        /// @param subName Topic to subscribe to for servo messages.
        FTLDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName);
        virtual ~FTLDriveCtrl() = default;

        virtual void servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) override;
        virtual bool loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors,
                                  int trainingAlgorithm, int actionSpaceType, std::string imgFormat, int width, int height, int numChannels, 
                                  int lidarChannels, int platform, int task, int preProcess) override;
        virtual void setStateActive(bool isActive) override;
        virtual std::string getLoadModelStatus() override;
        virtual void getCalibration(int type, std::vector<int> &cal) override;
        virtual void setCalibration(int type, const std::vector<int> & cal) override;
        virtual void setConstantThrottle(float throttle) override;
        virtual void getLedValue(std::vector<int> &ledValuesMap) override;
        virtual void setLedValue(int redPwm, int greenPwm, int bluePwm) override;

    private:
        /// Control node object
        std::shared_ptr<rclcpp::Node> ctrlNode;
        /// ROS subscriber object to the desired servo topic.
        rclcpp::Subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoSub_;
        /// ROS publisher object to the publish the servo messages that will handle the actuation.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servoPub_;
        /// ROS callback group object to be passed to the servoGPIOClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr servoGPIOClientCbGrp_;
        /// ROS client that sets the GPIO pin, this is required to be set when user
        /// starts servo.
        rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr servoGPIOClient_;
        /// State variable that tracks whether or not the state is active
        bool isActive_;
    };
}
#endif