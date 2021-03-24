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

#include "ctrl_pkg/ctrl_state.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "deepracer_interfaces_pkg/srv/inference_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/load_model_srv.hpp"
#include "deepracer_interfaces_pkg/srv/model_optimize_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/servo_gpio_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/nav_throttle_srv.hpp"

namespace {
    // Name of relavent services.
    //! TODO remove when the topic and service names are refactor to a common header
    const char* MODEL_OPT_SRV = "/model_optimizer_pkg/model_optimizer_server";
    const char* MODEL_SRV = "/inference_pkg/load_model";
    const char* INFER_SRV = "/inference_pkg/inference_state";
    const char* SERVO_GPIO_SRV = "/servo_pkg/servo_gpio";
    const char* SET_SERVO_CAL_SRV = "/servo_pkg/set_calibration";
    const char* GET_SERVO_CAL_SRV = "/servo_pkg/get_calibration";
    const char* GET_LED_PWM_SRV = "/servo_pkg/get_led_state";
    const char* SET_LED_PWM_SRV = "/servo_pkg/set_led_state";
    const char* NAV_THROTTLE_SRV = "/deepracer_navigation_pkg/navigation_throttle";
    const char* NAV_ACTION_SPACE_SRV = "/deepracer_navigation_pkg/load_action_space";
    const char* SERVO_TOPIC = "servo_msg";
    const char* RAW_PWM_TOPIC = "raw_pwm";

    /// Helper method to wait for the service which the client subscribes to.
    /// @param client Reference to a service client.
    /// @param ctrlNode Reference to the CtrlNode object.
    void waitForService(rclcpp::ClientBase::SharedPtr client,
                        std::shared_ptr<rclcpp::Node> ctrlNode) {
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(ctrlNode->get_logger(), "%s node failed", client->get_service_name());
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "%s not available, waiting again...", client->get_service_name());
        }
    }

    /// Helper method that ensures the gpio is enabled.
    /// @param servoGPIOClient Reference to the servo gpio client
    void enableGPIO(rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr &servoGPIOClient){
        auto servoGPIORequest = std::make_shared<deepracer_interfaces_pkg::srv::ServoGPIOSrv::Request>();
        // 0 for enable and 1 for disable
        servoGPIORequest->enable = 0;
        auto future_result_servo_gpio = servoGPIOClient->async_send_request(servoGPIORequest);
    }

}

namespace SysCtrl {
    AutoDriveCtrl::AutoDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName)
      : isActive_(false)
    {
        ctrlNode = ctrlNodePtr;
        // Subscribe to the appropriate servo topic.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        auto servoMsgStrategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<deepracer_interfaces_pkg::msg::ServoCtrlMsg, 1>>();
        servoPub_ = ctrlNode->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(SERVO_TOPIC, qos);

        servoSub_ = ctrlNode->create_subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(subName,
                                                                                               qos,
                                                                                               std::bind(&AutoDriveCtrl::servoCB,
                                                                                                         this,
                                                                                                         std::placeholders::_1),
                                                                                               rclcpp::SubscriptionOptions(),
                                                                                               servoMsgStrategy);
        
        servoGPIOClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        servoGPIOClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>(SERVO_GPIO_SRV,
                                                                                                rmw_qos_profile_services_default,
                                                                                                servoGPIOClientCbGrp_);
        waitForService(servoGPIOClient_, ctrlNode);
        
        modelOptimizerClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        modelOptimizerClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::ModelOptimizeSrv>(MODEL_OPT_SRV,
                                                                                          rmw_qos_profile_services_default,
                                                                                          modelOptimizerClientCbGrp_);
        waitForService(modelOptimizerClient_, ctrlNode);
        
        loadModelClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        loadModelClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::LoadModelSrv>(MODEL_SRV,
                                                                                          rmw_qos_profile_services_default,
                                                                                          loadModelClientCbGrp_);
        waitForService(loadModelClient_, ctrlNode);

        loadActionSpaceClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        loadActionSpaceClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::LoadModelSrv>(NAV_ACTION_SPACE_SRV,
                                                                                          rmw_qos_profile_services_default,
                                                                                          loadActionSpaceClientCbGrp_);
        waitForService(loadActionSpaceClient_, ctrlNode);

        inferStateClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        inferStateClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::InferenceStateSrv>(INFER_SRV,
                                                                                          rmw_qos_profile_services_default,
                                                                                          inferStateClientCbGrp_);
        waitForService(inferStateClient_, ctrlNode);

        navigationThrottleClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        navigationThrottleClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::NavThrottleSrv>(NAV_THROTTLE_SRV,
                                                                                          rmw_qos_profile_services_default,
                                                                                          navigationThrottleClientCbGrp_);
        waitForService(navigationThrottleClient_, ctrlNode);
    }

    void AutoDriveCtrl::servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) {
        if(!isActive_) {
            return;
        }
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = msg->angle;
        servoMsg.throttle = msg->throttle;
        servoPub_->publish(servoMsg);  // Publish it along.
    }

    bool AutoDriveCtrl::loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors, 
                                     int trainingAlgorithm, int actionSpaceType, std::string imgFormat,
                                     int width, int height, int numChannels, 
                                     int lidarChannels, int platform, int task, int preProcess) {
        (void)task;
        RCLCPP_INFO(ctrlNode->get_logger(), "New load model request: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
        latestLoadModelRequestSeq_ = requestSeqNum;
        const std::lock_guard<std::mutex> lock(loadModelMutex_);
        try {
            modelLoadingStatus_ = "loading";                                      
            //! TODO handle ssd models, maybe as a different state and switch
            //! to enums when build is consilidated.
            int rlTask = 0; // Only load rl models

            RCLCPP_INFO(ctrlNode->get_logger(), "[0/5] Starting the load model service calls for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            // Stop the inference task
            if (!setInferState(rlTask, false)) {
                RCLCPP_ERROR(ctrlNode->get_logger(), "Failed to call the inference service");
                modelLoadingStatus_ = "error";                                      
                return false;
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "[1/5] Stopped inference task for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            if(latestLoadModelRequestSeq_ > requestSeqNum) {
                RCLCPP_WARN(ctrlNode->get_logger(), "Another load model request %d is waiting over current request %d..", latestLoadModelRequestSeq_, requestSeqNum);
                return false;
            }

            // Optimize the model.
            auto moSrvRequest = std::make_shared<deepracer_interfaces_pkg::srv::ModelOptimizeSrv::Request>();
            moSrvRequest->model_name = modelName.c_str();
            moSrvRequest->model_metadata_sensors = modelMetadataSensors;
            moSrvRequest->training_algorithm = trainingAlgorithm;
            moSrvRequest->img_format = imgFormat.c_str();
            moSrvRequest->width = width;
            moSrvRequest->height = height;
            moSrvRequest->num_channels = numChannels;
            moSrvRequest->platform = platform;
            moSrvRequest->lidar_channels = lidarChannels;
            
            auto future_result_mo = modelOptimizerClient_->async_send_request(moSrvRequest);
            auto moSrvResponse = std::make_shared<deepracer_interfaces_pkg::srv::ModelOptimizeSrv::Response>();
            future_result_mo.wait();
            moSrvResponse = future_result_mo.get();
            if(moSrvResponse->error != 0){
                RCLCPP_ERROR(ctrlNode->get_logger(), "Model optimizer failed.");
                modelLoadingStatus_ = "error";
                return false;
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "[2/5] Optimized the model for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            if(latestLoadModelRequestSeq_ > requestSeqNum) {
                RCLCPP_WARN(ctrlNode->get_logger(), "Another load model request %d is waiting over current request %d..", latestLoadModelRequestSeq_, requestSeqNum);
                return false;
            }
            // Load the model into the memory.
            auto modelSrvRequest = std::make_shared<deepracer_interfaces_pkg::srv::LoadModelSrv::Request>();
            modelSrvRequest->artifact_path = moSrvResponse->artifact_path;
            modelSrvRequest->task_type = rlTask;
            modelSrvRequest->pre_process_type = preProcess;
            modelSrvRequest->action_space_type = actionSpaceType;

            auto future_result_load_model = loadModelClient_->async_send_request(modelSrvRequest);
            future_result_load_model.wait();
            auto loadModelSrvResponse = future_result_load_model.get();
            if(loadModelSrvResponse->error != 0){
                RCLCPP_ERROR(ctrlNode->get_logger(), "Model loader failed.");
                modelLoadingStatus_ = "error";
                return false;
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "[3/5] Inference node updated for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            if(latestLoadModelRequestSeq_ > requestSeqNum) {
                RCLCPP_WARN(ctrlNode->get_logger(), "Another load model request %d is waiting over current request %d..", latestLoadModelRequestSeq_, requestSeqNum);
                return false;
            }

            auto future_result_load_action_space = loadActionSpaceClient_->async_send_request(modelSrvRequest);
            future_result_load_action_space.wait();
            auto loadActionSpaceResponse = future_result_load_action_space.get();
            if(loadActionSpaceResponse->error != 0){
                RCLCPP_ERROR(ctrlNode->get_logger(), "Action space load failed.");
                modelLoadingStatus_ = "error";
                return false;
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "[4/5] Action space in navigation node updated for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            if(latestLoadModelRequestSeq_ > requestSeqNum) {
                RCLCPP_WARN(ctrlNode->get_logger(), "Another load model request %d is waiting over current request %d..", latestLoadModelRequestSeq_, requestSeqNum);
                return false;
            }
            // Start inference if the model was running.
            if (!setInferState(rlTask, isActive_)) {
                RCLCPP_ERROR(ctrlNode->get_logger(), "Failed to reintialize inference");
                modelLoadingStatus_ = "error";
                return false;
            }
            RCLCPP_INFO(ctrlNode->get_logger(), "[5/5] Inference task set for req: %d; Latest request: %d", requestSeqNum, latestLoadModelRequestSeq_);
            modelLoadingStatus_ = "loaded";   
        }
        catch (const std::exception &ex) {
            RCLCPP_ERROR(ctrlNode->get_logger(), "Model failed to load: %s", ex.what());
            modelLoadingStatus_ = "error";
            return false;
        }        
        return true;
    }

     void AutoDriveCtrl::setStateActive(bool isActive) {
        isActive_ = isActive;
        //rl model for the task type
        if (!setInferState(0, isActive_)) {
            RCLCPP_ERROR(ctrlNode->get_logger(), "Failed to set inference state");
            return;
        }
        // Stop the servo when state is inactive
        if (!isActive_) {
            auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
            servoMsg.angle = 0.0;
            servoMsg.throttle = 0.0;
            servoPub_->publish(servoMsg);
            enableGPIO(servoGPIOClient_);
        }
     }

    std::string AutoDriveCtrl::getLoadModelStatus() {
        return modelLoadingStatus_;
    }

    void AutoDriveCtrl::getCalibration(int type, std::vector<int> &cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Calibration information not available in autonomous mode");
    }
    
    void AutoDriveCtrl::setCalibration(int type, const std::vector<int> & cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set calibrations in autonomous mode");
    }

    void AutoDriveCtrl::getLedValue(std::vector<int> &ledValuesMap) {
        (void)ledValuesMap;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get LED values in autonomous mode");
    }

    void AutoDriveCtrl::setLedValue(int redPwm, int greenPwm, int bluePwm) {
        (void)redPwm;
        (void)greenPwm;
        (void)bluePwm;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set LED values in autonomous mode");
    }

    void AutoDriveCtrl::setConstantThrottle(float throttle) {
        auto srvRequest = std::make_shared<deepracer_interfaces_pkg::srv::NavThrottleSrv::Request>();
        if (throttle >= 0.0 && throttle <= 1.0) {
            srvRequest->throttle = throttle;
            auto future_result = navigationThrottleClient_->async_send_request(srvRequest);
            future_result.wait();
            auto srvResponse = future_result.get();
            if(srvResponse->error != 0){
                RCLCPP_ERROR(ctrlNode->get_logger(), "Setting throttle failed.");
            }
        }
        else {
            RCLCPP_ERROR(ctrlNode->get_logger(), "Invalid navigation throttle: %f", throttle);
        }
    }

     bool AutoDriveCtrl::setInferState(int taskType, bool isActive) {
        auto infSrvRequest = std::make_shared<deepracer_interfaces_pkg::srv::InferenceStateSrv::Request>();
        infSrvRequest->start = isActive;
        infSrvRequest->task_type = taskType;
        auto future_result = inferStateClient_->async_send_request(infSrvRequest);
        return true;
     }

     ManualDriveCtrl::ManualDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName)
       : isActive_(false)
     {
        ctrlNode = ctrlNodePtr;
        // Subscribe to the appropriate servo topic.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        auto servoMsgStrategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<deepracer_interfaces_pkg::msg::ServoCtrlMsg, 1>>();
        servoPub_ = ctrlNode->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(SERVO_TOPIC, qos);

        servoSub_ = ctrlNode->create_subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(subName,
                                                                                               qos,
                                                                                               std::bind(&ManualDriveCtrl::servoCB,
                                                                                                         this,
                                                                                                         std::placeholders::_1),
                                                                                               rclcpp::SubscriptionOptions(),
                                                                                               servoMsgStrategy);

        servoGPIOClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        servoGPIOClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>(SERVO_GPIO_SRV,
                                                                                                rmw_qos_profile_services_default,
                                                                                                servoGPIOClientCbGrp_);
        waitForService(servoGPIOClient_, ctrlNode);
   }

    void ManualDriveCtrl::servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) {
        if(!isActive_ || !servoPub_) {
            return;
        }
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = msg->angle;
        servoMsg.throttle = msg->throttle;
        servoPub_->publish(std::move(servoMsg));  // Publish it along.
    }

    bool ManualDriveCtrl::loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors, 
                                     int trainingAlgorithm, int actionSpaceType, std::string imgFormat,
                                     int width, int height, int numChannels, 
                                     int lidarChannels, int platform, int task, int preProcess) {
        (void)requestSeqNum;
        (void)modelName;
        (void)modelMetadataSensors;
        (void)trainingAlgorithm;
        (void)actionSpaceType;
        (void)imgFormat;
        (void)width;
        (void)height;
        (void)numChannels;
        (void)lidarChannels;
        (void)platform;
        (void)task;
        (void)preProcess;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot load model in manual mode");
        return false;
    }

    void ManualDriveCtrl::setStateActive(bool isActive) {
        isActive_ = isActive;
        // Stop car and straighten wheels when starting or stopping this state
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = 0.0;
        servoMsg.throttle = 0.0;
        servoPub_->publish(servoMsg);
        enableGPIO(servoGPIOClient_);

    }

    std::string ManualDriveCtrl::getLoadModelStatus() {
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get load model status in manual mode.");
        return "error";
    }

    void ManualDriveCtrl::getCalibration(int type, std::vector<int> &cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Calibration information not available in manual mode");
    }

    void ManualDriveCtrl::setCalibration(int type, const std::vector<int> & cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set calibrations in manual mode");
    }

    void ManualDriveCtrl::getLedValue(std::vector<int> &ledValuesMap) {
        (void)ledValuesMap;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get LED values in manual mode");
    }

    void ManualDriveCtrl::setLedValue(int redPwm, int greenPwm, int bluePwm) {
        (void)redPwm;
        (void)greenPwm;
        (void)bluePwm;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set LED values in manual mode");
    }

    void ManualDriveCtrl::setConstantThrottle(float throttle) {
        (void)throttle;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set constant throttle in manual mode");
    }

    CalibrationCtrl::CalibrationCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName)
      : isActive_(false)
    {
        ctrlNode = ctrlNodePtr;
        // Subscribe to the appropriate servo topic.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        auto servoMsgStrategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<deepracer_interfaces_pkg::msg::ServoCtrlMsg, 1>>();
        calibrationPub_ = ctrlNode->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(RAW_PWM_TOPIC, qos);

        servoSub_ = ctrlNode->create_subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(subName,
                                                                                               qos,
                                                                                               std::bind(&CalibrationCtrl::servoCB,
                                                                                                         this,
                                                                                                         std::placeholders::_1),
                                                                                               rclcpp::SubscriptionOptions(),
                                                                                               servoMsgStrategy);

        servoGPIOClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        servoGPIOClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>(SERVO_GPIO_SRV,
                                                                                                rmw_qos_profile_services_default,
                                                                                                servoGPIOClientCbGrp_);
        waitForService(servoGPIOClient_, ctrlNode);
        
        servoGetCalClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                                
        servoGetCalClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::GetCalibrationSrv>(GET_SERVO_CAL_SRV,
                                                                                                       rmw_qos_profile_services_default,
                                                                                                       servoGetCalClientCbGrp_);
        waitForService(servoGetCalClient_, ctrlNode);
        
        servoSetCalClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                               
        servoSetCalClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::SetCalibrationSrv>(SET_SERVO_CAL_SRV,
                                                                                                       rmw_qos_profile_services_default,
                                                                                                       servoSetCalClientCbGrp_);
        waitForService(servoSetCalClient_, ctrlNode);

        servoGetLedClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                                 
        servoGetLedClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::GetLedCtrlSrv>(GET_LED_PWM_SRV,
                                                                                                   rmw_qos_profile_services_default,
                                                                                                   servoGetLedClientCbGrp_);
        waitForService(servoGetLedClient_, ctrlNode);
        
        servoSetLedClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                                   
        servoSetLedClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::SetLedCtrlSrv>(SET_LED_PWM_SRV,
                                                                                                   rmw_qos_profile_services_default,
                                                                                                   servoSetLedClientCbGrp_);
        waitForService(servoSetLedClient_, ctrlNode);
    }

    void CalibrationCtrl::servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) {
        if(!isActive_ || !calibrationPub_) {
            return;
        }
        // Publish it along.
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = msg->angle;
        servoMsg.throttle = msg->throttle;
        calibrationPub_->publish(servoMsg);
    }

    bool CalibrationCtrl::loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors, 
                                     int trainingAlgorithm, int actionSpaceType, std::string imgFormat,
                                     int width, int height, int numChannels, 
                                     int lidarChannels, int platform, int task, int preProcess) {
        (void)requestSeqNum;
        (void)modelName;
        (void)modelMetadataSensors;
        (void)trainingAlgorithm;
        (void)actionSpaceType;
        (void)imgFormat;
        (void)width;
        (void)height;
        (void)numChannels;
        (void)lidarChannels;
        (void)platform;
        (void)task;
        (void)preProcess;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot load model in calibration mode");
        return false;
    }

    void CalibrationCtrl::setStateActive(bool isActive) {
        isActive_ = isActive;
        // Make sure GPIO is enabled
        enableGPIO(servoGPIOClient_);
    }

    std::string CalibrationCtrl::getLoadModelStatus() {
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get load model status in manual mode.");
        return "error";
    }

    void CalibrationCtrl::getCalibration(int type, std::vector<int> &cal) {
        cal.clear();
        auto srvRequest = std::make_shared<deepracer_interfaces_pkg::srv::GetCalibrationSrv::Request>();
        srvRequest->cal_type = type;
        auto future_result = servoGetCalClient_->async_send_request(srvRequest);
        future_result.wait();
        auto srvResponse = std::make_shared<deepracer_interfaces_pkg::srv::GetCalibrationSrv::Response>();
        srvResponse = future_result.get();
        if(srvResponse->error != 0){
            RCLCPP_ERROR(ctrlNode->get_logger(), "Getting calibration failed.");
        }
        for (int i = 0; i < ServoCalType::total; ++i) {
            cal.push_back(0);
        }
        cal[ServoCalType::max] = srvResponse->max;
        cal[ServoCalType::mid] = srvResponse->mid;
        cal[ServoCalType::min] = srvResponse->min;
        cal[ServoCalType::polarity] = srvResponse->polarity;
    }

    void CalibrationCtrl::setCalibration(int type, const std::vector<int> &cal) {
        if (cal.size() != ServoCalType::total) {
            RCLCPP_ERROR(ctrlNode->get_logger(), "Invalid calibration vector");
            return;
        }
        if (cal[ServoCalType::max] == cal[ServoCalType::min]
            || cal[ServoCalType::max] == cal[ServoCalType::mid]
            || cal[ServoCalType::min] == cal[ServoCalType::mid]
            || std::max(cal[ServoCalType::max], cal[ServoCalType::min]) != cal[ServoCalType::max]
            || std::max(cal[ServoCalType::max], cal[ServoCalType::mid]) != cal[ServoCalType::max]
            || std::max(cal[ServoCalType::min], cal[ServoCalType::mid]) != cal[ServoCalType::mid]) {
            RCLCPP_ERROR(ctrlNode->get_logger(), "Invalid calibration values");
            return;
        }
        auto srvRequest = std::make_shared<deepracer_interfaces_pkg::srv::SetCalibrationSrv::Request>();
        srvRequest->cal_type = type;
        srvRequest->max = cal[ServoCalType::max];
        srvRequest->mid = cal[ServoCalType::mid];
        srvRequest->min = cal[ServoCalType::min];
        srvRequest->polarity = cal[ServoCalType::polarity];
        auto future_result = servoSetCalClient_->async_send_request(srvRequest);
        future_result.wait();

        auto srvResponse = future_result.get();
        if(srvResponse->error != 0){
            RCLCPP_ERROR(ctrlNode->get_logger(), "Setting calibration failed.");
        }
    }

    void CalibrationCtrl::setLedValue(int redPwm, int greenPwm, int bluePwm) {
        auto ledRequest = std::make_shared<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Request>();

        ledRequest->red = redPwm;
        ledRequest->green = greenPwm;
        ledRequest->blue = bluePwm;
        auto future_result = servoSetLedClient_->async_send_request(ledRequest);
        future_result.wait();
        auto ledResponse = future_result.get();
        if(ledResponse->error != 0){
            RCLCPP_ERROR(ctrlNode->get_logger(), "Setting LED failed.");
        }
    }

    void CalibrationCtrl::getLedValue(std::vector<int> &ledValuesMap) {
        auto ledRequest = std::make_shared<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Request>();
        auto future_result = servoGetLedClient_->async_send_request(ledRequest);
        // auto ledResponse = std::make_shared<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Response>();
        future_result.wait();
        auto ledResponse = future_result.get();
        
        ledValuesMap[ServoLedChannels::red] = ledResponse->red;
        ledValuesMap[ServoLedChannels::green] = ledResponse->green;
        ledValuesMap[ServoLedChannels::blue] = ledResponse->blue;
    }

    void CalibrationCtrl::setConstantThrottle(float throttle) {
        (void)throttle;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set constant throttle in calibration mode");
    }

    FTLDriveCtrl::FTLDriveCtrl(std::shared_ptr<rclcpp::Node> ctrlNodePtr, const std::string &subName)
       : isActive_(false)
     {
        ctrlNode = ctrlNodePtr;
        // Subscribe to the appropriate servo topic.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        auto servoMsgStrategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<deepracer_interfaces_pkg::msg::ServoCtrlMsg, 1>>();
        servoPub_ = ctrlNode->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(SERVO_TOPIC, qos);

        servoSub_ = ctrlNode->create_subscription<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(subName,
                                                                                               qos,
                                                                                               std::bind(&FTLDriveCtrl::servoCB,
                                                                                                         this,
                                                                                                         std::placeholders::_1),
                                                                                               rclcpp::SubscriptionOptions(),
                                                                                               servoMsgStrategy);

        servoGPIOClientCbGrp_ = ctrlNode->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);                                                                                          
        servoGPIOClient_ = ctrlNode->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>(SERVO_GPIO_SRV,
                                                                                                rmw_qos_profile_services_default,
                                                                                                servoGPIOClientCbGrp_);
        waitForService(servoGPIOClient_, ctrlNode);
   }

    void FTLDriveCtrl::servoCB(const deepracer_interfaces_pkg::msg::ServoCtrlMsg::SharedPtr msg) {
        if(!isActive_ || !servoPub_) {
            return;
        }
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = msg->angle;
        servoMsg.throttle = msg->throttle;
        servoPub_->publish(std::move(servoMsg));  // Publish it along.
    }

    bool FTLDriveCtrl::loadModelReq(int requestSeqNum, std::string modelName, std::vector<int> modelMetadataSensors, 
                                     int trainingAlgorithm, int actionSpaceType, std::string imgFormat,
                                     int width, int height, int numChannels, 
                                     int lidarChannels, int platform, int task, int preProcess) {
        (void)requestSeqNum;
        (void)modelName;
        (void)modelMetadataSensors;
        (void)trainingAlgorithm;
        (void)actionSpaceType;
        (void)imgFormat;
        (void)width;
        (void)height;
        (void)numChannels;
        (void)lidarChannels;
        (void)platform;
        (void)task;
        (void)preProcess;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot load model in follow the leader mode");
        return false;
    }

    void FTLDriveCtrl::setStateActive(bool isActive) {
        isActive_ = isActive;
        // Stop car and straighten wheels when starting or stopping this state
        auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        servoMsg.angle = 0.0;
        servoMsg.throttle = 0.0;
        servoPub_->publish(servoMsg);
        enableGPIO(servoGPIOClient_);

    }

    std::string FTLDriveCtrl::getLoadModelStatus() {
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get load model status in follow the leader mode.");
        return "error";
    }

    void FTLDriveCtrl::getCalibration(int type, std::vector<int> &cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Calibration information not available in follow the leader mode");
    }

    void FTLDriveCtrl::setCalibration(int type, const std::vector<int> & cal) {
        (void)type;
        (void)cal;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set calibrations in follow the leader mode");
    }

    void FTLDriveCtrl::getLedValue(std::vector<int> &ledValuesMap) {
        (void)ledValuesMap;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot get LED values in follow the leader mode");
    }

    void FTLDriveCtrl::setLedValue(int redPwm, int greenPwm, int bluePwm) {
        (void)redPwm;
        (void)greenPwm;
        (void)bluePwm;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set LED values in follow the leader mode");
    }

    void FTLDriveCtrl::setConstantThrottle(float throttle) {
        (void)throttle;
        RCLCPP_ERROR(ctrlNode->get_logger(), "Cannot set constant throttle in follow the leader mode");
    }
}
