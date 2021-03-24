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
#include "deepracer_interfaces_pkg/srv/active_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/model_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_model_loading_status_srv.hpp"
#include "deepracer_interfaces_pkg/srv/enable_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/nav_throttle_srv.hpp"
#include "deepracer_interfaces_pkg/srv/video_state_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_calibration_srv.hpp"
#include "deepracer_interfaces_pkg/srv/set_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_ctrl_modes_srv.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

namespace SysCtrl {
    /// Topics and Services names
    #define AUTO_DRIVE_TOPIC "/deepracer_navigation_pkg/auto_drive"
    #define MANUAL_DRIVE_TOPIC "/webserver_pkg/manual_drive"
    #define CALIBRATION_DRIVE_TOPIC "/webserver_pkg/calibration_drive"
    #define FTL_DRIVE_TOPIC "/ftl_navigation_pkg/ftl_drive"

    /// Available states.
    enum CtrlState {
        manual,
        autonomous,
        calibration,
        followtheleader,
        numStates
    };

    const char* VEHICLE_STATE_SRV = "vehicle_state";
    const char* ENABLE_STATE_SRV = "enable_state";
    const char* MODEL_STATE_SRV = "model_state";
    const char* IS_MODEL_LOADING_SRV = "is_model_loading";
    const char* GET_CAL_SRV = "get_car_cal";
    const char* SET_CAL_SRV = "set_car_cal";
    const char* GET_LED_SRV = "get_car_led";
    const char* SET_LED_SRV = "set_car_led";
    const char* AUTONOMOUS_THROTTLE_SRV = "autonomous_throttle";
    const char* NAV_ACTION_SPACE_SRV = "/deepracer_navigation_pkg/load_action_space";
    const char* GET_CTRL_MODES_SRV = "get_ctrl_modes";
    const char* SERVO_TOPIC = "servo_msg";
    const char* RAW_PWM_TOPIC = "raw_pwm";
    class CtrlNodeMgr : public rclcpp::Node
    {
    /// Class that manages the states of the car.
    public:
        /// @param nodeName Reference to the string containing name of the node.
        CtrlNodeMgr(const std::string & nodeName)
        : Node(nodeName), initialized_(false)
        {
            RCLCPP_INFO(this->get_logger(), "%s started", nodeName.c_str());

            vehicleCtrlModesServiceCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            getVehicleCtrlModesService_ = this->create_service<deepracer_interfaces_pkg::srv::GetCtrlModesSrv>(GET_CTRL_MODES_SRV,
                                                                                                               std::bind(&SysCtrl::CtrlNodeMgr::getCtrlModesHdl,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2,
                                                                                                               std::placeholders::_3),
                                                                                                               ::rmw_qos_profile_default,
                                                                                                               vehicleCtrlModesServiceCbGrp_);

            vehicleModeServiceCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            setVehicleModeService_ = this->create_service<deepracer_interfaces_pkg::srv::ActiveStateSrv>(VEHICLE_STATE_SRV,
                                                                                                         std::bind(&SysCtrl::CtrlNodeMgr::stateHdl,
                                                                                                         this,
                                                                                                         std::placeholders::_1,
                                                                                                         std::placeholders::_2,
                                                                                                         std::placeholders::_3),
                                                                                                         ::rmw_qos_profile_default,
                                                                                                         vehicleModeServiceCbGrp_);
            activateVehicleServiceCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            activateVehicleService_ = this->create_service<deepracer_interfaces_pkg::srv::EnableStateSrv>(ENABLE_STATE_SRV,
                                                                                                          std::bind(&SysCtrl::CtrlNodeMgr::ctrlStateHdl,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2,
                                                                                                          std::placeholders::_3),
                                                                                                          ::rmw_qos_profile_default,
                                                                                                          activateVehicleServiceCbGrp_);


            loadModelCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            loadModelService_ = this->create_service<deepracer_interfaces_pkg::srv::ModelStateSrv>(MODEL_STATE_SRV,
                                                                                                    std::bind(&SysCtrl::CtrlNodeMgr::loadModelHdl,
                                                                                                    this,
                                                                                                    std::placeholders::_1,
                                                                                                    std::placeholders::_2,
                                                                                                    std::placeholders::_3),
                                                                                                    ::rmw_qos_profile_default,
                                                                                                    loadModelCbGrp_);
            isModelLoadingCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            isModelLoadingService_ = this->create_service<deepracer_interfaces_pkg::srv::GetModelLoadingStatusSrv>(IS_MODEL_LOADING_SRV,
                                                                                                                    std::bind(&SysCtrl::CtrlNodeMgr::isModelLoadingHdl,
                                                                                                                    this,
                                                                                                                    std::placeholders::_1,
                                                                                                                    std::placeholders::_2,
                                                                                                                    std::placeholders::_3),
                                                                                                                    ::rmw_qos_profile_default,
                                                                                                                    isModelLoadingCbGrp_);
            vehicleCalibrationCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
            getCarCalibrationService_ = this->create_service<deepracer_interfaces_pkg::srv::GetCalibrationSrv>(GET_CAL_SRV,
                                                                                                               std::bind(&SysCtrl::CtrlNodeMgr::getCarCalHdl,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2,
                                                                                                               std::placeholders::_3),
                                                                                                               ::rmw_qos_profile_default,
                                                                                                               vehicleCalibrationCbGrp_);
            setCarCalibrationService_ = this->create_service<deepracer_interfaces_pkg::srv::SetCalibrationSrv>(SET_CAL_SRV,
                                                                                                               std::bind(&SysCtrl::CtrlNodeMgr::setCarCalHdl,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2,
                                                                                                               std::placeholders::_3),
                                                                                                               ::rmw_qos_profile_default,
                                                                                                               vehicleCalibrationCbGrp_);

            vehicleLedCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
            getCarLedService_ = this->create_service<deepracer_interfaces_pkg::srv::GetLedCtrlSrv>(GET_LED_SRV,
                                                                                                   std::bind(&SysCtrl::CtrlNodeMgr::getCarLedHdl,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2,
                                                                                                   std::placeholders::_3),
                                                                                                   ::rmw_qos_profile_default,
                                                                                                   vehicleLedCbGrp_);
            setCarLedService_ = this->create_service<deepracer_interfaces_pkg::srv::SetLedCtrlSrv>(SET_LED_SRV,
                                                                                                    std::bind(&SysCtrl::CtrlNodeMgr::setCarLedHdl,
                                                                                                    this,
                                                                                                    std::placeholders::_1,
                                                                                                    std::placeholders::_2,
                                                                                                    std::placeholders::_3),
                                                                                                    ::rmw_qos_profile_default,
                                                                                                    vehicleLedCbGrp_);

            autonomousThrottleCbGrp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
            setAutonomousThrottleService_ = this->create_service<deepracer_interfaces_pkg::srv::NavThrottleSrv>(AUTONOMOUS_THROTTLE_SRV,
                                                                                                                std::bind(&SysCtrl::CtrlNodeMgr::autoThrottleHdl,
                                                                                                                this,
                                                                                                                std::placeholders::_1,
                                                                                                                std::placeholders::_2,
                                                                                                                std::placeholders::_3),
                                                                                                                ::rmw_qos_profile_default,
                                                                                                                autonomousThrottleCbGrp_);
            // TODO: Find better way to initialize requiring to exit constructor to use shared_from_this()
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SysCtrl::CtrlNodeMgr::init, this));
        }

        void init(){
            if(!initialized_){
                stateList_ = { {autonomous, std::make_shared<SysCtrl::AutoDriveCtrl>(this->shared_from_this(), AUTO_DRIVE_TOPIC)},
                           {manual, std::make_shared<SysCtrl::ManualDriveCtrl>(this->shared_from_this(), MANUAL_DRIVE_TOPIC)},
                           {calibration, std::make_shared<SysCtrl::CalibrationCtrl>(this->shared_from_this(), CALIBRATION_DRIVE_TOPIC)},
                           {followtheleader, std::make_shared<SysCtrl::FTLDriveCtrl>(this->shared_from_this(), FTL_DRIVE_TOPIC)} };
                activeState_ = stateList_.find(manual);
                activeState_->second->setStateActive(true);
                initialized_ = true;
                timer_->cancel();
                waitForServices();
                enableVideo();
            }
        }

        /// Request handler for getting the number of modes supported by ctrl_pkg
        void getCtrlModesHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                                  std::shared_ptr<deepracer_interfaces_pkg::srv::GetCtrlModesSrv::Request> req,
                                  std::shared_ptr<deepracer_interfaces_pkg::srv::GetCtrlModesSrv::Response> res) {
            (void)request_header; 
            (void)req;
            std::vector< int > list;
            for (int i = manual; i != numStates; i++) {
                list.push_back(i);
            }
            if(res) {
                res->modes = list;
                res->error = 0;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Result pointer not validated");
            }
        }

        void waitForServices(){
            videoClientCbGrp_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            videoClient_ = this->create_client<deepracer_interfaces_pkg::srv::VideoStateSrv>("/camera_pkg/media_state",
                                                                                             rmw_qos_profile_services_default,
                                                                                             videoClientCbGrp_);
            while (!videoClient_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Camera node failed");
                }
                RCLCPP_INFO(this->get_logger(), "Camera node not available, waiting again...");
            }
        }

        void enableVideo() {
            auto videoStateRequest = std::make_shared<deepracer_interfaces_pkg::srv::VideoStateSrv::Request>();
            videoStateRequest->activate_video = 1;
            auto future_result = videoClient_->async_send_request(videoStateRequest);
            future_result.wait();
            auto videoSrvResponse = future_result.get();
            if(videoSrvResponse->error == 0){
                RCLCPP_INFO(this->get_logger(), "Camera node enabled");
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Camera node could not to be enabled");
            }
        }

        /// Request handler for changing between states.
        void stateHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                      std::shared_ptr<deepracer_interfaces_pkg::srv::ActiveStateSrv::Request> req,
                      std::shared_ptr<deepracer_interfaces_pkg::srv::ActiveStateSrv::Response> res) {
            (void)request_header; 
            res->error = 1;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                return;
            }
            activeState_->second->setStateActive(false);

            auto itState = stateList_.find(req->state);

            if (itState != stateList_.end()) {
                activeState_ = itState;
                res->error = 0;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Invalid state");
            }
        }

        /// Request handler for checking if model is loading.
        void isModelLoadingHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::GetModelLoadingStatusSrv::Request> req,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::GetModelLoadingStatusSrv::Response> res) {
            (void)request_header; 
            (void)req; 
            res->model_loading_status = activeState_->second->getLoadModelStatus();
            res->error = 0;
        }

        /// Request handler for loading the model.
        void loadModelHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::ModelStateSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::ModelStateSrv::Response> res) {
            RCLCPP_INFO(this->get_logger(), "loadModelHdl %d", request_header->sequence_number);
            res->error = 1;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                return;
            }
            try {
                /// Call the load model function in a background thread.
                auto thrd = std::thread{std::bind(&CtrlStateBase::loadModelReq,
                                                    activeState_->second.get(),
                                                    request_header->sequence_number,
                                                    req->model_name,
                                                    req->model_metadata_sensors,
                                                    req->training_algorithm,
                                                    req->action_space_type,
                                                    req->img_format, 
                                                    req->width,
                                                    req->height,
                                                    req->num_channels,
                                                    req->lidar_channels, 
                                                    req->platform,
                                                    req->task_type,
                                                    req->pre_process_type)};
                    thrd.detach();
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "Model failed to load: %s", ex.what());
                return;
            }
            res->error = 0;
        }
        /// Request handler for when clients when to enable or disable the current state.
        void ctrlStateHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::EnableStateSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::EnableStateSrv::Response> res) {
            (void)request_header;
            res->error = 1;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                return;
            }
            activeState_->second->setStateActive(req->is_active);
            res->error = 0;
        }

        /// Request handler for when clients requests for current calibration.
        void getCarCalHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::GetCalibrationSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::GetCalibrationSrv::Response> res) {
            (void)request_header;
            std::vector<int> cal;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                return;
            }
            activeState_->second->getCalibration(req->cal_type, cal);
            if (cal.size() != ServoCalType::total) {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieve calibrations");
                return ;
            }
            res->max = cal[ServoCalType::max];
            res->mid = cal[ServoCalType::mid];
            res->min = cal[ServoCalType::min];
            res->polarity = cal[ServoCalType::polarity];
        }

        /// Request handler for when clients requests to set new calibration.
        void setCarCalHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::SetCalibrationSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::SetCalibrationSrv::Response> res) {
            (void)request_header;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                res->error = 1;
                return;
            }
            std::vector<int> cal(ServoCalType::total);
            cal[ServoCalType::max] = req->max;
            cal[ServoCalType::mid] = req->mid;
            cal[ServoCalType::min] = req->min;
            cal[ServoCalType::polarity] = req->polarity;
            res->error = 0;
            activeState_->second->setCalibration(req->cal_type, cal);
        }

        /// Request handler for when clients requests for current tail light LED RGB channel values.
        void getCarLedHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Response> res) {
            (void)request_header;
            (void)req;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                return;
            }
            std::vector<int> ledValuesMap(ServoLedChannels::numChannels, 0);
            activeState_->second->getLedValue(ledValuesMap);
            res->red = ledValuesMap[ServoLedChannels::red];
            res->green = ledValuesMap[ServoLedChannels::green];
            res->blue = ledValuesMap[ServoLedChannels::blue];
        }

        /// Request handler for when clients requests to set new tail light LED RGB channel values.
        void setCarLedHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Request> req,
                          std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Response> res) {
            (void)request_header;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                res->error = 1;
                return;
            }
            activeState_->second->setLedValue(req->red, req->green, req->blue);
            res->error = 0;
        }

        /// Request handler for when clients requests to set the throttle scale in autonomous mode.
        void autoThrottleHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                             std::shared_ptr<deepracer_interfaces_pkg::srv::NavThrottleSrv::Request> req,
                             std::shared_ptr<deepracer_interfaces_pkg::srv::NavThrottleSrv::Response> res) {
            (void)request_header;
            if (activeState_ == stateList_.end()) {
                RCLCPP_ERROR(this->get_logger(), "No active state");
                res->error = 1;
                return;
            }
            activeState_->second->setConstantThrottle(req->throttle);
            res->error = 0;
        }

    private:
        /// initialization flag
        bool initialized_;
        /// init timer
        rclcpp::TimerBase::SharedPtr timer_;
        /// Available states.
        std::unordered_map<int, std::shared_ptr<CtrlStateBase>> stateList_;
        /// Current state.
        std::unordered_map<int, std::shared_ptr<CtrlStateBase>>::const_iterator activeState_;

        /// ROS callback group object to be passed to the videoClient_.
        rclcpp::callback_group::CallbackGroup::SharedPtr videoClientCbGrp_;
        /// ROS service client to activate the camera node to start publishing images.
        rclcpp::Client<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr videoClient_;
        rclcpp::callback_group::CallbackGroup::SharedPtr vehicleCtrlModesServiceCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::GetCtrlModesSrv>::SharedPtr getVehicleCtrlModesService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr vehicleModeServiceCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::ActiveStateSrv>::SharedPtr setVehicleModeService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr activateVehicleServiceCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::EnableStateSrv>::SharedPtr activateVehicleService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr loadModelCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::ModelStateSrv>::SharedPtr loadModelService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr isModelLoadingCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::GetModelLoadingStatusSrv>::SharedPtr isModelLoadingService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr vehicleCalibrationCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::GetCalibrationSrv>::SharedPtr getCarCalibrationService_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::SetCalibrationSrv>::SharedPtr setCarCalibrationService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr vehicleLedCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::GetLedCtrlSrv>::SharedPtr getCarLedService_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::SetLedCtrlSrv>::SharedPtr setCarLedService_;
        rclcpp::callback_group::CallbackGroup::SharedPtr autonomousThrottleCbGrp_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::NavThrottleSrv>::SharedPtr setAutonomousThrottleService_;
    };
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SysCtrl::CtrlNodeMgr>("control_node");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
