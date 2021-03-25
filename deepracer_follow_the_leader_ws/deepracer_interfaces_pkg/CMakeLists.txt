cmake_minimum_required(VERSION 3.5)
project(deepracer_interfaces_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/CameraMsg.msg"
  "msg/InferResults.msg"
  "msg/InferResultsArray.msg"
  "msg/EvoSensorMsg.msg"
  "msg/ServoCtrlMsg.msg"
  "msg/NetworkConnectionStatus.msg"
  "msg/SoftwareUpdatePctMsg.msg"
  "msg/USBFileSystemNotificationMsg.msg"
  "msg/DetectionDeltaMsg.msg"
)
set(srv_files
  "srv/ActiveStateSrv.srv"
  "srv/BatteryLevelSrv.srv"
  "srv/BeginSoftwareUpdateSrv.srv"
  "srv/ConsoleModelActionSrv.srv"
  "srv/EnableStateSrv.srv"
  "srv/GetCalibrationSrv.srv"
  "srv/GetDeviceInfoSrv.srv"
  "srv/GetLedCtrlSrv.srv"
  "srv/GetModelLoadingStatusSrv.srv"
  "srv/InferenceStateSrv.srv"
  "srv/LidarConfigSrv.srv"
  "srv/LoadModelSrv.srv"
  "srv/ModelOptimizeSrv.srv"
  "srv/ModelStateSrv.srv"
  "srv/NavThrottleSrv.srv"
  "srv/OTGLinkStateSrv.srv"
  "srv/SensorStatusCheckSrv.srv"
  "srv/SetCalibrationSrv.srv"
  "srv/ServoGPIOSrv.srv"
  "srv/SetLedCtrlSrv.srv"
  "srv/SetMaxSpeedSrv.srv"
  "srv/SetStatusLedBlinkSrv.srv"
  "srv/SetStatusLedSolidSrv.srv"
  "srv/SoftwareUpdateCheckSrv.srv"
  "srv/SoftwareUpdateStateSrv.srv"
  "srv/USBFileSystemSubscribeSrv.srv"
  "srv/USBMountPointManagerSrv.srv"
  "srv/VerifyModelReadySrv.srv"
  "srv/VideoStateSrv.srv"
  "srv/GetCtrlModesSrv.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
${msg_files}
${srv_files}
DEPENDENCIES builtin_interfaces std_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
