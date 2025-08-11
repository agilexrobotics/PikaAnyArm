cmake_minimum_required(VERSION 3.5)
project(pika_remote_piper)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
if(NOT TARGET JsonCpp::JsonCpp)
    find_package(jsoncpp REQUIRED QUIET)
endif()
# find_package(Qt5SerialPort REQUIRED)
# find_package(serial REQUIRED)
# set(OpenCV_STATIC ON)
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
find_package(data_msgs REQUIRED)

include_directories(
  include
)

# add_executable(serial_gripper_imu src/serial_gripper_imu.cpp)
# ament_target_dependencies(serial_gripper_imu rclcpp rclpy std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs serial data_msgs)
# # rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(serial_gripper_imu 
#   "${cpp_typesupport_target}"  
#   libjsoncpp.so
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/piper_FK.py
  scripts/piper_IK.py
  scripts/teleop_piper_publish.py
  scripts/transformations.py
  scripts/arm_FIK.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)

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

ament_package()

