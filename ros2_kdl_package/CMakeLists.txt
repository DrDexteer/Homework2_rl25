cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_kdl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)  
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)




add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp)

target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  rclcpp_action
  ros2_kdl_interfaces
  std_msgs
  tf2_ros
  tf2_kdl 
  tf2_geometry_msgs
  ros_gz_interfaces
  geometry_msgs)

install(TARGETS ros2_kdl_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(ros2_kdl_node_example
  src/ros2_kdl_node_example.cpp)

target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node_example
  orocos_kdl
  rclcpp)

install(TARGETS ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME})

# -------------------- ACTION CLIENT (COMPONENT) --------------------
# libreria del component (nome diverso dall'eseguibile!)
add_library(execute_linear_traj_client_lib SHARED
  src/execute_linear_traj_client.cpp)

target_include_directories(execute_linear_traj_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(execute_linear_traj_client_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  ros2_kdl_interfaces)

# registra il component e crea l'eseguibile "execute_linear_traj_client"
rclcpp_components_register_node(execute_linear_traj_client_lib
  PLUGIN "ros2_kdl_pkg::ExecuteLinearTrajClient"
  EXECUTABLE execute_linear_traj_client)

# installa libreria + eseguibile
install(TARGETS
  execute_linear_traj_client_lib
  execute_linear_traj_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# -------------------- SHARE --------------------
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# -------------------- TESTING --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

