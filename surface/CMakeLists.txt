cmake_minimum_required(VERSION 3.8)
project(surface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED  NO_MODULE)
find_package(message_type REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

add_library(oobstacle_processing src/oobstacle_processing.cpp)
target_include_directories(oobstacle_processing PRIVATE include)

add_library(mathtool src/mathtool.cpp)
target_include_directories(mathtool PRIVATE include)

add_library(controller src/controller.cpp)
target_include_directories(controller PRIVATE include)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



# Add executables
#add_executable(subs src/controller.cpp)
#ament_target_dependencies(subs rclcpp std_msgs nav_msgs)

add_executable(pid src/pid.cpp)
ament_target_dependencies(pid rclcpp std_msgs nav_msgs geometry_msgs)  # Fixed

add_executable(pose_control src/pose_control.cpp)
ament_target_dependencies(pose_control rclcpp std_msgs nav_msgs geometry_msgs tf2)
target_include_directories(pose_control PRIVATE include)


add_executable(new_control src/new_control.cpp)
ament_target_dependencies(new_control rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3) 
target_link_libraries(new_control Eigen3::Eigen)

add_executable(leader_follower src/leader_follower.cpp)
ament_target_dependencies(leader_follower rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3) 
target_link_libraries(leader_follower Eigen3::Eigen)

add_executable(traj_control src/traj_control.cpp)
ament_target_dependencies(traj_control rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3) 
target_link_libraries(traj_control Eigen3::Eigen)

add_executable(task_publisher src/task_publisher.cpp)
ament_target_dependencies(task_publisher rclcpp std_msgs message_type)

add_executable(vehicle_decide_control src/vehicle_decide_control.cpp)
ament_target_dependencies(vehicle_decide_control rclcpp std_msgs)

add_executable(roam src/roam.cpp)
ament_target_dependencies(roam rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3)
target_link_libraries(roam Eigen3::Eigen oobstacle_processing)


add_executable(roam_linear src/soph_roam.cpp  src/controller.cpp
    src/oobstacle_processing.cpp)
ament_target_dependencies(roam_linear rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3)
target_link_libraries(roam_linear Eigen3::Eigen oobstacle_processing mathtool controller) 


add_executable(roam_study src/under_study_roam.cpp  src/controller.cpp
    src/oobstacle_processing.cpp)
ament_target_dependencies(roam_study rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3)
target_link_libraries(roam_study Eigen3::Eigen oobstacle_processing mathtool controller) 

add_executable(multi_dmm src/multi_dmm.cpp  src/controller.cpp
    src/oobstacle_processing.cpp)
ament_target_dependencies(multi_dmm rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3)
target_link_libraries(multi_dmm Eigen3::Eigen oobstacle_processing mathtool controller) 



add_executable(concave src/concave.cpp  src/controller.cpp
    src/oobstacle_processing.cpp)
ament_target_dependencies(concave rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3)
target_link_libraries(concave Eigen3::Eigen oobstacle_processing mathtool controller) 

add_executable(multi_agent_1 src/multi_agent_1.cpp  src/controller.cpp
    src/oobstacle_processing.cpp
    )
ament_target_dependencies(multi_agent_1 rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3 message_type)
target_link_libraries(multi_agent_1 Eigen3::Eigen oobstacle_processing mathtool controller ) 

add_executable(task_allocated_multi_agent src/task_allocated_multi_agent.cpp  src/controller.cpp
    src/oobstacle_processing.cpp
    )
ament_target_dependencies(task_allocated_multi_agent rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3 message_type)
target_link_libraries(task_allocated_multi_agent Eigen3::Eigen oobstacle_processing mathtool controller ) 

add_executable(decision_making src/decision_making.cpp )
ament_target_dependencies(decision_making rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs Eigen3 message_type)
target_link_libraries(decision_making ) 

add_executable(aggregator_node src/agrregator_node.cpp)
ament_target_dependencies(aggregator_node rclcpp message_type)
target_link_libraries(aggregator_node)



# Install launch, models, and worlds directories
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS
  
  pid  # Added `pid` in the same install block
  pose_control
  new_control
  leader_follower
  traj_control
  task_publisher
  vehicle_decide_control
  roam
  roam_linear
  concave
  multi_agent_1
  decision_making
  aggregator_node
  task_allocated_multi_agent
  roam_study
  multi_dmm
  DESTINATION lib/${PROJECT_NAME})



ament_package()

