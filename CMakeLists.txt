cmake_minimum_required(VERSION 3.8)
project(nissan_can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

include_directories(include)

add_executable(nissan_vehicle_info src/nissanVehicleInfo.cpp src/nissanCanDefinitions.cpp)
ament_target_dependencies(nissan_vehicle_info rclcpp std_msgs can_msgs)

add_executable(nissan_vehicle_control src/nissanVehicleControl.cpp src/nissanCanDefinitions.cpp)
ament_target_dependencies(nissan_vehicle_control rclcpp std_msgs can_msgs autoware_control_msgs)

install(TARGETS
  nissan_vehicle_info
  nissan_vehicle_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
