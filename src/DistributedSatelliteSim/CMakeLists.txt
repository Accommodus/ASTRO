cmake_minimum_required(VERSION 3.8)
project(distributed_satellite_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------- Generate service interfaces ----------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ActuationCmd.srv"
)

# ---------- Env Node ----------
add_executable(env_node src/env_node.cpp)
target_include_directories(env_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(env_node
  rclcpp
  std_msgs
)

# ---------- GNC Node ----------
add_executable(gnc_node src/gnc_node.cpp)
target_include_directories(gnc_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(gnc_node
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(env_node "${typesupport_target}")
target_link_libraries(gnc_node "${typesupport_target}")

install(TARGETS env_node gnc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
