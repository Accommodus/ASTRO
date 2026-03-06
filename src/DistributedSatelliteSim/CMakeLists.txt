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

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------- Env Node library (testable) ----------
add_library(env_node_lib SHARED
  src/env_node.cpp
)
target_include_directories(env_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(env_node_lib
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  "${typesupport_target}"
)

# ---------- Env Node executable ----------
add_executable(env_node src/env_node.cpp)
target_link_libraries(env_node env_node_lib)

install(TARGETS env_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS env_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_env_node test/test_env_node.cpp)
  target_link_libraries(test_env_node env_node_lib)
endif()

ament_package()
