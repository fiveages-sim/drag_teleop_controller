cmake_minimum_required(VERSION 3.16)
project(drag_teleop_controller)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ruckig REQUIRED)
# pinocchio 4.x 强制 C++17（见 pinocchioConfig.cmake CHECK_MINIMAL_CXX_STANDARD）
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# 自定义服务接口（TeleopMode / TeleopFeedback）。
# rosidl_generate_interfaces 会创建同名 target（drag_teleop_controller），
# 因此插件库 target 命名为 ${PROJECT_NAME}_lib，通过 OUTPUT_NAME 保持
# 输出库文件名 libdrag_teleop_controller.so（pluginlib xml 按此名加载）。
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TeleopMode.srv"
  "srv/TeleopFeedback.srv"
)

# rosidl target 是 UTILITY 类型不能直接链接，需取 typesupport target
rosidl_get_typesupport_target(rosidl_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_lib SHARED
  src/drag_teleop_controller.cpp
  src/joint_mapper.cpp
  src/dynamics.cpp
  src/control_calculator.cpp
  src/ocs2_publisher.cpp
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# pinocchio 不是 ament 包：直接用其 CMake target 链接（include 路径随之传播）。
# 注意：这里必须用 plain 签名，与下方 ament_target_dependencies 的 plain 调用保持一致。
target_link_libraries(${PROJECT_NAME}_lib
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  ${rosidl_typesupport_target}  # rosidl 接口 typesupport
)

ament_target_dependencies(${PROJECT_NAME}_lib
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ruckig
  urdf
)

pluginlib_export_plugin_description_file(controller_interface drag_teleop_controller.xml)

install(
  TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES drag_teleop_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ruckig
  pinocchio
  urdf
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
