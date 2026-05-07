cmake_minimum_required(VERSION 3.8)
project(trace_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ===================== for plugin =========================
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(TBB REQUIRED)

# msg generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Perception.msg"
  "msg/Detection.msg"
  "msg/CarInfo.msg"
  "msg/GuiControl.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# compile so
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(camera_view_plugin SHARED
  src/camera_view_plugin.cpp
)
target_link_libraries(camera_view_plugin
  ${GAZEBO_LIBRARIES}
  "${cpp_typesupport_target}"
  Qt5::Core
  Qt5::Widgets
  TBB::tbb
)
ament_target_dependencies(camera_view_plugin
  rclcpp
  std_msgs
)

install(TARGETS camera_view_plugin
  LIBRARY DESTINATION lib
)
# ===================== end for plugin =========================


# ====== C++ node ======
add_executable(ccd_node
  src/ccd_node.cpp
  src/ccd_processor.cpp
)
ament_target_dependencies(ccd_node rclcpp sensor_msgs OpenCV)
target_link_libraries(ccd_node "${cpp_typesupport_target}")

target_include_directories(ccd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(controller_node
  src/controller_node.cpp
  src/controller_core.cpp
)
ament_target_dependencies(controller_node rclcpp sensor_msgs std_msgs)
target_link_libraries(controller_node "${cpp_typesupport_target}")

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ccd_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)


# ====== Python node ======
# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    trace_car/ccd_node.py
    trace_car/controller_node.py
    trace_car/monitor_gui.py
    DESTINATION lib/${PROJECT_NAME}
)

# shared folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
