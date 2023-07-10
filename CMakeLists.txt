cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)

project(point_cloud_transport)

find_package(ament_cmake_ros REQUIRED)

find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

include(FetchContent)
fetchcontent_declare(
  expected
  GIT_REPOSITORY https://github.com/TartanLlama/expected
  GIT_TAG        v1.1.0
)

fetchcontent_makeavailable(expected)

# Build libpoint_cloud_transport
add_library(${PROJECT_NAME}
  # src/point_cloud_codec.cpp # TODO: This is needed for exposing the encode/decode functions without spinning an node
  src/point_cloud_common.cpp
  src/point_cloud_transport.cpp
  src/publisher.cpp
  src/publisher_plugin.cpp
  src/raw_subscriber.cpp
  src/single_subscriber_publisher.cpp
  src/subscriber.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${expected_SOURCE_DIR}/include>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  message_filters::message_filters
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE
  pluginlib::pluginlib)

# Build libpoint_cloud_transport_plugins
add_library(${PROJECT_NAME}_plugins
  src/manifest.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_plugins ALIAS ${PROJECT_NAME}_plugins)

target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME}
  pluginlib::pluginlib)

add_library(republish_node SHARED src/republish.cpp)
target_link_libraries(republish_node
  ${PROJECT_NAME}
)
ament_target_dependencies(republish_node
  pluginlib
  rclcpp_components
  rclcpp
)
rclcpp_components_register_nodes(republish_node "point_cloud_transport::Republisher")

add_executable(republish
  src/republish_program.cpp
  src/utilities/utilities.cpp
)
target_link_libraries(republish
  ${PROJECT_NAME}
  republish_node
)
ament_target_dependencies(republish
  pluginlib
  rclcpp_components
  rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# Build list_transports
add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports
  ${PROJECT_NAME}
  pluginlib::pluginlib)

# Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} default_plugins.xml)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins republish_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# Install executables
install(
  TARGETS list_transports republish
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(message_filters rclcpp sensor_msgs pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE doc/conf.py)
  ament_lint_auto_find_test_dependencies()
endif()

# TODO: Fix testing
# if (CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)

#   # catkin_lint - checks validity of package.xml and CMakeLists.txt
#   # ROS buildfarm calls this without any environment and with empty rosdep cache,
#   # so we have problems reading the list of packages from env
#   # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
#   if(DEFINED ENV{ROS_HOME})
#     #catkin_lint: ignore_once env_var
#     set(ROS_HOME "$ENV{ROS_HOME}")
#   else()
#     #catkin_lint: ignore_once env_var
#     set(ROS_HOME "$ENV{HOME}/.ros")
#   endif()
#   #catkin_lint: ignore_once env_var
#   if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
#     roslint_custom(catkin_lint "-W2" .)
#   endif()

#   # Roslint C++ - checks formatting and some other rules for C++ files

#   file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
#   file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
#   #file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

#   set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
#     -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
#     -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
#   roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

#   # Roslint Python

#   # Run roslint on Python sources
#   file(GLOB_RECURSE python_files src/*.py)
#   roslint_python("${python_files}")

#   roslint_add_test()
# endif()

ament_package()
