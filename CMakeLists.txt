cmake_minimum_required(VERSION 3.10.2)

project(point_cloud_transport)

find_package(ament_cmake_ros REQUIRED)

find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Build libpoint_cloud_transport
add_library(${PROJECT_NAME}
  # src/point_cloud_codec.cpp # TODO: This is needed for exposing the encode/decode functions without spinning an node
  src/point_cloud_common.cpp
  src/point_cloud_transport.cpp
  src/publisher.cpp
  src/publisher_plugin.cpp
  src/single_subscriber_publisher.cpp
  src/subscriber.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
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

# Build republish (TODO: there was some nodelet stuff happening here)
# add_executable(republish src/republish.cpp)
# target_link_libraries(republish
#   ${PROJECT_NAME}
#   pluginlib::pluginlib)

# Build list_transports (TODO: there was some nodelet stuff happening here)
add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports
  ${PROJECT_NAME}
  pluginlib::pluginlib)

# Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} default_plugins.xml)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# Install executables
install(
  TARGETS list_transports #republish
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(message_filters rclcpp sensor_msgs pluginlib)

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
