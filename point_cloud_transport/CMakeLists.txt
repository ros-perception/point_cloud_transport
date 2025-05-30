cmake_minimum_required(VERSION 3.10.2)

project(point_cloud_transport)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build libpoint_cloud_transport
add_library(${PROJECT_NAME}
  src/point_cloud_codec.cpp
  src/point_cloud_common.cpp
  src/point_cloud_transport.cpp
  src/publisher.cpp
  src/publisher_plugin.cpp
  src/raw_subscriber.cpp
  src/single_subscriber_publisher.cpp
  src/subscriber_filter.cpp
  src/subscriber.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  message_filters::message_filters
  pluginlib::pluginlib
  rclcpp::rclcpp
  rcpputils::rcpputils
  rmw::rmw
  ${sensor_msgs_TARGETS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "POINT_CLOUD_TRANSPORT_BUILDING_DLL")

# Build libpoint_cloud_transport_plugins
add_library(${PROJECT_NAME}_plugins
  src/manifest.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_plugins ALIAS ${PROJECT_NAME}_plugins)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME}
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
)

add_library(pc_republish_node SHARED src/republish.cpp)
target_link_libraries(pc_republish_node PRIVATE
  ${PROJECT_NAME}
  pluginlib::pluginlib
  rclcpp_components::component
  rclcpp::rclcpp
)
target_compile_definitions(pc_republish_node PRIVATE "POINT_CLOUD_TRANSPORT_BUILDING_DLL")
target_include_directories(pc_republish_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
rclcpp_components_register_nodes(pc_republish_node "point_cloud_transport::Republisher")

rclcpp_components_register_node(pc_republish_node
  PLUGIN "point_cloud_transport::Republisher"
  EXECUTABLE republish
)

# Build list_transports
add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports PRIVATE
  ${PROJECT_NAME}
  pluginlib::pluginlib)

# Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} default_plugins.xml)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins pc_republish_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# Install executables
install(
  TARGETS
    list_transports
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(message_filters rclcpp sensor_msgs pluginlib rcpputils rmw)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}-point_cloud_common test/test_point_cloud_common.cpp)
  if(TARGET ${PROJECT_NAME}-point_cloud_common)
    target_link_libraries(${PROJECT_NAME}-point_cloud_common ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-publisher test/test_publisher.cpp)
  if(TARGET ${PROJECT_NAME}-publisher)
    target_link_libraries(${PROJECT_NAME}-publisher ${PROJECT_NAME} rclcpp::rclcpp)
  endif()

  ament_add_gtest(${PROJECT_NAME}-codec test/test_point_cloud_codec.cpp)
  if(TARGET ${PROJECT_NAME}-codec)
    target_link_libraries(${PROJECT_NAME}-codec ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-subscriber)
    target_link_libraries(${PROJECT_NAME}-subscriber ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-message_passing test/test_message_passing.cpp)
  if(TARGET ${PROJECT_NAME}-message_passing)
    target_link_libraries(${PROJECT_NAME}-message_passing ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-qos_override test/test_qos_override.cpp)
  if(TARGET ${PROJECT_NAME}-qos_override)
    target_link_libraries(${PROJECT_NAME}-qos_override ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-remapping test/test_remapping.cpp)
  if(TARGET ${PROJECT_NAME}-remapping)
    target_link_libraries(${PROJECT_NAME}-remapping ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-message-filter test/test_message_filter.cpp)
  if(TARGET ${PROJECT_NAME}-message-filter)
    target_link_libraries(${PROJECT_NAME}-message-filter
      ${PROJECT_NAME} message_filters::message_filters rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-single_subscriber_publisher test/test_single_subscriber_publisher.cpp)
  if(TARGET ${PROJECT_NAME}-single_subscriber_publisher)
    target_link_libraries(${PROJECT_NAME}-single_subscriber_publisher ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()
endif()

ament_package()