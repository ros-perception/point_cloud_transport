cmake_minimum_required(VERSION 3.10.2)
project(point_cloud_transport)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  cras_topic_tools
  dynamic_reconfigure
  message_generation
  pluginlib
  roscpp
  sensor_msgs
  topic_tools
)

find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(cfg/NoConfig.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} raw_${PROJECT_NAME}
  CATKIN_DEPENDS cras_cpp_common cras_topic_tools dynamic_reconfigure message_filters message_runtime roscpp sensor_msgs topic_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Build libpoint_cloud_transport
add_library(${PROJECT_NAME}
  src/point_cloud_codec.cpp
  src/point_cloud_transport.cpp
  src/publisher.cpp
  src/publisher_plugin.cpp
  src/single_subscriber_publisher.cpp
  src/subscriber.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# The library is build twice. Once with symbols exported for direct use, and once with symbols hidden for use via pluginlib.

# Build libraw_point_cloud_transport
add_library(raw_${PROJECT_NAME} src/raw_publisher.cpp src/raw_subscriber.cpp)
target_link_libraries(raw_${PROJECT_NAME} ${PROJECT_NAME})
add_dependencies(raw_${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Build libpoint_cloud_transport_plugins
add_library(${PROJECT_NAME}_plugins src/manifest.cpp src/raw_publisher.cpp src/raw_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS})
class_loader_hide_library_symbols(${PROJECT_NAME}_plugins)

add_executable(republish src/republish.cpp)
target_link_libraries(republish ${PROJECT_NAME})

add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins raw_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS list_transports republish
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES default_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()
  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
  endif()

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
  #file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

  roslint_add_test()
endif()
