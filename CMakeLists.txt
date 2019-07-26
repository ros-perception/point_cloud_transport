cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transport)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        message_filters
        message_generation
        pluginlib
        rosconsole
        roscpp
        roslib
        rosgraph_msgs
        std_msgs
        sensor_msgs)

find_package(Draco REQUIRED)
message("${draco_INCLUDE_DIR}")
message("${draco_LIBRARY_DIR}")

find_package(Boost REQUIRED)

add_message_files(
  FILES
  InfoPointCloud2.msg
  CompressedPointCloud2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PointCloud.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure message_runtime message_filters pluginlib rosconsole roscpp roslib rosgraph_msgs std_msgs sensor_msgs
  DEPENDS draco
)

include_directories(include include/point_cloud_conversion include/point_cloud_transport ${catkin_INCLUDE_DIRS} ${draco_INCLUDE_DIR}/..)
link_directories(${draco_LIBRARY_DIR})

# Build libpoint_cloud_transport
add_library(${PROJECT_NAME}
        src/point_cloud_transport.cpp
        src/publisher.cpp
        src/single_subscriber_publisher.cpp
        src/subscriber.cpp
        )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

# Build libpoint_cloud_transport_plugins
add_library(${PROJECT_NAME}_plugins src/manifest.cpp src/raw_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

add_executable(republish src/republish.cpp)
target_link_libraries(republish ${PROJECT_NAME})

add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(TARGETS list_transports republish
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES default_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

add_executable(talker_exec src/nodes/talker_node.cpp src/talker.cpp src/PC2toDraco.cpp src/conversion_utilities.cpp)
target_link_libraries(talker_exec ${catkin_LIBRARIES} dracoenc)
add_dependencies(talker_exec point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)


add_executable(listener_exec src/listener.cpp src/nodes/listener_node.cpp src/DracotoPC2.cpp src/conversion_utilities.cpp)
target_link_libraries(listener_exec ${catkin_LIBRARIES} dracodec)
add_dependencies(listener_exec point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)


# test files -- start
add_executable(pct_pubtry test/pubtry.cpp src/PC2toDraco.cpp src/conversion_utilities.cpp)
target_link_libraries(pct_pubtry ${catkin_LIBRARIES} draco)
add_dependencies(pct_pubtry point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)


add_executable(pct_subtry test/subtry.cpp src/PC2toDraco.cpp src/DracotoPC2.cpp test/Repubtry.cpp src/conversion_utilities.cpp)
target_link_libraries(pct_subtry ${catkin_LIBRARIES} draco)
add_dependencies(pct_subtry point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)
# test files -- end


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS listener_exec talker_exec
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
