cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transport)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs std_msgs)

add_message_files(
  FILES
  PointCloudTransportData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PointCloud.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} include/point_cloud_conversion include/point_cloud_transport)

link_directories(lib)


add_executable(talker_exec src/nodes/talker_node.cpp src/talker.cpp)
target_link_libraries(talker_exec ${catkin_LIBRARIES} libdracoenc.a)
add_dependencies(talker_exec point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)

add_executable(listener_exec src/listener.cpp src/nodes/listener_node.cpp)
target_link_libraries(listener_exec ${catkin_LIBRARIES} libdracodec.a)
add_dependencies(listener_exec point_cloud_transport_gencfg point_cloud_transport_generate_messages_cpp)


# test files -- start
add_executable(pct_pubtry test/pubtry.cpp src/PC2toDraco.cpp)
target_link_libraries(pct_pubtry ${catkin_LIBRARIES} libdraco.a)
add_executable(pct_subtry test/subtry.cpp src/PC2toDraco.cpp src/DracotoPC2.cpp)
target_link_libraries(pct_subtry ${catkin_LIBRARIES} libdraco.a)
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
