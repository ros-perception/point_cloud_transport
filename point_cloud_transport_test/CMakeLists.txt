cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transport_test)

find_package(catkin REQUIRED COMPONENTS point_cloud_transport sensor_msgs rosbag)

catkin_package(CATKIN_DEPENDS point_cloud_transport sensor_msgs rosbag)

include_directories(include ${catkin_INCLUDE_DIRS})

# raw publisher
add_executable(raw_publisher_test src/raw_publisher_try.cpp)
add_dependencies(raw_publisher_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(raw_publisher_test ${catkin_LIBRARIES})


# raw subscriber
add_executable(raw_subscriber_test src/raw_subscriber_try.cpp)
add_dependencies(raw_subscriber_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(raw_subscriber_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS raw_publisher_test raw_subscriber_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

