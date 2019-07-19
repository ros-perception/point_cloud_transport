# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "point_cloud_transport: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipoint_cloud_transport:/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(point_cloud_transport_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_custom_target(_point_cloud_transport_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "point_cloud_transport" "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(point_cloud_transport
  "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_cloud_transport
)

### Generating Services

### Generating Module File
_generate_module_cpp(point_cloud_transport
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_cloud_transport
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(point_cloud_transport_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(point_cloud_transport_generate_messages point_cloud_transport_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_dependencies(point_cloud_transport_generate_messages_cpp _point_cloud_transport_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_cloud_transport_gencpp)
add_dependencies(point_cloud_transport_gencpp point_cloud_transport_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_cloud_transport_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(point_cloud_transport
  "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_cloud_transport
)

### Generating Services

### Generating Module File
_generate_module_eus(point_cloud_transport
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_cloud_transport
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(point_cloud_transport_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(point_cloud_transport_generate_messages point_cloud_transport_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_dependencies(point_cloud_transport_generate_messages_eus _point_cloud_transport_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_cloud_transport_geneus)
add_dependencies(point_cloud_transport_geneus point_cloud_transport_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_cloud_transport_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(point_cloud_transport
  "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_cloud_transport
)

### Generating Services

### Generating Module File
_generate_module_lisp(point_cloud_transport
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_cloud_transport
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(point_cloud_transport_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(point_cloud_transport_generate_messages point_cloud_transport_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_dependencies(point_cloud_transport_generate_messages_lisp _point_cloud_transport_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_cloud_transport_genlisp)
add_dependencies(point_cloud_transport_genlisp point_cloud_transport_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_cloud_transport_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(point_cloud_transport
  "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/point_cloud_transport
)

### Generating Services

### Generating Module File
_generate_module_nodejs(point_cloud_transport
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/point_cloud_transport
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(point_cloud_transport_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(point_cloud_transport_generate_messages point_cloud_transport_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_dependencies(point_cloud_transport_generate_messages_nodejs _point_cloud_transport_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_cloud_transport_gennodejs)
add_dependencies(point_cloud_transport_gennodejs point_cloud_transport_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_cloud_transport_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(point_cloud_transport
  "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_cloud_transport
)

### Generating Services

### Generating Module File
_generate_module_py(point_cloud_transport
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_cloud_transport
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(point_cloud_transport_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(point_cloud_transport_generate_messages point_cloud_transport_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jakub/Desktop/ROS_workspace_1/src/point_cloud_transport/msg/PointCloudTransportData.msg" NAME_WE)
add_dependencies(point_cloud_transport_generate_messages_py _point_cloud_transport_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_cloud_transport_genpy)
add_dependencies(point_cloud_transport_genpy point_cloud_transport_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_cloud_transport_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_cloud_transport)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_cloud_transport
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(point_cloud_transport_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_cloud_transport)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_cloud_transport
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(point_cloud_transport_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_cloud_transport)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_cloud_transport
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(point_cloud_transport_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/point_cloud_transport)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/point_cloud_transport
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(point_cloud_transport_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_cloud_transport)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_cloud_transport\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_cloud_transport
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(point_cloud_transport_generate_messages_py std_msgs_generate_messages_py)
endif()
