# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "move_plan: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imove_plan:/home/tjark/YOLO_move/src/move_plan/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(move_plan_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_move_plan_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_plan" "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" "move_plan/BoundingBox:std_msgs/Header"
)

get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_move_plan_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_plan" "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_plan
)
_generate_msg_cpp(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_plan
)

### Generating Services

### Generating Module File
_generate_module_cpp(move_plan
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_plan
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(move_plan_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(move_plan_generate_messages move_plan_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_cpp _move_plan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_cpp _move_plan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_plan_gencpp)
add_dependencies(move_plan_gencpp move_plan_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_plan_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_plan
)
_generate_msg_eus(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_plan
)

### Generating Services

### Generating Module File
_generate_module_eus(move_plan
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_plan
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(move_plan_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(move_plan_generate_messages move_plan_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_eus _move_plan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_eus _move_plan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_plan_geneus)
add_dependencies(move_plan_geneus move_plan_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_plan_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_plan
)
_generate_msg_lisp(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_plan
)

### Generating Services

### Generating Module File
_generate_module_lisp(move_plan
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_plan
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(move_plan_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(move_plan_generate_messages move_plan_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_lisp _move_plan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_lisp _move_plan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_plan_genlisp)
add_dependencies(move_plan_genlisp move_plan_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_plan_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_plan
)
_generate_msg_nodejs(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_plan
)

### Generating Services

### Generating Module File
_generate_module_nodejs(move_plan
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_plan
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(move_plan_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(move_plan_generate_messages move_plan_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_nodejs _move_plan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_nodejs _move_plan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_plan_gennodejs)
add_dependencies(move_plan_gennodejs move_plan_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_plan_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan
)
_generate_msg_py(move_plan
  "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan
)

### Generating Services

### Generating Module File
_generate_module_py(move_plan
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(move_plan_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(move_plan_generate_messages move_plan_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_py _move_plan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tjark/YOLO_move/src/move_plan/msg/BoundingBox.msg" NAME_WE)
add_dependencies(move_plan_generate_messages_py _move_plan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_plan_genpy)
add_dependencies(move_plan_genpy move_plan_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_plan_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_plan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_plan
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(move_plan_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_plan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_plan
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(move_plan_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_plan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_plan
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(move_plan_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_plan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_plan
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(move_plan_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_plan
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(move_plan_generate_messages_py std_msgs_generate_messages_py)
endif()
