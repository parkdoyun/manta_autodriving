# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "manta: 1 messages, 1 services")

set(MSG_I_FLAGS "-Imanta:/home/lee/catkin_ws/src/manta/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Imorai_msgs:/home/lee/catkin_ws/src/morai_msgs/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(manta_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_custom_target(_manta_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manta" "/home/lee/catkin_ws/src/manta/msg/student.msg" ""
)

get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_manta_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manta" "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(manta
  "/home/lee/catkin_ws/src/manta/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manta
)

### Generating Services
_generate_srv_cpp(manta
  "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manta
)

### Generating Module File
_generate_module_cpp(manta
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manta
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(manta_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(manta_generate_messages manta_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_dependencies(manta_generate_messages_cpp _manta_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(manta_generate_messages_cpp _manta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manta_gencpp)
add_dependencies(manta_gencpp manta_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manta_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(manta
  "/home/lee/catkin_ws/src/manta/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manta
)

### Generating Services
_generate_srv_eus(manta
  "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manta
)

### Generating Module File
_generate_module_eus(manta
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manta
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(manta_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(manta_generate_messages manta_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_dependencies(manta_generate_messages_eus _manta_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(manta_generate_messages_eus _manta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manta_geneus)
add_dependencies(manta_geneus manta_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manta_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(manta
  "/home/lee/catkin_ws/src/manta/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manta
)

### Generating Services
_generate_srv_lisp(manta
  "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manta
)

### Generating Module File
_generate_module_lisp(manta
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manta
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(manta_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(manta_generate_messages manta_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_dependencies(manta_generate_messages_lisp _manta_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(manta_generate_messages_lisp _manta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manta_genlisp)
add_dependencies(manta_genlisp manta_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manta_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(manta
  "/home/lee/catkin_ws/src/manta/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manta
)

### Generating Services
_generate_srv_nodejs(manta
  "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manta
)

### Generating Module File
_generate_module_nodejs(manta
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manta
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(manta_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(manta_generate_messages manta_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_dependencies(manta_generate_messages_nodejs _manta_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(manta_generate_messages_nodejs _manta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manta_gennodejs)
add_dependencies(manta_gennodejs manta_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manta_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(manta
  "/home/lee/catkin_ws/src/manta/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta
)

### Generating Services
_generate_srv_py(manta
  "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta
)

### Generating Module File
_generate_module_py(manta
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(manta_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(manta_generate_messages manta_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/msg/student.msg" NAME_WE)
add_dependencies(manta_generate_messages_py _manta_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lee/catkin_ws/src/manta/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(manta_generate_messages_py _manta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manta_genpy)
add_dependencies(manta_genpy manta_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manta_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manta
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(manta_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET morai_msgs_generate_messages_cpp)
  add_dependencies(manta_generate_messages_cpp morai_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manta
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(manta_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET morai_msgs_generate_messages_eus)
  add_dependencies(manta_generate_messages_eus morai_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manta
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(manta_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET morai_msgs_generate_messages_lisp)
  add_dependencies(manta_generate_messages_lisp morai_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manta
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(manta_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET morai_msgs_generate_messages_nodejs)
  add_dependencies(manta_generate_messages_nodejs morai_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manta
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(manta_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET morai_msgs_generate_messages_py)
  add_dependencies(manta_generate_messages_py morai_msgs_generate_messages_py)
endif()
