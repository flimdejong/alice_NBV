# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "alice_octomap: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(alice_octomap_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_custom_target(_alice_octomap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alice_octomap" "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(alice_octomap
  "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alice_octomap
)

### Generating Module File
_generate_module_cpp(alice_octomap
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alice_octomap
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(alice_octomap_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(alice_octomap_generate_messages alice_octomap_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_dependencies(alice_octomap_generate_messages_cpp _alice_octomap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alice_octomap_gencpp)
add_dependencies(alice_octomap_gencpp alice_octomap_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alice_octomap_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(alice_octomap
  "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alice_octomap
)

### Generating Module File
_generate_module_eus(alice_octomap
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alice_octomap
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(alice_octomap_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(alice_octomap_generate_messages alice_octomap_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_dependencies(alice_octomap_generate_messages_eus _alice_octomap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alice_octomap_geneus)
add_dependencies(alice_octomap_geneus alice_octomap_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alice_octomap_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(alice_octomap
  "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alice_octomap
)

### Generating Module File
_generate_module_lisp(alice_octomap
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alice_octomap
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(alice_octomap_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(alice_octomap_generate_messages alice_octomap_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_dependencies(alice_octomap_generate_messages_lisp _alice_octomap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alice_octomap_genlisp)
add_dependencies(alice_octomap_genlisp alice_octomap_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alice_octomap_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(alice_octomap
  "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alice_octomap
)

### Generating Module File
_generate_module_nodejs(alice_octomap
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alice_octomap
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(alice_octomap_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(alice_octomap_generate_messages alice_octomap_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_dependencies(alice_octomap_generate_messages_nodejs _alice_octomap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alice_octomap_gennodejs)
add_dependencies(alice_octomap_gennodejs alice_octomap_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alice_octomap_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(alice_octomap
  "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alice_octomap
)

### Generating Module File
_generate_module_py(alice_octomap
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alice_octomap
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(alice_octomap_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(alice_octomap_generate_messages alice_octomap_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/flimdejong/catkin_ws/src/alice_octomap/srv/octomap.srv" NAME_WE)
add_dependencies(alice_octomap_generate_messages_py _alice_octomap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alice_octomap_genpy)
add_dependencies(alice_octomap_genpy alice_octomap_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alice_octomap_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alice_octomap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alice_octomap
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(alice_octomap_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alice_octomap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alice_octomap
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(alice_octomap_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alice_octomap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alice_octomap
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(alice_octomap_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alice_octomap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alice_octomap
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(alice_octomap_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alice_octomap)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alice_octomap\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alice_octomap
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(alice_octomap_generate_messages_py std_msgs_generate_messages_py)
endif()
