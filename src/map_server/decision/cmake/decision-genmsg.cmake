# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "decision: 5 messages, 0 services")

set(MSG_I_FLAGS "-Idecision:/home/nvidia/hitsz_icra_2019/src/decision/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(decision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" ""
)

get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" ""
)

get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" ""
)

get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" ""
)

get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)

### Generating Services

### Generating Module File
_generate_module_cpp(decision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(decision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(decision_generate_messages decision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_gencpp)
add_dependencies(decision_gencpp decision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)

### Generating Services

### Generating Module File
_generate_module_eus(decision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(decision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(decision_generate_messages decision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_geneus)
add_dependencies(decision_geneus decision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)

### Generating Services

### Generating Module File
_generate_module_lisp(decision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(decision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(decision_generate_messages decision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_genlisp)
add_dependencies(decision_genlisp decision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(decision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(decision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(decision_generate_messages decision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_gennodejs)
add_dependencies(decision_gennodejs decision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)

### Generating Services

### Generating Module File
_generate_module_py(decision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(decision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(decision_generate_messages decision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/RFID.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameInfo.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/Hurt.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/EnemyPos.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/hitsz_icra_2019/src/decision/msg/GameBuff.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_genpy)
add_dependencies(decision_genpy decision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(decision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(decision_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(decision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(decision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(decision_generate_messages_py std_msgs_generate_messages_py)
endif()
