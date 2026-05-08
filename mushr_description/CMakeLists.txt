cmake_minimum_required(VERSION 3.5)
project(mushr_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

############################################################
# Helper function to build xacro → urdf
############################################################
function(build_xacro infile outfile)
  cmake_parse_arguments(XACRO "" "" "ARGS" ${ARGN})

  add_custom_command(
    OUTPUT ${outfile}
    COMMAND xacro ${infile} -o ${outfile} ${XACRO_ARGS}
    DEPENDS ${infile}
    COMMENT "Generating ${outfile}"
    VERBATIM
  )
endfunction()

############################################################
# Directories
############################################################
set(ROBOTS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/robots")
set(ROBOTS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/robots")

file(MAKE_DIRECTORY ${ROBOTS_OUTPUT_DIR})

############################################################
# Robot Xacro Definitions (FROM YOUR FILE TREE)
############################################################

# racecar MIT
set(RACECAR_MIT_XACRO "${ROBOTS_SRC_DIR}/racecar-mit.urdf.xacro")
set(RACECAR_MIT_URDF "${ROBOTS_OUTPUT_DIR}/racecar-mit.urdf")

# mushr_tx2 (base model)
set(MUSHR_TX2_XACRO "${ROBOTS_SRC_DIR}/mushr_tx2.urdf.xacro")
set(MUSHR_TX2_URDF "${ROBOTS_OUTPUT_DIR}/mushr_tx2.urdf")
set(MUSHR_TX2_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-tx2.urdf")

# mushr_tx2 color variant
set(MUSHR_TX2_GREEN_BLACK_URDF "${ROBOTS_OUTPUT_DIR}/mushr_tx2_green_black.urdf")
set(MUSHR_TX2_GREEN_BLACK_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-tx2-green-black.urdf")
set(MUSHR_TX2_GREEN_BLACK_ARGS "platform_color:=green" "inset_color:=black")

# mushr_nano (base model)
set(MUSHR_NANO_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano.urdf")
set(MUSHR_NANO_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano.urdf")

# mushr_nano color variants
set(MUSHR_NANO_RED_WHITE_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_red_white.urdf")
set(MUSHR_NANO_RED_WHITE_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-red-white.urdf")
set(MUSHR_NANO_RED_WHITE_ARGS "platform_color:=red" "inset_color:=white")

set(MUSHR_NANO_TEAL_PURPLE_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_teal_purple.urdf")
set(MUSHR_NANO_TEAL_PURPLE_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-teal-purple.urdf")
set(MUSHR_NANO_TEAL_PURPLE_ARGS "platform_color:=teal" "inset_color:=purple")

set(MUSHR_NANO_PURPLE_GOLD_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_purple_gold.urdf")
set(MUSHR_NANO_PURPLE_GOLD_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-purple-gold.urdf")
set(MUSHR_NANO_PURPLE_GOLD_ARGS "platform_color:=purple" "inset_color:=gold")

set(MUSHR_NANO_GREEN_BLACK_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_green_black.urdf")
set(MUSHR_NANO_GREEN_BLACK_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-green-black.urdf")
set(MUSHR_NANO_GREEN_BLACK_ARGS "platform_color:=green" "inset_color:=black")

###############################
# NEW: mushr_base robots
###############################

# mushr_base_nano
set(MUSHR_BASE_NANO_XACRO "${ROBOTS_SRC_DIR}/mushr_base_nano.urdf.xacro")
set(MUSHR_BASE_NANO_URDF "${ROBOTS_OUTPUT_DIR}/mushr_base_nano.urdf")

# mushr_base_tx2
set(MUSHR_BASE_TX2_XACRO "${ROBOTS_SRC_DIR}/mushr_base_tx2.urdf.xacro")
set(MUSHR_BASE_TX2_URDF "${ROBOTS_OUTPUT_DIR}/mushr_base_tx2.urdf")

############################################################
# Build all xacros → urdf
############################################################
build_xacro(${RACECAR_MIT_XACRO} ${RACECAR_MIT_URDF})

build_xacro(${MUSHR_TX2_XACRO} ${MUSHR_TX2_URDF})
build_xacro(${MUSHR_TX2_XACRO} ${MUSHR_TX2_GREEN_BLACK_URDF} ARGS ${MUSHR_TX2_GREEN_BLACK_ARGS})

build_xacro(${MUSHR_NANO_XACRO} ${MUSHR_NANO_URDF})
build_xacro(${MUSHR_NANO_XACRO} ${MUSHR_NANO_RED_WHITE_URDF}   ARGS ${MUSHR_NANO_RED_WHITE_ARGS})
build_xacro(${MUSHR_NANO_XACRO} ${MUSHR_NANO_TEAL_PURPLE_URDF} ARGS ${MUSHR_NANO_TEAL_PURPLE_ARGS})
build_xacro(${MUSHR_NANO_XACRO} ${MUSHR_NANO_PURPLE_GOLD_URDF} ARGS ${MUSHR_NANO_PURPLE_GOLD_ARGS})
build_xacro(${MUSHR_NANO_XACRO} ${MUSHR_NANO_GREEN_BLACK_URDF} ARGS ${MUSHR_NANO_GREEN_BLACK_ARGS})

# NEW: build mushr_base robots
build_xacro(${MUSHR_BASE_NANO_XACRO} ${MUSHR_BASE_NANO_URDF})
build_xacro(${MUSHR_BASE_TX2_XACRO} ${MUSHR_BASE_TX2_URDF})

############################################################
# Collect everything into build target
############################################################
set(CORE_DESCRIPTION_FILES
  ${RACECAR_MIT_URDF}
  ${MUSHR_TX2_URDF}
  ${MUSHR_TX2_GREEN_BLACK_URDF}
  ${MUSHR_NANO_URDF}
  ${MUSHR_NANO_RED_WHITE_URDF}
  ${MUSHR_NANO_TEAL_PURPLE_URDF}
  ${MUSHR_NANO_PURPLE_GOLD_URDF}
  ${MUSHR_NANO_GREEN_BLACK_URDF}
  ${MUSHR_BASE_NANO_URDF}
  ${MUSHR_BASE_TX2_URDF}
)

add_custom_target(description_files ALL DEPENDS ${CORE_DESCRIPTION_FILES})

############################################################
# Install xacros + meshes + urdfs
############################################################

install(DIRECTORY robots meshes
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ${CORE_DESCRIPTION_FILES}
  DESTINATION share/${PROJECT_NAME}/robots
)

############################################################
ament_package()
############################################################
