cmake_minimum_required(VERSION 3.0.2)
project(mushr_description)

find_package(catkin REQUIRED)

catkin_package(
CATKIN_DEPENDS realsense2_description
)

function(build_xacro infile outfile xacro_args)
  # Call out to xacro to get dependencies
  execute_process(COMMAND "${CATKIN_ENV} rosrun xacro xacro --deps \"${infile}\""
    ERROR_VARIABLE _xacro_err_ignore
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  separate_arguments(_xacro_deps_result)
  separate_arguments(xacro_args)

  # Process xacro into final output
  add_custom_command(OUTPUT "${outfile}"
    COMMENT "Generating ${outfile}"
    COMMAND ${CATKIN_ENV} rosrun xacro xacro
    -o "${outfile}" "${infile}" ${_xacro_deps_result} ${xacro_args}
    DEPENDS "${infile}" ${_xacro_deps_result}
    VERBATIM)
endfunction(build_xacro)

set(ROBOTS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/robots")
set(BASE_OUTPUT_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")
set(ROBOTS_OUTPUT_DIR "${BASE_OUTPUT_DIR}/robots")
file(MAKE_DIRECTORY ${ROBOTS_OUTPUT_DIR})

# Build component parts
# To compile a URDF with new colors, take the following steps
#	1. Set appropriate URDF_XACRO, URDF, LINK, and ARGS variables
#	   following the template below
#	2. Pass these variables to build_xacro() as shown below
#	3. Add the URDF variable to the CORE_DESCRIPTION_FILES list
#	4. Pass the URDF and link variables to add_custom_target()
#	   following the template below. Note that the first argument
#	   needs to have a unique name
# See mushr_description/robots/color.urdf.xacro for a list of available
# colors. Note that colors can be added to that file using the same format
set(RACECAR_MIT_URDF_XACRO "${ROBOTS_SRC_DIR}/racecar-mit.urdf.xacro")
set(RACECAR_MIT_URDF "${ROBOTS_OUTPUT_DIR}/racecar-mit.urdf")

set(MUSHR_TX2_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_tx2.urdf.xacro")
set(MUSHR_TX2_URDF "${ROBOTS_OUTPUT_DIR}/mushr_tx2.urdf")
set(MUSHR_TX2_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-tx2.urdf")

set(MUSHR_TX2_GREEN_BLACK_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_tx2.urdf.xacro")
set(MUSHR_TX2_GREEN_BLACK_URDF "${ROBOTS_OUTPUT_DIR}/mushr_tx2_green_black.urdf")
set(MUSHR_TX2_GREEN_BLACK_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-tx2-green-black.urdf")
set(MUSHR_TX2_GREEN_BLACK_ARGS "platform_color:=green" "inset_color:=black")

set(MUSHR_NANO_BASE "mushr_nano.urdf")
set(MUSHR_NANO_LINK "racecar-uw-nano.urdf")
set(MUSHR_NANO_URDF_XACRO "${ROBOTS_SRC_DIR}/${MUSHR_NANO_BASE}.xacro")
set(MUSHR_NANO_URDF "${ROBOTS_OUTPUT_DIR}/${MUSHR_NANO_BASE}")
set(MUSHR_NANO_LINK_DEVEL "${ROBOTS_OUTPUT_DIR}/${MUSHR_NANO_LINK}")

set(MUSHR_NANO_RED_WHITE_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_RED_WHITE_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_red_white.urdf")
set(MUSHR_NANO_RED_WHITE_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-red-white.urdf")
set(MUSHR_NANO_RED_WHITE_ARGS "platform_color:=red" "inset_color:=white")

set(MUSHR_NANO_TEAL_PURPLE_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_TEAL_PURPLE_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_teal_purple.urdf")
set(MUSHR_NANO_TEAL_PURPLE_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-teal-purple.urdf")
set(MUSHR_NANO_TEAL_PURPLE_ARGS "platform_color:=teal" "inset_color:=purple")

set(MUSHR_NANO_PURPLE_GOLD_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_PURPLE_GOLD_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_purple_gold.urdf")
set(MUSHR_NANO_PURPLE_GOLD_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-purple-gold.urdf")
set(MUSHR_NANO_PURPLE_GOLD_ARGS "platform_color:=purple" "inset_color:=gold")

set(MUSHR_NANO_GREEN_BLACK_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_GREEN_BLACK_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano_green_black.urdf")
set(MUSHR_NANO_GREEN_BLACK_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano-green-black.urdf")
set(MUSHR_NANO_GREEN_BLACK_ARGS "platform_color:=green" "inset_color:=black")

build_xacro(${RACECAR_MIT_URDF_XACRO} ${RACECAR_MIT_URDF} "")
build_xacro(${MUSHR_TX2_URDF_XACRO} ${MUSHR_TX2_URDF} "")
build_xacro(${MUSHR_TX2_GREEN_BLACK_URDF_XACRO}
						${MUSHR_TX2_GREEN_BLACK_URDF}
						"${MUSHR_TX2_GREEN_BLACK_ARGS}")
build_xacro(${MUSHR_NANO_URDF_XACRO} ${MUSHR_NANO_URDF} "")
build_xacro(${MUSHR_NANO_RED_WHITE_URDF_XACRO} 
						${MUSHR_NANO_RED_WHITE_URDF} 
						"${MUSHR_NANO_RED_WHITE_ARGS}")
build_xacro(${MUSHR_NANO_TEAL_PURPLE_URDF_XACRO} 
						${MUSHR_NANO_TEAL_PURPLE_URDF} 
						"${MUSHR_NANO_TEAL_PURPLE_ARGS}")
build_xacro(${MUSHR_NANO_PURPLE_GOLD_URDF_XACRO} 
						${MUSHR_NANO_PURPLE_GOLD_URDF} 
						"${MUSHR_NANO_PURPLE_GOLD_ARGS}")
build_xacro(${MUSHR_NANO_GREEN_BLACK_URDF_XACRO} 
						${MUSHR_NANO_GREEN_BLACK_URDF} 
						"${MUSHR_NANO_GREEN_BLACK_ARGS}")

list(APPEND CORE_DESCRIPTION_FILES
	${RACECAR_MIT_URDF}
	${MUSHR_TX2_URDF}
	${MUSHR_TX2_GREEN_BLACK_URDF}
	${MUSHR_NANO_URDF}
	${MUSHR_NANO_RED_WHITE_URDF}
	${MUSHR_NANO_TEAL_PURPLE_URDF}
	${MUSHR_NANO_PURPLE_GOLD_URDF}
	${MUSHR_NANO_GREEN_BLACK_URDF}
)

# Add target to actually cause generation
add_custom_target(description_files ALL DEPENDS
	${CORE_DESCRIPTION_FILES}
)

add_custom_target(link_mushr_tx2 ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_TX2_URDF} ${MUSHR_TX2_LINK})
add_custom_target(link_mushr_tx2_green_black ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_TX2_GREEN_BLACK_URDF} ${MUSHR_TX2_GREEN_BLACK_LINK})
add_custom_target(link_mushr_nano ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_URDF} ${MUSHR_NANO_LINK_DEVEL})
add_custom_target(link_mushr_nano_red_white ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_RED_WHITE_URDF} ${MUSHR_NANO_RED_WHITE_LINK})
add_custom_target(link_mushr_nano_teal_purple ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_TEAL_PURPLE_URDF} ${MUSHR_NANO_TEAL_PURPLE_LINK})
add_custom_target(link_mushr_nano_purple_gold ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_PURPLE_GOLD_URDF} ${MUSHR_NANO_PURPLE_GOLD_LINK})
add_custom_target(link_mushr_nano_green_black ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_GREEN_BLACK_URDF} ${MUSHR_NANO_GREEN_BLACK_LINK})

install(DIRECTORY robots DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

install(FILES
  ${CORE_DESCRIPTION_FILES}
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/robots")

install(CODE "execute_process( \
    COMMAND ${CMAKE_COMMAND} -E create_symlink \
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/robots/${MUSHR_NANO_BASE}   \
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/robots/${MUSHR_NANO_LINK} \
    )"
)

install(DIRECTORY meshes
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

