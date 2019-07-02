cmake_minimum_required(VERSION 2.8.3)
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
    COMMAND ${CATKIN_ENV} rosrun xacro xacro --inorder
    -o "${outfile}" "${infile}" ${_xacro_deps_result} ${xacro_args}
    DEPENDS "${infile}" ${_xacro_deps_result}
    VERBATIM)
endfunction(build_xacro)

set(ROBOTS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/robots")
set(BASE_OUTPUT_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")
set(ROBOTS_OUTPUT_DIR "${BASE_OUTPUT_DIR}/robots")
file(MAKE_DIRECTORY ${ROBOTS_OUTPUT_DIR})

# Build component parts
set(RACECAR_MIT_URDF_XACRO "${ROBOTS_SRC_DIR}/racecar-mit.urdf.xacro")
set(RACECAR_MIT_URDF "${ROBOTS_OUTPUT_DIR}/racecar-mit.urdf")

set(MUSHR_TX2_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_tx2.urdf.xacro")
set(MUSHR_TX2_URDF "${ROBOTS_OUTPUT_DIR}/mushr_tx2.urdf")
set(MUSHR_TX2_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-tx2.urdf")

set(MUSHR_NANO_URDF_XACRO "${ROBOTS_SRC_DIR}/mushr_nano.urdf.xacro")
set(MUSHR_NANO_URDF "${ROBOTS_OUTPUT_DIR}/mushr_nano.urdf")
set(MUSHR_NANO_LINK "${ROBOTS_OUTPUT_DIR}/racecar-uw-nano.urdf")

build_xacro(${RACECAR_MIT_URDF_XACRO} ${RACECAR_MIT_URDF} "")
build_xacro(${MUSHR_NANO_URDF_XACRO} ${MUSHR_NANO_URDF} "")
build_xacro(${MUSHR_TX2_URDF_XACRO} ${MUSHR_TX2_URDF} "")

list(APPEND CORE_DESCRIPTION_FILES
	${RACECAR_MIT_URDF}
	${MUSHR_NANO_URDF}
	${MUSHR_TX2_URDF}
)

# Add target to actually cause generation
add_custom_target(description_files ALL DEPENDS
	${CORE_DESCRIPTION_FILES}
)

add_custom_target(link_mushr_tx2 ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_TX2_URDF} ${MUSHR_TX2_LINK})


add_custom_target(link_mushr_nano ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${MUSHR_NANO_URDF} ${MUSHR_NANO_LINK})

install(DIRECTORY DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/robots")
install(FILES
  ${DESCRIPTION_FILES}
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/robots")

install(DIRECTORY meshes
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

