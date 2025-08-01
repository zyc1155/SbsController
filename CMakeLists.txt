cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR ON)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)

set(PROJECT_NAME SbsController)
set(PROJECT_DESCRIPTION "SbsController")
set(PROJECT_URL "")

include(cmake/base.cmake)
project(${PROJECT_NAME} CXX)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  ros_falcon
)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARIES})

set(CONFIG_OUT "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/SbsController.yaml")
set(CONFIG_DIR_STANDALONE "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/SbsController")
set(CONFIG_DIR_CATKIN "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/SbsController")
if(EXISTS "${CONFIG_DIR_STANDALONE}" AND IS_DIRECTORY "${CONFIG_DIR_STANDALONE}")
  execute_process(COMMAND cmake -E create_symlink
      "${CONFIG_DIR_STANDALONE}" "${CONFIG_DIR_CATKIN}"
    )
endif()

# set(STATES_INSTALL_PREFIX ${CATKIN_DEVEL_PREFIX}/lib)

configure_file(etc/SbsController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/SbsController DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")


