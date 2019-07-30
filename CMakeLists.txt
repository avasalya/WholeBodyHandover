cmake_minimum_required(VERSION 2.8)

# matplotlib #
# FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})



# set(CXX_DISABLE_WERROR 1)
# set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# include(cmake/base.cmake)
# include(cmake/boost.cmake)

set(PROJECT_NAME handovercontroller)
set(PROJECT_DESCRIPTION "human-robot tool handover controller for mc_rtc")
set(PROJECT_URL https://gite.lirmm.fr/ashesh/handovercontroller)

# setup_project()
project(${PROJECT_NAME} CXX)  # because setup_project() is not called

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
# add_required_dependency(mc_rtc)
find_package(mc_rtc REQUIRED)


add_subdirectory(src)
add_subdirectory(states)
add_subdirectory(cortex)


configure_file(etc/mc_rtc.conf ${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc.conf)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc.conf
        DESTINATION ${CMAKE_INSTALL_PREFIX}/etc)

set(AROBASE "@")
execute_process(COMMAND rospack find handover_urdf
	            RESULT_VARIABLE HANDOVER_URDF_FOUND
	            OUTPUT_VARIABLE HANDOVER_URDF_PATH
	            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
configure_file(etc/Handover.in.conf ${CMAKE_CURRENT_BINARY_DIR}/etc/Handover.conf)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/Handover.conf
    DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

# setup_project_finalize()
