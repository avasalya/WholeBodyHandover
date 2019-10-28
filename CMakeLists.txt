# Copyright 2018-2019 CNRS-UM LIRMM
#
# This file is part of lipm_walking_controller.
#
# lipm_walking_controller is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# lipm_walking_controller is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
# General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with lipm_walking_controller. If not, see
# <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME handover_walking_controller)
set(PROJECT_DESCRIPTION "Handover Controller with Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://gite.lirmm.fr/ashesh/handovercontroller)

set(PROJECT_VERSION 1.1.0)

project(${PROJECT_NAME} CXX)  # because setup_project() is not called

set(CMAKE_CXX_STANDARD 14) # my pc
# set(CMAKE_CXX_STANDARD 11) # hrp2012c pc

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(mc_rtc REQUIRED)
find_package(copra REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  cortex_bridge
  cortex_ros_bridge
  cortex_ros_bridge_msgs
  roscpp
  roslib
  std_msgs
  tf
)

add_subdirectory(src)
