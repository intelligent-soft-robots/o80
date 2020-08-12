cmake_minimum_required(VERSION 2.8.7)

project(o80)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(catkin REQUIRED COMPONENTS
  pybind11_catkin
  synchronizer
  shared_memory
  real_time_tools
  time_series)

# defined in catkin_package mpi_cmake_modules,
# used to detected for example if using real time computer,
# also host various cmake files
DEFINE_OS()

catkin_package(
 LIBRARIES  o80
 INCLUDE_DIRS include internal
 CATKIN_DEPENDS
 pybind11_catkin
 shared_memory
 real_time_tools
 synchronizer
 time_series
)

include_directories(
  internal
  include
  ${catkin_INCLUDE_DIRS}
)


################
# Main Library #
################

add_library( o80
  src/time_stamp.cpp
  src/time.cpp
  src/command_type.cpp
  src/burster.cpp
  src/bool_state.cpp
  src/void_state.cpp
  src/frequency_manager.cpp
  src/frequency_measure.cpp
  src/logger.cpp )
target_link_libraries(o80 ${catkin_LIBRARIES})

###################
# Python bindings #
###################

pybind11_add_module(o80_py srcpy/wrappers.cpp)
target_link_libraries(
  o80_py PRIVATE o80 ${catkin_LIBRARIES})
set_target_properties(o80_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME o80
  )
install(TARGETS o80_py DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})


#################
# Documentation #
#################

build_sphinx_documentation()

##############
# Unit tests #
##############

#
# unit tests are in the o80_example
# packages
#

