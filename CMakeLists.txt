cmake_minimum_required(VERSION 2.8.7)

project(o80)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  robot_interfaces
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
 INCLUDE_DIRS include example/include
 CATKIN_DEPENDS
 shared_memory
 real_time_tools
 synchronizer
 robot_interfaces
 time_series
)

include_directories(include
  example/include
  ${catkin_INCLUDE_DIRS} )


################
# Main Library #
################

# Note : this is mostly a header library,
#        so o80 library does not contain much

add_library( o80
  src/time_stamp.cpp
  src/typedefs.cpp
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

#######################
# Example executables #
#######################

add_executable( drivers_demo
  example/src/drivers_demo.cpp)
target_link_libraries(drivers_demo o80 ${catkin_LIBRARIES})

#add_executable( example_standalone
#  example/src/main.cpp)
#target_link_libraries(example_standalone o80)
#add_executable( example_mirror
#  example/src/mirror.cpp)
#target_link_libraries(example_mirror o80)

###################################
# Python bindings for the example #
###################################

pybind11_add_module(o80_example example/srcpy/wrappers.cpp)
target_link_libraries(o80_example PRIVATE o80 ${catkin_LIBRARIES})
install(TARGETS o80_example DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})


##############
# Unit tests #
##############

catkin_add_gtest(o80_unit_tests
  tests/main.cpp
  tests/unit_tests.cpp)
target_link_libraries(o80_unit_tests o80 ${catkin_libraries})

