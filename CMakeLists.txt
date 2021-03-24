cmake_minimum_required(VERSION 3.10.2)

project(o80)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(synchronizer REQUIRED)
find_package(shared_memory REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(time_series REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
ament_export_dependencies(
  Boost
  mpi_cmake_modules
  pybind11
  synchronizer
  shared_memory
  real_time_tools
  time_series )

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/time_stamp.cpp
  src/time.cpp
  src/command_type.cpp
  src/burster.cpp
  src/bool_state.cpp
  src/void_state.cpp
  src/frequency_manager.cpp
  src/frequency_measure.cpp )
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  synchronizer
  shared_memory
  real_time_tools
  time_series )
target_link_libraries(${PROJECT_NAME} shared_memory::shared_memory)
target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} synchronizer::synchronizer)
target_link_libraries(${PROJECT_NAME} time_series::time_series)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_py MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_py
  PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}"
  OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}_py
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_py DESTINATION ${PYTHON_INSTALL_DIR})


#######################
# debian control file #
#######################

# debian control file in
# the o80_example package


#################
# documentation #
#################

add_documentation()


######################
# Install and export #
######################\

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY internal/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
ament_package()


##############
# Unit tests #
##############

#
# unit tests are in
# the o80_example package
#

