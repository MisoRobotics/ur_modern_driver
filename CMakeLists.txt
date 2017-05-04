cmake_minimum_required(VERSION 2.8.3)
project(ur_modern_driver)
add_definitions( -DROS_BUILD )
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  ur_msgs
  tf)

find_package(message_generation)

add_service_files(
  FILES
  ResetConnection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur_modern_driver ur_hardware_interface
  CATKIN_DEPENDS hardware_interface controller_manager actionlib control_msgs geometry_msgs roscpp sensor_msgs trajectory_msgs ur_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur_hardware_interface src/ur_hardware_interface.cpp)
target_link_libraries(ur_hardware_interface ${catkin_LIBRARIES})

add_executable(ur_driver
  src/ur_ros_wrapper.cpp
  src/ur_driver.cpp
  src/ur_realtime_communication.cpp
  src/ur_communication.cpp
  src/robot_state.cpp
  src/robot_state_RT.cpp
  src/do_output.cpp)
add_dependencies(ur_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_driver
  ur_hardware_interface
  ${catkin_LIBRARIES})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(TARGETS ur_driver ur_hardware_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Add all the files to CMake for qtcreator
function(miso_add_custom_files_helper glob_pattern target_name)
  FILE(GLOB_RECURSE target_files ${glob_pattern})
  add_custom_target(${target_name} SOURCES ${target_files})
endfunction(miso_add_custom_files_helper)

function(miso_add_custom_files glob_pattern type)
  miso_add_custom_files_helper(${glob_pattern} ${PROJECT_NAME}_${type})
endfunction(miso_add_custom_files)

function(miso_add_config_files)
  miso_add_custom_files("config/*" config_files)
endfunction(miso_add_config_files)

function(miso_add_headers)
  miso_add_custom_files("include/*.h*" headers)
endfunction(miso_add_headers)

function(miso_add_launch_files)
  miso_add_custom_files("launch/*.launch" launch_files)
endfunction(miso_add_launch_files)

function(miso_add_scripts)
  miso_add_custom_files("scripts/*" scripts)
endfunction(miso_add_scripts)

function(miso_add_ros_files)
  miso_add_config_files()
  miso_add_headers()
  miso_add_launch_files()
endfunction(miso_add_ros_files)

miso_add_ros_files()
