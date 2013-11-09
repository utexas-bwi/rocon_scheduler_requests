cmake_minimum_required(VERSION 2.8.3)
project(scheduler_request_manager)

## Dependencies -- mostly needed for the eventual C++ interface.
set(${PROJECT_NAME}_MSG_DEPENDENCIES
  rocon_std_msgs
  scheduler_msgs
  uuid_msgs)
find_package(catkin REQUIRED
  COMPONENTS unique_id ${${PROJECT_NAME}_MSG_DEPENDENCIES})

## Install Python modules
catkin_python_setup()

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
