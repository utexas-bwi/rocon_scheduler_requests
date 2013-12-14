cmake_minimum_required(VERSION 2.8.3)
project(rocon_scheduler_requests)

## Dependencies -- mostly needed for the eventual C++ interface.
set(${PROJECT_NAME}_MSG_DEPENDENCIES
  scheduler_msgs
  uuid_msgs)
find_package(catkin REQUIRED
  COMPONENTS roslint unique_id ${${PROJECT_NAME}_MSG_DEPENDENCIES})

## Install Python modules
catkin_python_setup()

# Lint Python modules files
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS}
               tests/example_requester.py)

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
