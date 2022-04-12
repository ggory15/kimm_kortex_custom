cmake_minimum_required(VERSION 3.0.2)
project(kimm_kortex_custum)

add_compile_options(-std=c++17 -Ofast)

INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(KORTEX_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/thirdparty/include")
set(KORTEX_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/thirdparty/lib")
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS system_lib
)
link_directories(${Boost_LIBRARY_DIR})

###########
## Build ##
###########

## Define OS Type
add_definitions(-D_OS_UNIX)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS
  include/utilities/pin_to_urdf.hpp
  include/utilities/kinova_utilities.hpp
)
SET(${PROJECT_NAME}_SOURCES
  src/utilities/pin_to_urdf.cpp
  src/utilities/kinova_utilities.cpp
)


ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp )


TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_home src/example/move_to_home_position.cpp)
set(EXTRA_LIB ${EXTRA_LIB} ${KORTEX_LIBRARY_DIR}/release/libKortexApiCpp.a)
set(EXTRA_INC ${EXTRA_INC} ${KORTEX_INCLUDE_DIRS} ${KORTEX_INCLUDE_DIRS}/client ${KORTEX_INCLUDE_DIRS}/common ${KORTEX_INCLUDE_DIRS}/messages ${KORTEX_INCLUDE_DIRS}/client_stubs ${KORTEX_INCLUDE_DIRS}/google)

target_link_libraries(${PROJECT_NAME}_home ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIB})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_home SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} ${EXTRA_INC})
# add_dependencies(${PROJECT_NAME}_home ${PROJECT_NAME}_gencpp  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


