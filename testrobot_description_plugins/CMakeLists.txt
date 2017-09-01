cmake_minimum_required(VERSION 2.8.3)
project(testrobot_description_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  std_msgs 
)
  
find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES thrusterPlugin
 CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(thrusterPlugin src/thrusterPlugin.cc)
target_link_libraries(thrusterPlugin ${catkin_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES})
add_dependencies(thrusterPlugin ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS thrusterPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".~" EXCLUDE
)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  #target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  #target_link_libraries(vel ${GAZEBO_LIBRARIES})
endif()
