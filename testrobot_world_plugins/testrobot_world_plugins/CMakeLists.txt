cmake_minimum_required(VERSION 2.8.3)
project(testrobot_world_plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_msgs)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

include(FindPkgConfig)
pkg_check_modules(OGRE OGRE)

catkin_package(
	INCLUDE_DIRS include
               ${OGRE_INCLUDE_DIRS}
               ${GAZEBO_INCLUDE_DIRS}
               ${Boost_INCLUDE_DIRS}
	LIBRARIES UnderwaterWorldPlugin
	CATKIN_DEPENDS gazebo_msgs
	DEPENDS gazebo
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
		    ${Boost_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}/Paging # workaround for: "/usr/include/OGRE/Terrain/OgreTerrainPaging.h:33:35: fatal error: OgrePagedWorldSection.h: No such file or directory"
                  )
link_directories(${GAZEBO_LIBRARY_DIRS}
                 ${OGRE_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_BINARY_DIR}/msgs)

###########
## Build ##
###########

add_library(UnderwaterWorldPlugin
  src/UnderwaterWorldPlugin.cc
  src/GaussMarkovProcess.cc)
target_link_libraries(UnderwaterWorldPlugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS UnderwaterWorldPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

