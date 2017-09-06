cmake_minimum_required(VERSION 2.8.3)
project(testrobot_description_plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  std_msgs
  message_generation
  testrobot_msg_stack
  visualization_msgs
)

  
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
                ${Boost_INCLUDE_DIR}
                ${CATKIN_INCLUDE_DIRS}
                ${GAZEBO_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES underwater_Object_Plugin thrusterPlugin
  DEPENDS gazebo
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})


link_directories(${EIGEN3_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

###########
## Build ##
###########

add_library(thrusterPlugin src/thrusterPlugin.cc)
target_link_libraries(thrusterPlugin ${CATKIN_libraries} ${GAZEBO_libraries})
add_dependencies(thrusterPlugin ${catkin_EXPORTED_TARGETS})

add_library(UnderwaterObjectPlugin
            src/HydrodynamicModel.cc
            src/BuoyantObject.cc
            src/UnderwaterObjectPlugin.cc 
 )           
target_link_libraries(UnderwaterObjectPlugin 
 ${GAZEBO_libraries} 
 ${EIGEN3_LIBRARIES}
)

add_library(underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
target_link_libraries(underwater_object_ros_plugin ${catkin_LIBRARIES})
add_dependencies(underwater_object_ros_plugin underwate
  ${catkin_EXPORTED_TARGETS})

#add_dependencies(Underwater_Object_Plugin ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS thrusterPlugin UnderwaterObjectPlugin underwater_object_ros_plugin  
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

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

