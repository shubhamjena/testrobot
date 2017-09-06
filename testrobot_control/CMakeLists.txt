cmake_minimum_required(VERSION 2.8.3)
project(testrobot_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
std_msgs 
genmsg 
message_generation
testrobot_msg_stack)

find_package(gazebo REQUIRED)

#add_message_files(DIRECTORY ~/catkin
#FILES
#thrusterData6.msg
#forceTorqueData.msg)

#generate_messages(DEPENDENCIES std_msgs)



catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS 

roscpp 
message_runtime 
std_msgs

)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}

#//not needed..target link libraries serves the purpose
#link_directories(
#  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_executable(pub_test src/pub_test.cc)
target_link_libraries(pub_test ${catkin_LIBRARIES})
add_dependencies(pub_test ${catkin_EXPORTED_TARGETS})

add_executable(thruster_manager src/thruster_manager.cc)
target_link_libraries(thruster_manager ${catkin_LIBRARIES})
add_dependencies(thruster_manager ${catkin_EXPORTED_TARGETS})

add_executable(pub_test_2 src/pub_test_2.cc)
target_link_libraries(pub_test_2 ${catkin_LIBRARIES})
add_dependencies(pub_test_2 ${catkin_EXPORTED_TARGETS})


install(TARGETS pub_test thruster_manager pub_test_2
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        #PATTERN "*~" EXCLUDE)

