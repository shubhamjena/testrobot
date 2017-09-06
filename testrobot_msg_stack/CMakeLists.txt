cmake_minimum_required(VERSION 2.8.3)
project(testrobot_msg_stack)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
std_msgs
geometry_msgs  
message_generation)

add_message_files(
  FILES
  forceTorqueData.msg
  thrusterData6.msg
  FloatStamped.msg
  UnderwaterObjectModel.msg
)

add_service_files(
  FILES
  SetUseGlobalCurrentVel.srv
  GetModelProperties.srv
  SetCurrentModel.srv
  GetCurrentModel.srv
  SetCurrentVelocity.srv
  SetCurrentDirection.srv
  SetOriginSphericalCoord.srv
  GetOriginSphericalCoord.srv
)

generate_messages(
DEPENDENCIES
std_msgs
 geometry_msgs
)


catkin_package(
CATKIN_DEPENDS 
message_runtime
)






