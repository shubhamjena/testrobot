cmake_minimum_required(VERSION 2.8.3)
project(testrobot_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) # this is only required to get gazebo's directories

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_descriptions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Install python scripts

install(DIRECTORY TR1 kraken_model
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


