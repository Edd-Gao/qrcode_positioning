##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(onboard_statemachine)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED
    rosjava_build_tools
    mavros
    sensor_msgs
    onboard_statemachine_msgs
    hover_controller_msgs
)

generate_rosjava_messages(
  PACKAGES
    mavros
    sensor_msgs
)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp)

catkin_package()

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})

#install flight scripts
install (DIRECTORY flight_script/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/flight_script)

install (DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
