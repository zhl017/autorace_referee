cmake_minimum_required(VERSION 3.0.2)
project(rbiz_autorace_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    sensor_msgs
    std_srvs
    )

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  SensorStateLevelCrossing.msg
  SensorStateTrafficLight.msg
  SensorStateStopwatch.msg
  DoIt.msg
  Time.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs std_srvs message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
