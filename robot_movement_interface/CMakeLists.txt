cmake_minimum_required(VERSION 2.8.3)
project(robot_movement_interface)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs actionlib std_msgs geometry_msgs message_generation)

add_message_files(
  FILES
  Result.msg
  Command.msg
  CommandList.msg
  EulerFrame.msg

# old io system
  IOState.msg
  IOStates.msg

# new io system
  IOID.msg
  IOConstants.msg
  IOInfo.msg
  IOError.msg
  DigitalIOValue.msg
  AnalogIOValue.msg
  IOGroupInfo.msg
)

add_service_files(
  FILES
  SetFloat.srv
  GetBool.srv
  GetFloat.srv
  GetFloatList.srv
  SetString.srv

# old io system
  GetIO.srv
  SetIO.srv

# new io system
  GetDigitalIOs.srv
  SetDigitalIOs.srv
  GetAnalogIOs.srv
  SetAnalogIOs.srv
  GetIOGroupInfo.srv
  GetAllIOGroupInfos.srv
  GetIOInfo.srv
  SetIODisplayTitle.srv
  SetIOSimulated.srv

# FK and IK
  GetFK.srv
  GetIK.srv
)

add_action_files(
  DIRECTORY action FILES Commands.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs geometry_msgs
)

#############
## Install ##
#############
install(PROGRAMS
  scripts/actionizer.py
  scripts/notify_robot_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
