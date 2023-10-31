cmake_minimum_required(VERSION 3.0.2)
project(robot_movement_interface)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs actionlib std_msgs geometry_msgs message_generation)

add_message_files(
  FILES
  Result.msg
  Command.msg
  CommandList.msg
  EulerFrame.msg
  ErrorCodeInformation.msg

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
  IOChanges.msg

# KEBA PLC 
  IECVariable.msg
  IECVariableList.msg
)

add_service_files(
  FILES
  SetFloat.srv
  GetBool.srv
  GetFloat.srv
  GetFloatList.srv
  SetString.srv
  GetString.srv
  GetErrorCodeInformation.srv

# old io system
  GetIO.srv
  SetIO.srv

# new io system
  GetDigitalIOs.srv
  SetDigitalIOs.srv
  GetAnalogIOs.srv
  SetAnalogIOs.srv
  GetCategoryIOs.srv
  GetIOGroupInfo.srv
  GetAllIOGroupInfos.srv
  GetIOInfo.srv
  GetIOCategories.srv
  SetIODisplayTitle.srv
  SetIOSimulated.srv
  SetIOCategories.srv

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
