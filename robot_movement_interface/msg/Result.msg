# Header include time stamps
Header header

# Executed command reference
uint32 command_id

# Command result code
int32 result_code

int32 SUCCESS = 0
int32 FAILURE_EXECUTION = 3000
int32 FAILURE_COMPONENT_NOT_READY = 3010
int32 FAILURE_TARGET_NOT_REACHABLE = 3020
int32 FAILURE_OUT_OF_REACH = 3021
int32 FAILURE_ROBOT_CONFIGURATION = 3022
int32 FAILURE_JOINT_LIMIT = 3023
int32 FAILURE_CROSSING_SINGULARITY = 3024
int32 FAILURE_EMERGENCY_STOP = 3030
int32 FAILURE_COMMUNICATION = 3040

# Additional information
string additional_information
