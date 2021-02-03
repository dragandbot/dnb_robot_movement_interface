# This is a flexible command message for robotic drivers

# Header includes time stamps
Header header

# Command id
# This id will be copied to the result message to identify the executed command
# Although it doesn't affect to the driver functionality, it should be unique to allow future association
uint32 command_id

# Command type contains the move type (LIN, PTP, JOINT_DIRECT, ...)
string command_type

# If applicable, indicates which frame is used as base frame (relative positions)
string pose_reference

# Optional: Frame coordinates to execute the command
# if not set, command will be executed in '/base' frame
EulerFrame pose_reference_frame

string pose_type # EULER_INTRINSIC_ZYX, JOINTS
float32[] pose

string velocity_type # M/S RAD/S PERCENT(0 ... 100) %(0.0 ... 1.0)
float32[] velocity

string acceleration_type # M/S^2 RAD/S^2 PERCENT(0 ... 100) %(0.0 ... 1.0)
float32[] acceleration

string effort_type
float32[] effort

string blending_type # M PERCENT %
float32[] blending

# Reserved for future extension
string[] additional_parameters
float32[] additional_values
