# A struct describing a footstep for walking pattern generation.
# It defines, where and when a leg should move.
string leg_name # For Solo8: FL, FR, HL, HR
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
geometry_msgs/Pose pose
