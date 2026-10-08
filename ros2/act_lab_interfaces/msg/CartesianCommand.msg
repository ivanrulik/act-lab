# Intent must pass SafeCartesianRobot; header.frame_id identifies target_pose.
std_msgs/Header header
string episode_id
uint64 sequence
geometry_msgs/Pose target_pose
float64 gripper_position
bool enabled
