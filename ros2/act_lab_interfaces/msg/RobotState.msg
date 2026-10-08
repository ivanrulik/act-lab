# Header identifies the world frame and the measured simulation timestamp.
std_msgs/Header header
string episode_id
string[6] joint_names
float64[6] joint_positions_rad
float64[6] joint_velocities_rad_s
geometry_msgs/Pose end_effector_pose
float64 gripper_position
