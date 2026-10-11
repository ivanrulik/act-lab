# Atomic post-command acquisition; original RGB, never viewer JPEG.
std_msgs/Header header
act_lab_interfaces/CommandReport report
string[] image_keys
string[] camera_ids
sensor_msgs/Image[] images
