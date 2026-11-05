# Original acquisition timestamp, never a relay publication timestamp.
std_msgs/Header header
uint16 robot_id
uint8 UNKNOWN=0
uint8 ALLY=1
uint8 ENEMY=2
uint8 team
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
bool velocity_valid
float64 confidence
uint32 source_mask
