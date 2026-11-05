# header.stamp is the last real observation time; pose may be short-term predicted.
std_msgs/Header header
uint16 robot_id
uint8 team
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
bool velocity_valid
uint8 UNKNOWN=0
uint8 OBSERVED=1
uint8 PREDICTED=2
uint8 STALE=3
uint8 status
float64 confidence
float64 age
uint64 generation
RobotObservation[<=128] history
