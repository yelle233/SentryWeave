std_msgs/Header header
string session_id
uint64 task_id
uint64 based_on_world_seq
uint8 HOLD=0
uint8 SUPPORT=1
uint8 INTERCEPT=2
uint8 DEFEND=3
uint8 RECOVER=4
uint8 task_type
string target_zone_id
uint16 target_robot_id
bool target_robot_valid
string goal_id
geometry_msgs/PoseStamped target_pose
bool target_pose_valid
float64 utility
float64 confidence
bool urgent
builtin_interfaces/Time created_at
builtin_interfaces/Time valid_until
string reason_code
uint8 POSTURE_ATTACK=1
uint8 POSTURE_DEFENSE=2
uint8 POSTURE_MOVE=3
string posture                # "attack" | "defense" | "move"
bool posture_enhanced         # 2027 enhanced posture request
bool spin_enabled             # reactive chassis spin active flag
float64 spin_rate_rad_s       # target chassis spin rate in rad/s (0 when disabled)

