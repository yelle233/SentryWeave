# One robot's status. stamp is the original source time of these values.
# Each *_valid flag covers only its own value; an invalid value is unknown, never 0 or full.
builtin_interfaces/Time stamp
uint16 robot_id               # official number: 1, 3, 4, 6, 7 (red) / 101, 103, ... (blue)
uint8 UNKNOWN=0
uint8 ALLY=1
uint8 ENEMY=2
uint8 team                    # relative to the own team, like RobotObservation
float64 hp                    # 0 means not alive unless alive_valid says otherwise
float64 max_hp
bool hp_valid                 # covers hp only; hp is kept when max_hp is unknown
bool max_hp_valid             # covers max_hp only; without it the hp ratio stays unknown
bool alive
bool alive_valid
uint16 level
bool level_valid
float64 ammo_ratio            # remaining allowance in [0, 1]
bool ammo_valid
uint32 SOURCE_REFEREE=1
uint32 SOURCE_RADAR=2
uint32 SOURCE_ONBOARD=4
uint32 SOURCE_TEAM=8
uint32 source                 # bit mask of SOURCE_*
