# Normalized game state. header.stamp is the original source time, never a relay time.
std_msgs/Header header
string season                 # "rmuc2027", "rmuc2026" or empty when unknown
uint8 TEAM_UNKNOWN=0
uint8 RED=1
uint8 BLUE=2
uint8 own_team
uint8 UNKNOWN=0
uint8 READY=1
uint8 RUNNING=2
uint8 FINISHED=3
uint8 phase
bool phase_valid
float64 remaining_time        # seconds
bool remaining_time_valid
bool supply_available         # own supply/recovery zone usable now
bool supply_valid
uint32 SOURCE_REFEREE=1
uint32 SOURCE_RADAR=2
uint32 SOURCE_ONBOARD=4
uint32 SOURCE_TEAM=8
uint32 source                 # bit mask of SOURCE_*
float64 own_gold              # 0x0208 own team remaining gold (>= 0)
bool own_gold_valid
float64 enemy_gold            # 5.7.4 radar RF info-wave enemy remaining gold (>= 0)
bool enemy_gold_valid

