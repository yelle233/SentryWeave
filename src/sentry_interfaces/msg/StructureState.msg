# One structure. stamp is the original source time of these values.
builtin_interfaces/Time stamp
string id                     # own_base, own_outpost, enemy_base, enemy_outpost or a field-defined id
float64 hp
float64 max_hp
bool hp_valid                 # covers hp and max_hp
bool alive
bool alive_valid
float64 shield
bool shield_valid
uint32 SOURCE_REFEREE=1
uint32 SOURCE_RADAR=2
uint32 SOURCE_ONBOARD=4
uint32 SOURCE_TEAM=8
uint32 source                 # bit mask of SOURCE_*
