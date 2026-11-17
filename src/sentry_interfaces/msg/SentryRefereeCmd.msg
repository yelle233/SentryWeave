# Sentry autonomous referee command (0x0120 sentry_cmd bits 0 & 21-22 + 2027 enhanced flag).
std_msgs/Header header
uint8 POSTURE_KEEP=0
uint8 POSTURE_ATTACK=1
uint8 POSTURE_DEFENSE=2
uint8 POSTURE_MOVE=3
uint8 posture_code            # 1=attack, 2=defense, 3=move (0x0120 bits 21-22)
string posture                # "attack" | "defense" | "move"
bool posture_enhanced         # 2027 enhanced posture request
bool confirm_revive           # 0x0120 bit 0: confirm free respawn when knocked out
uint32 raw_sentry_cmd         # ((posture_code & 0x3) << 21) | int(confirm_revive) for 0x0120 TX
uint64 based_on_task_id
string reason_code
