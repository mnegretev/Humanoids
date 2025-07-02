# Game types
uint8 ROUNDROBIN = 0
uint8 PLAYOFF = 1
uint8 DROPIN = 2

# States
uint8 INITIAL = 0
uint8 READY = 1
uint8 SET = 2
uint8 PLAYING = 3
uint8 FINISHED = 4

# Secondary game states
uint8 NORMAL = 0
uint8 PENALTYSHOOT = 1
uint8 OVERTIME = 2
uint8 TIMEOUT = 3
uint8 DIRECT_FREEKICK = 4
uint8 INDIRECT_FREEKICK = 5
uint8 PENALTYKICK = 6

std_msgs/Header header

string gc_header
uint16 protocol_version
uint8 packet_number
uint8 players_per_team
uint8 game_type
string state
bool first_half
uint8 kick_off_team
string secondary_state
byte[4] secondary_state_info
uint8 drop_in_team
uint16 drop_in_time
uint16 secs_remaining
uint16 secondary_time
TeamInfo[2] teams
