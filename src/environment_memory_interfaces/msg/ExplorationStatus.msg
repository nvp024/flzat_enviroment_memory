uint8 WAITING=0
uint8 EXPLORING=1
uint8 FINALIZING=2
uint8 COMPLETED=3
uint8 FAILED=4

builtin_interfaces/Time stamp
uint8 state
uint32 goals_seen
uint32 goals_succeeded
uint32 goals_failed
geometry_msgs/PoseStamped current_goal
string reason
