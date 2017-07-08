# Pose related variables (position and speed)
int32 position
int32 speed

# MotionHandler
# Internal state related variables
uint8  motion_handler_state
uint16 current_traj_id 
uint16 current_set_point_id # only valid if we are following trajectory
int32  position_error

# A guess at some motion handler states (TBC)
uint8 MOTION_HANDLER_IDLE=0
uint8 MOTION_HANDLER_FOLLOWING_TRAJ=1
uint8 MOTION_HANDLER_MANUAL_CONTROL=2
uint8 MOTION_HANDLER_SEEKING_POSITION=3
uint8 MOTION_HANDLER_STOPPED_OK=4
uint8 MOTION_HANDLER_STOPPING_ERROR=5
uint8 MOTION_HANDLER_STOPPED_ERROR=6
uint8 MOTION_HANDLER_COMPLETED_TRAJ=7

# A guess at drive modes
#uint8 DRIVE_MODE_MANUAL_SPEED=0
#uint8 DRIVE_MODE_POSITION=1
#uint8 DRIVE_MODE_TRAJ=2

