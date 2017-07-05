# This message describes the plan made for a particular motor
# It is a little convoluted, as it really shows what the node
# intends to do to achieve the require path (i.e when it plans
# to change speed,accel,decel or desired position). Hence :

# A certain motion can be achieved by doing the following :
# a. Changing the speed at which the motor runs (i.e MAXSPEED)
# b. Changing the desired position (i.e ABSPOS) the driver is trying to achieve
# c. Changing the accel and decel profiles (i.e ACCEL & DECEL)

uint16 traj_id # keep it attached with the plan we require

# Initial conditions
int32 initial_speed
int32 initial_pos
#ProfileCfg initial_profile

# The Plan
int32[] pos_setpoint_changes
uint32[] pos_change_times

int32[] speed_profile_changes
uint32[] speed_change_times

int32[] accel_profile_changes
uint32[] accel_change_times

int32[] decel_profile_changes
uint32[] decel_change_times 
