# Robot
string robot_name

# Parameters
int32 ALGOR
bool Stop_AND_Wait
float32 W1
float32 W2
float32 time_step
float32 time_horizon
float32 sphere_radius
float32 CRIT_DIST
float32 DES_VEL
float32 ROT_VEL
float32 K_ROT_MAX
float32 K_ROT_MIN
float32 AZIMUTH_ERROR

# Results
bool robot_stuck
int32 avoided_collisions
int32 fatal_collisions
float32 navigation_time
float32 CA_time
float32 distance_travelled

# Comparative
float32 optimal_time
float32 optimal_distance_travelled
