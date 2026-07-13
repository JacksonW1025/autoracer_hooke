# Feedback data related to hooke2.0 steering report.

std_msgs/Header header

bool steer_flt1             	# steer fault1 status, no fault or steer system hardware fault
bool steer_flt2	            	# steer fault2 status, no fault or steer system communication fault
uint8 steer_en_state_type	    # steer enable status, manual - 0, auto = 1, takkeover - 2, stanby - 3
int32 steer_angle_spd_actual	# steer speed status, unit: deg/s
int32 steer_angle_actual	    # steer position, unit: degree
