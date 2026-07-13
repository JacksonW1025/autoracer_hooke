# Feedback data related to hooke2.0 VCU report.

std_msgs/Header header

uint8 brake_light_actual	#  brake light status, BRAKELIGHT_OFF - 0, BRAKELIGHT_ON - 1
uint8 turn_light_actual		#  turn light status, TURNLAMPSTS_OFF - 0, LEFT_TURNLAMPSTS_ON - 1, RIGHT_TURNLAMPSTS_ON - 2, HAZARD_LAMPSTS_ON - 3
uint8 chassis_errcode		#  vehicle chassis error code, reserved
uint8 drive_mode_sts		#  vehicle chassis driving mode status, THROTTLE_PADDLE_DRIVE - 0, SPEED_DRIVE - 1
uint8 steer_mode_sts		#  vehicle steering mode status, STANDARD - 0, NON_DIRECTION - 1, SYNC_DIRECTION - 2 
uint8 vehicle_mode_state	#  vehicle remote control mode status, MANUAL_REMOTE_MODE - 0, AUTO_MODE - 1, EMERGENCY_MODE - 2, STANDBY_MODE - 3
bool frontcrash_state		#  front crash status, NO_EVENT - 0, CRASH_EVENT - 1
bool backcrash_state		#  back crash status, NO_EVENT - 0, CRASH_EVENT - 1
bool aeb_state			    #  vehicle chassis AEB status, AEB_STATE_INACTIVE - 0, AEB_STATE_ACTIVE - 1
float64 acc			        #  vehicle acceleration data, unit:m/s^2, [-10|10]
float64 speed			    #  vehicle speed data, unit: m/s, [-32.768|32.767]
