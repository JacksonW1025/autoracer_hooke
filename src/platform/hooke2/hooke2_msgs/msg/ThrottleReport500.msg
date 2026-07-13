# Feedback data related to hooke2.0 throttle report.

std_msgs/Header header

float64 throttle_pedal_actual	#  throttle pedal position, unit: %, [0, 100]
bool throttle_flt1	        #  throttle system hardware fault, no fault - 0, hardware fault - 1
bool throttle_flt2	    	#  throttle system communication fault, no fault - 0, communication fault - 1
uint8 throttle_en_state		#  throttle enable status,  manual - 0, auto = 1, takkeover - 2, stanby - 3
