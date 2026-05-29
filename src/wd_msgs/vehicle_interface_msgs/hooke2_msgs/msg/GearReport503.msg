# Feedback data related to hooke2.0 gear report.
uint8 GEAR_ACTUAL_INVALID = 0
uint8 GEAR_ACTUAL_PARK = 1
uint8 GEAR_ACTUAL_REVERSE = 2
uint8 GEAR_ACTUAL_NEUTRAL = 3
uint8 GEAR_ACTUAL_DRIVE =4


std_msgs/Header header

bool gear_flt	        #  gear system fault, no fault - 0, fault - 1
uint8 gear_actual	#  gear position status,  invalid - 0, Park - 1, Reverse - 2, Neutral - 3, Drive - 4
