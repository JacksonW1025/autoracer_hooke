# Feedback data related to hooke2.0 brake report.

std_msgs/Header header

float64 brake_pedal_actual	#  brake pedal position, unit: %, [0, 100]
bool brake_flt1	            #  brake system hardware fault, no fault - 0, hardware fault - 1
bool brake_flt2	    		#  brake system communication fault, no fault - 0, communication fault - 1
uint8 brake_en_state		#  brake enable status,  manual - 0, auto = 1, takkeover - 2, stanby - 3
