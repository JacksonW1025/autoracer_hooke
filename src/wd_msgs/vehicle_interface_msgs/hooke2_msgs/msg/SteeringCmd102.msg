# Vehicle Steer cmd system

std_msgs/Header header

# Control Message
# Steering enable cmd
uint8 STEER_EN_CTRL_DISABLE = 0
uint8 STEER_EN_CTRL_ENABLE = 1

uint8 steer_en_ctrl		    #  vehicle steer enable cmd,0 - STEER_EN_CTRL_DISABLE, 1 - STEER_EN_CTRL_ENABLE
int32 steer_angle_target	#  vehicle steer cmd, unit: degree, [-465|465]
int32 steer_angle_spd		#  vehicle steer speed cmd, unit: deg/s, [0|250]
int32 checksum_102		    #  [0|255]
