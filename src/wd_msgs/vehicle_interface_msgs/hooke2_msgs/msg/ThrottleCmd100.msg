# Vehicle throttle drive cmd system

std_msgs/Header header

# Control Message
uint8 THROTTLE_EN_CTRL_DISABLE = 0
uint8 THROTTLE_EN_CTRL_ENABLE = 1

float64 vel_target		#  vehicle velocity cmd, unit: m/s, [0|10.23]
float64 throttle_acc		#  vehicle acceleration cmd, unit: m/s^2, [0|10]
int32 checksum_100		#  [0|255]
float64 throttle_pedal_target	#  vehicle throttle_pedal cmd, unit: %, [0|100]
uint8 throttle_en_ctrl		#  vehicle throttle drive enable cmd, 0:'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE
