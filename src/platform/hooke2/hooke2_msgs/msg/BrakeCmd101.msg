# Vehicle Brake cmd system

std_msgs/Header header

# Control Message
# AEB cmd
uint8 AEB_EN_CTRL_DISABLE_AEB = 0
uint8 AEB_EN_CTRL_ENABLE_AEB = 1

# Brake enable cmd
uint8 BRAKE_EN_CTRL_DISABLE = 0
uint8 BRAKE_EN_CTRL_ENABLE = 1

uint8 aeb_en_ctrl		#  vehicle aeb enable cmd, 0 - AEB_EN_CTRL_DISABLE_AEB, 1 - AEB_EN_CTRL_ENABLE_AEB
float64 brake_dec		#  vehicle decceleration cmd, unit: m/s^2, [1.0|10]
int32 checksum_101		#  [0|255]
float64 brake_pedal_target	#  vehicle brake_pedal cmd, unit: %, [0|100]
uint8 brake_en_ctrl		#  vehicle throttle drive enable cmd, 0 - BRAKE_EN_CTRL_DISABLE, 1 - BRAKE_EN_CTRL_ENABLE
