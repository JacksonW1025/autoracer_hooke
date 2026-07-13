# Vehicle Gear cmd system

std_msgs/Header header

# Control Message
# Gear enable cmd
uint8 GEAR_EN_CTRL_DISABLE = 0
uint8 GEAR_EN_CTRL_ENABLE = 1

# Gear position target
uint8 GEAR_TARGET_INVALID = 0
uint8 GEAR_TARGET_PARK = 1
uint8 GEAR_TARGET_REVERSE = 2
uint8 GEAR_TARGET_NEUTRAL = 3
uint8 GEAR_TARGET_DRIVE = 4

uint8 gear_target		#  vehicle gear position cmd, 0-GEAR_TARGET_INVALID, 1-GEAR_TARGET_PARK, 2-GEAR_TARGET_REVERSE, 3-GEAR_TARGET_NEUTRAL, 4-GEAR_TARGET_DRIVE
int8 gear_en_ctrl		#  vehicle gear enable cmd, 0-GEAR_EN_CTRL_DISABLE, 1-GEAR_EN_CTRL_ENABLE
int32 checksum_103		#  [0|255]
