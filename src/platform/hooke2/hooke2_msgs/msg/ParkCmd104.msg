# Vehicle Park cmd system

std_msgs/Header header

# Control Message
# Park enable cmd
uint8 PARK_EN_CTRL_DISABLE = 0
uint8 PARK_EN_CTRL_ENABLE = 1

# Park position target cmd
uint8 PARK_TARGET_RELEASE = 0
uint8 PARK_TARGET_PARKING_TRIGGER = 1

int32 checksum_104		#  [0|255]
uint8 park_target		#  vehicle park position cmd, 0-PARK_TARGET_RELEASE, 1-PARK_TARGET_PARKING_TRIGGER
uint8 park_en_ctrl		#  vehicle park enable cmd, 0-PARK_EN_CTRL_DISABLE, 1-PARK_EN_CTRL_ENABLE
