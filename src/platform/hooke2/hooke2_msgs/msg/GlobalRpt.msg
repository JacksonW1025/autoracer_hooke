# The global report reflects the status of the hooke2 system as a whole.

std_msgs/Header header

bool enabled                     # True if control any system on the hooke2 is enabled
bool override_active             # True if an override was triggered on any system
bool user_can_timeout            # True if a timeout has occurred on the user CAN interface
bool steering_can_timeout        # True if a timeout has occurred on the steering CAN interface
bool brake_can_timeout           # True if a timeout has occurred on the brake CAN interface
bool subsystem_can_timeout       # True if a timeout has occurred when communicating with
                                 # a hooke2 subsystem
bool vehicle_can_timeout         # True if a timeout has occurred on a vehicle CAN interface
bool hooke2_sys_fault_active     # True if a fault is active on any system
bool supervisory_enable_required # True if the platform supports Supervisory Enablement Rules
bool config_fault_active         # True if the CONFIG.TXT file on any component is absent or
                                 # is read incorrectly
uint16 user_can_read_errors      # The number of read errors encoutered by the hooke2 on the user
                                 # CAN interface
bool system_enabled              # True if control any system on the hooke2 is enabled
bool system_override_active      # True if an override was triggered on any system
bool system_fault_active         # True if a fault is active on any system
string vehicle_vin               # The VIN of the vehicle