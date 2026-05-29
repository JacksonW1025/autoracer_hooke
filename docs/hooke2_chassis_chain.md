# Hooke2 Chassis Chain

This repository vendors the complete bottom control chain locally:

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/turn_indicators_cmd
/control/command/hazard_lights_cmd
/control/command/emergency_cmd
        |
        v
hooke2_interface
        |
        | publishes /can_rx_from_autoware
        v
can_driver
        |
        | SocketCAN can0, 500000 bps
        v
Hooke2 chassis CAN bus
```

Feedback returns on `/can_tx_to_autoware`, then `hooke2_interface` republishes
vehicle status topics such as:

```text
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/steering_wheel_status
/vehicle/status/gear_status
/vehicle/status/control_mode
```

Use `/vehicle/status/velocity_status` and `/vehicle/status/steering_status` for
localization, planning, and control consumers. Raw `/hooke2/*` reports stay inside
`hooke2_interface` except for debug tooling.

Minimal vehicle-only launch:

```bash
source ./scripts/ros_env.sh
ros2 launch autoracer_bringup vehicle.launch.py
```

For read-only bench feedback validation, prefer:

```bash
RUN_LAUNCH=false ./scripts/verify_sensing_feedback.sh
./scripts/verify_sensing_feedback.sh
```

The legacy-compatible local launch is also available:

```bash
ros2 launch hooke2_launch vehicle_interface.launch.xml
```

Use the smoke test only with a safety operator, working E-stop, and clear wheels:

```bash
./scripts/hooke2_autoware_control_smoke_test.sh 1
```
