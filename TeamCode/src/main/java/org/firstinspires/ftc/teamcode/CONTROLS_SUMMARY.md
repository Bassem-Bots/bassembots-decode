# Robot Control Summary

## Gamepad 1 Controls

### Movement (Already implemented)
- **Left Stick**: X and Y movement (axial and lateral)
- **Right Stick**: Rotation (yaw)

### New Controls Added
- **Right Trigger**: Intake motor (hold to run intake)
- **Left Bumper**: Decrease shooter speed (OFF → LOW → MEDIUM → SEMI → HIGH)
- **Right Bumper**: Increase shooter speed (OFF → LOW → MEDIUM → SEMI → HIGH)
- **X Button**: Toggle shooter wheel on/off

### Shooter Power Levels
- OFF: 0% power
- LOW: 25% power  
- MEDIUM: 50% power
- SEMI: 75% power
- HIGH: 100% power

### Other Controls
- **A Button**: Toggle AprilTag assist mode
- **Y Button**: Resume camera streaming
- **B Button**: Stop camera streaming
- **D-pad Up/Down**: Change target AprilTag ID
- **Right Stick Button**: Speed control toggle

## Hardware Requirements
Make sure your robot configuration includes:
- `intake_motor` - for the intake system
- `testMotor` - for the shooter (already configured)

## Usage Notes
- Shooter speed can be adjusted even when shooter is off
- When shooter is toggled on, it will use the currently selected power level
- Intake runs at full power when right trigger is pressed
- All controls include proper debouncing to prevent accidental multiple presses