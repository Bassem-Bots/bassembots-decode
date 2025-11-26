# Goal Facing Implementation Guide

## What Was Added

Your robot can now automatically face the goal position while driving. The implementation calculates the heading from the robot's current position to the goal and automatically adjusts rotation.

## New Methods in RobotControl.java

### `calculateHeadingToGoal()`
Calculates the angle (in radians) from the robot's current position to the goal.
- Uses odometry to get robot position
- Uses GOAL_X and GOAL_Y constants (currently set to 2633.769mm, 1004.791mm)
- Returns angle using `Math.atan2(deltaY, deltaX)`

### `calculateTurnToGoal(double kP)`
Calculates the turn power needed to face the goal.
- `kP`: Proportional gain (recommended: 0.5 to 1.5)
- Returns turn power from -1.0 to 1.0
- Automatically normalizes angle error to [-π, π]

### `driveWithGoalFacing(double axial, double lateral, double mod, double turnKP)`
Drive the robot while automatically facing the goal.
- `axial`: Forward/backward movement
- `lateral`: Left/right strafe
- `mod`: Speed modifier
- `turnKP`: Proportional gain for turning (1.0 is default)

## Controls in MainCode.java

### Toggle Goal-Facing Mode
- **Press dpad_left** to toggle goal-facing mode ON/OFF
- When ON: Robot automatically rotates to face goal while you drive
- When OFF: Normal manual rotation control with right stick

### Telemetry Display
- Shows current robot heading
- Shows calculated goal heading
- Shows goal-facing mode status

## How to Use

1. **Set Goal Position**: Update `GOAL_X` and `GOAL_Y` in RobotControl.java to match your field coordinates
2. **Drive Normally**: Use left stick for movement
3. **Toggle Mode**: Press dpad_left to enable goal-facing
4. **Adjust Sensitivity**: Change the `1.0` parameter in `driveWithGoalFacing()` to tune rotation speed
   - Higher values (1.5-2.0): Faster, more aggressive turning
   - Lower values (0.5-0.8): Slower, smoother turning

## Example Usage in Autonomous

```java
// In your autonomous code
robot.driveWithGoalFacing(0.5, 0, 0.8, 1.2);  // Drive forward while facing goal
```

## Tuning Tips

- If robot oscillates (wobbles back and forth), reduce the kP value
- If robot turns too slowly, increase the kP value
- Make sure odometry is properly calibrated for accurate heading calculation
