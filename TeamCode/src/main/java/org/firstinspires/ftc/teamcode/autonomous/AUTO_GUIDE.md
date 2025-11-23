# FTC Into The Deep - Autonomous Guide

## MainAuto.java Overview

The MainAuto autonomous program is now ready for FTC Into The Deep competition. It includes:

### Features
- **Odometry-based navigation** using GoBilda Pinpoint Driver
- **AprilTag detection** for precise positioning
- **Modular action methods** for easy customization
- **Field coordinate system** for precise movements

### Key Components

#### 1. Field Positions
Adjust these constants based on your field measurements (in millimeters):
```java
START_X, START_Y, START_HEADING          // Starting position
SUBMERSIBLE_X, SUBMERSIBLE_Y             // Submersible scoring zone
OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y   // Sample collection area
NET_ZONE_X, NET_ZONE_Y                   // Net scoring zone
```

#### 2. Main Sequence
The `runAutonomousSequence()` method contains your autonomous routine:
1. Move to submersible
2. Score specimen
3. Move to observation zone
4. Collect samples
5. Move to net zone
6. Score in net
7. Park using AprilTag or odometry

#### 3. Action Methods
Customize these methods for your robot's mechanisms:
- `scoreSpecimen()` - Score on submersible
- `collectSamples()` - Intake samples from field
- `scoreInNet()` - Score samples in net
- `parkRobot()` - Park in designated zone

### Customization Tips

#### Adjust Movement Parameters
```java
auto.moveTo(x, y, heading, power, timeout, controlPower);
```
- `x, y` - Target position in mm
- `heading` - Target angle in degrees
- `power` - Movement speed (0.0 to 1.0)
- `timeout` - Max time in seconds
- `controlPower` - Control system power limit

#### Use AprilTag Navigation
```java
auto.navigateToAprilTag(tagId, distance, timeout);
```
- `tagId` - AprilTag ID to navigate to
- `distance` - Desired distance in inches
- `timeout` - Max time in seconds

#### Control Robot Mechanisms
```java
auto.robot.setShooterPower(ShooterPower.HIGH);  // OFF, LOW, MEDIUM, SEMI, HIGH
auto.robot.runIntake();                          // Run intake forward
auto.robot.reverseIntake();                      // Run intake backward
auto.robot.stopIntake();                         // Stop intake
```

### Testing Checklist
1. ✓ Verify field coordinate system matches your setup
2. ✓ Test individual movement commands
3. ✓ Calibrate odometry offsets
4. ✓ Test AprilTag detection
5. ✓ Adjust timing for scoring actions
6. ✓ Test full autonomous sequence
7. ✓ Add alliance-specific variations if needed

### Creating Alliance-Specific Versions
To create separate programs for Red/Blue alliances:
1. Copy MainAuto.java
2. Rename to RedAuto.java or BlueAuto.java
3. Adjust field positions and AprilTag IDs
4. Update @Autonomous annotation name

### Troubleshooting
- **Robot doesn't move**: Check motor directions in RobotControl.java
- **Inaccurate positioning**: Calibrate odometry offsets in AutoControl.java
- **AprilTag not detected**: Verify camera configuration and lighting
- **Timeout errors**: Increase timeout values or adjust movement speed
