# TeamCode Project Structure

## Organized Folders

The code is now organized into clear, logical folders:

### 📁 `/teleop`
TeleOp (driver-controlled) OpModes:
- `MainCode.java` - Main TeleOp program
- `Code_Z.java` - Test TeleOp with AprilTag assist
- `TeleOpWithAprilTag.java` - Example TeleOp with AprilTag integration

### 📁 `/autonomous`
Autonomous OpModes:
- `AprilTagAutonomous.java` - AprilTag-based autonomous navigation
- `TestAutoOp.java` - Test autonomous sequence

### 📁 `/test`
Testing and diagnostic OpModes:
- `MotorTest.java` - Individual motor testing
- `TestOp.java` - General hardware testing

### 📁 `/util`
Utility classes and helpers:
- `RobotControl.java` - Core robot control and drive functions
- `AutoControl.java` - Autonomous control utilities
- `EnhancedNavigation.java` - PIDF navigation system
- `GoBildaPinpointDriver.java` - Odometry computer driver
- `AprilTagHelper.java` - AprilTag detection utilities
- `ShooterPower.java` - Shooter power level enum

## Benefits of This Structure

✅ **Easy to find files** - Everything is categorized by purpose
✅ **Better collaboration** - Team members know where to add new code
✅ **Cleaner imports** - Proper package structure
✅ **Professional organization** - Follows Java best practices
