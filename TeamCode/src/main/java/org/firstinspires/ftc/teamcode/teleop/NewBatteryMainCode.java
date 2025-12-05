package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.util.RobotControlBattery;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.EnhancedNavigation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Locale;

@TeleOp(name="Main Code With Different Battery Methods", group="Usethis")
public class NewBatteryMainCode extends LinearOpMode {
    // Target coordinates for shooting
    private static final double TARGET_X = 800;  // mm
    private static final double TARGET_Y = 3257;     // mm
    private static final double NET_HEIGHT_INCHES = 37.0;  // inches
    private static final double NET_HEIGHT_MM = NET_HEIGHT_INCHES * 25.4;  // Convert to mm
    private static final double LAUNCH_ANGLE_DEG = 65.0;  // Launch angle of shooter (degrees, steeper than 45)
    
    // Tuning constants
    private static final double WAIT_TIME_CONSTANT = 8;  // seconds per unit of power (4-5 seconds from 0-1)
    private static final double SHOOT_POWER_CONSTANT = 0.74;  // Adjust this to tune actual motor strength
    
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControlBattery robot = new RobotControlBattery(this);
    //AutoControl auto = new AutoControl(this);
    private EnhancedNavigation navigation;
    private double mod = 1;
    private double slow = 1;
    private boolean fieldCentric = false;
    private boolean blueTeam = true;
    private boolean goalFacingMode = false;
    private double shooterPower = 0;
    private double intakePower = 0;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadLeftState = false;
    private boolean goalFacingExecuted = false;
    private boolean lastBButtonState = false;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();


        // Initialize odometry
        odo = robot.odo;
        odo.setOffsets(-6.25, 168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Normal teleop control when not navigating to basket
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Rotate to point at coordinates when X button is pressed (skip normal drive)
            if (gamepad1.x) {
                navigation.rotateToPointAtCoordinates(TARGET_X, TARGET_Y, 1);
            } else {
                // Use goal-facing mode if enabled, otherwise use manual yaw control
                if (goalFacingMode && !goalFacingExecuted) {
                    robot.driveWithGoalFacing(axial, lateral, mod, 1.0);
                    goalFacingExecuted = true;
                } else if (!goalFacingMode) {
                    robot.controllerDrive(axial, lateral, yaw, mod);
                }
            }

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }

            if (gamepad1.left_stick_button) {
                fieldCentric = !robot.fieldCentric;
            }
            if (!gamepad1.left_stick_button) {
                robot.fieldCentric = fieldCentric;
            }

            if (gamepad1.a) {
                shooterPower = 0.7;
                robot.setShooterVelocity(shooterPower);
            }

            // Button B: Auto shoot sequence
            boolean currentBButton = gamepad1.b;
            if (currentBButton && !lastBButtonState) {
                performAutoShoot();
            }
            lastBButtonState = currentBButton;

            // Intake controlled by left trigger (forward)
            if (gamepad1.left_trigger > 0) {
                intakePower = -1;
            } else {
                intakePower = 0;
            }

            // Shootpush controlled by right trigger
            if (gamepad1.right_trigger > 0) {
                robot.shootpush.setPower(1);
            } else {
                robot.shootpush.setPower(0);
            }
            if (gamepad1.dpad_down) {
                if (blueTeam==true) {
                    navigation.navigateToPosition(1687.8, 1689.18, -50.8,1);
                    shooterPower = 0.6;
                }else{
                    navigation.navigateToPosition(1483, 2104, 46.48, 1);
                    shooterPower = 0.6;
                }
            }
            if (gamepad1.y) {
                if (blueTeam==true) {
                    navigation.navigateToPosition(667, 2268, -87.2, 1);
                    shooterPower = 0.63;
                }else{
                    navigation.navigateToPosition(564.14, 1338.2, 92,1);
                    shooterPower = 0.65;
                }
            }
            if (gamepad1.dpad_left) {
                blueTeam= true;
            }
            if (gamepad1.dpad_right) {
                blueTeam= false;
            }

            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumperState) {
                shooterPower = Math.min(1.0, shooterPower + 0.05);
                robot.setShooterVelocity(shooterPower);
            }
            lastRightBumperState = currentRightBumper;

            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumperState) {
                shooterPower = Math.max(0.0, shooterPower - 0.05);
                robot.setShooterVelocity(shooterPower);
            }
            lastLeftBumperState = currentLeftBumper;

//            boolean currentDpadUp = gamepad1.dpad_up;
//            if (currentDpadUp && !lastDpadUpState) {
//                odo.resetPosAndIMU();
//            }
//            lastDpadUpState = currentDpadUp;

            // Toggle goal-facing mode with dpad_left
//            boolean currentDpadLeft = gamepad1.dpad_left;
//            if (currentDpadLeft && !lastDpadLeftState) {
//                goalFacingMode = !goalFacingMode;
//                goalFacingExecuted = false; // Reset flag when toggling mode
//            }
//            lastDpadLeftState = currentDpadLeft;

            robot.setShooterVelocity(shooterPower);
            robot.intake.setPower(intakePower);

            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Calculate target heading for X button rotation
            double targetX = TARGET_X;
            double targetY = TARGET_Y;
            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            // Calculate base heading, add 180 degrees, normalize, then invert (same as rotate function)
            double headingGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
            headingGoal = headingGoal + 180.0;  // Add 180 to face opposite direction
            // Normalize to -180 to 180 range
            headingGoal = headingGoal % 360;
            if (headingGoal > 180) headingGoal -= 360;
            if (headingGoal < -180) headingGoal += 360;
            headingGoal = -headingGoal;  // Invert at the end

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            //telemetry.addData("Arm motor target", robot.armTarget);
            telemetry.addData("Robot Heading", "%.1f°", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Goal Heading", "%.1f°", Math.toDegrees(robot.calculateHeadingToGoal()));
            if (gamepad1.x) {
                telemetry.addData("HEADING GOAL", "%.1f°", headingGoal);
            }
            telemetry.addData("Speed", "%.1f°", shooterPower*100);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Goal Facing Mode", goalFacingMode ? "ON (dpad_left to toggle)" : "OFF (dpad_left to toggle)");
            
            // Show distance and calculated power for auto shoot
            double distanceToTarget = calculateDistanceToTarget();
            double calculatedPower = calculateShooterPower(distanceToTarget);
            telemetry.addData("Distance to Target", "%.1f mm (%.1f in)", distanceToTarget, distanceToTarget / 25.4);
            telemetry.addData("Calculated Power", "%.3f", calculatedPower);
            telemetry.addData("Shooter Power Set", "%.3f", shooterPower);
            telemetry.addData("Shooter Actual Velocity", "%.1f ticks/sec", robot.getShooterVelocity());
            
            telemetry.update();
        }

    }

    /**
     * Calculate the straight-line distance from robot to target coordinates
     * @return distance in mm
     */
    private double calculateDistanceToTarget() {
        odo.update();
        Pose2D pos = odo.getPosition();
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        
        double deltaX = TARGET_X - currentX;
        double deltaY = TARGET_Y - currentY;
        
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }


    /**
     * Calculate shooter power needed based on distance to target
     * Accounts for projectile motion with launch angle and net height
     * When close, needs more power due to steep angle required to clear height
     * @param distanceMM horizontal distance to target in mm
     * @return shooter power (0.0 to 1.0)
     */
    private double calculateShooterPower(double distanceMM) {
        // Convert to inches for easier calculation
        double horizontalDistanceInches = distanceMM / 25.4;
        double netHeightInches = NET_HEIGHT_INCHES;
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        
        // For projectile motion with launch angle θ and height h:
        // The required velocity increases significantly when close because:
        // - Short horizontal distance + high vertical height = very steep trajectory needed
        // - This requires more power than simple distance scaling
        
        // Calculate the angle needed to reach the target (ideal trajectory angle)
        // For close shots, this angle is much steeper than the launch angle
        double idealAngle = Math.atan2(netHeightInches, horizontalDistanceInches);
        
        // When close, the required angle is steeper than launch angle, requiring more power
        // Use a reduced factor to avoid over-compensating
        double angleFactor = 1.0;
        if (idealAngle > launchAngleRad) {
            // Need steeper angle than launch angle - increase power requirement (reduced multiplier)
            double angleDifference = idealAngle - launchAngleRad;
            angleFactor = 1.0 + angleDifference * 0.6;  // Reduced from 2.0 to 0.6
        }
        
        // Base power calculation using distance
        double minPower = 0.45;  // Slightly lower minimum (45%)
        double maxHorizontalDistance = 140.0;  // Increased max distance to scale better for far shots
        
        // Normalize distance
        double normalizedDistance = Math.min(1.0, horizontalDistanceInches / maxHorizontalDistance);
        
        // Reduced close distance boost
        double closeDistanceBoost = 0.0;
        if (horizontalDistanceInches < 30.0) {
            // Very close - reduced boost
            double closeFactor = (30.0 - horizontalDistanceInches) / 30.0;  // 0 to 1 as distance approaches 0
            closeDistanceBoost = closeFactor * 0.1;  // Reduced from 0.3 to 0.1
        }
        
        // Use a curve that's less aggressive at close, more aggressive at far
        // Square root for close distances, but use a power curve for far distances
        double distanceFactor;
        if (normalizedDistance < 0.5) {
            // Close distances: use square root (less aggressive)
            distanceFactor = Math.sqrt(normalizedDistance * 2.0) / Math.sqrt(2.0);
        } else {
            // Far distances: use quadratic curve (more aggressive scaling)
            double farNormalized = (normalizedDistance - 0.5) * 2.0;  // 0 to 1 for far half
            distanceFactor = 0.707 + (farNormalized * farNormalized) * 0.293;  // Quadratic scaling
        }
        
        double basePower = minPower + distanceFactor * (1.0 - minPower);
        
        // Apply reduced close distance boost and angle factor
        basePower = basePower + closeDistanceBoost;
        basePower = basePower * angleFactor;
        
        // Clamp base power before applying constants (reduced max from 1.2 to 1.1)
        basePower = Math.max(0.45, Math.min(1.1, basePower));
        
        // Apply the shoot power constant
        double calculatedPower = basePower * SHOOT_POWER_CONSTANT;
        
        // Clamp to valid range
        return Math.max(0.0, Math.min(1.0, calculatedPower));
    }

    /**
     * Perform automatic shooting sequence:
     * 1. Calculate distance to target
     * 2. Calculate required shooter power
     * 3. Set shooter power
     * 4. Wait for motor to reach speed
     * 5. Activate shootpush to fire
     */
    private void performAutoShoot() {
        // Calculate distance to target
        double distance = calculateDistanceToTarget();
        
        // Calculate required shooter power
        double targetPower = calculateShooterPower(distance);
        
        // Get current shooter power
        double currentPower = shooterPower;
        
        // Calculate wait time based on speed change
        // Account for direction (increasing vs decreasing) and current power level
        double deltaPower = Math.abs(targetPower - currentPower);
        boolean isDecreasing = targetPower < currentPower;
        boolean isIncreasing = targetPower > currentPower;
        
        // Base wait time
        double baseWaitTime = deltaPower * WAIT_TIME_CONSTANT;
        
        // When decreasing speed, motors take longer to slow down (more inertia at higher speeds)
        // When increasing, also account for current power level (higher current power = more inertia)
        double directionFactor = 1.0;
        if (isDecreasing) {
            // Slowing down: takes longer, especially from high speeds
            directionFactor = 1.5 + (currentPower * 0.5);  // 1.5x to 2.0x longer when decreasing
        } else if (isIncreasing) {
            // Speeding up: account for current power level (inertia)
            directionFactor = 1.0 + (currentPower * 0.3);  // 1.0x to 1.3x based on current speed
        }
        
        // Calculate final wait time with all factors
        long waitTimeMs = (long)(baseWaitTime * directionFactor * 1000);
        
        // Set shooter to target power immediately using velocity control
        shooterPower = targetPower;
        robot.setShooterVelocity(shooterPower);
        
        // Wait for motor to reach speed
        if (waitTimeMs > 0) {
            sleep(waitTimeMs);
        }
        
        // Activate shootpush to fire the ball for 1 second
        robot.shootpush.setPower(1.0);
        sleep(1000);  // Push for 1 second
        robot.shootpush.setPower(0.0);
        
        // Update telemetry
        telemetry.addData("Auto Shoot", "Fired!");
        telemetry.addData("Distance", "%.1f mm (%.1f in)", distance, distance / 25.4);
        telemetry.addData("Shooter Power", "%.2f", targetPower);
        telemetry.addData("Wait Time", "%.2f s", deltaPower * WAIT_TIME_CONSTANT);
        telemetry.update();
    }
}