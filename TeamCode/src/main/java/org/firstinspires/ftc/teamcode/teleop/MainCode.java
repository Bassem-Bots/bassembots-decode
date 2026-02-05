package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.util.RobotControl;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.EnhancedNavigation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name="Main Code - Dec 6", group="Usethis")
public class MainCode extends LinearOpMode {
    // Target coordinates for shooting (dynamic based on team)
    // Blue team: (615, 700)
    // Red team: (800, 3257)
    private static final double NET_HEIGHT_INCHES = 37.0;  // inches
    private static final double NET_HEIGHT_MM = NET_HEIGHT_INCHES * 25.4;  // Convert to mm
    private static final double LAUNCH_ANGLE_DEG = 65.0;  // Launch angle of shooter (degrees, steeper than 45)
    
    // Tuning constants - Adjusted based on Round 3 data analysis
    private static final double WAIT_TIME_CONSTANT = 8;  // seconds per unit of power (4-5 seconds from 0-1)
    private static final double SHOOT_POWER_CONSTANT = 0.80;  // Adjusted for better consistency (was 0.78)
    private static final double REFERENCE_VOLTAGE = 12.5;  // Reference voltage for compensation
    private static final double VOLTAGE_COMPENSATION_FACTOR = 0.8;  // Voltage impact factor (reduced power at higher voltage)
    private static final double VOLTAGE_CHANGE_THRESHOLD = 0.2;  // Voltage change (volts) that triggers re-adjustment before shooting
    
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControl robot = new RobotControl(this);
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
            // Also calculate and set the correct shooter power for that distance
            if (gamepad1.x) {
                navigation.rotateToPointAtCoordinates(getTargetX(), getTargetY(), 1);
                // Calculate distance and set shooter power
                double distance = calculateDistanceToTarget();
                shooterPower = calculateShooterPower(distance);
                robot.shooter.setPower(shooterPower);
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
            }

            // Button B: Intake reverse (opposite of left trigger)
            // Left trigger = forward (intakePower = -1), B = backward (intakePower = 1)
            if (gamepad1.b) {
                intakePower = 1;  // Reverse/backward (opposite of left trigger)
            } else if (gamepad1.left_trigger > 0) {
                intakePower = -1;  // Forward
            } else {
                intakePower = 0;
            }

            // ARCHIVED: Button B Auto shoot sequence (commented out)
            /*
            boolean currentBButton = gamepad1.b;
            if (currentBButton && !lastBButtonState) {
                performAutoShoot();
            }
            lastBButtonState = currentBButton;
            */

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
            }
            lastRightBumperState = currentRightBumper;

            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumperState) {
                shooterPower = Math.max(-1.0, shooterPower - 0.05);
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

            robot.shooter.setPower(shooterPower);
            robot.intake.setPower(intakePower);

            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Calculate target heading for X button rotation
            double targetX = getTargetX();
            double targetY = getTargetY();
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
            double batteryVoltage = getBatteryVoltage();
            telemetry.addData("Distance to Target", "%.1f mm (%.1f in)", distanceToTarget, distanceToTarget / 25.4);
            telemetry.addData("Calculated Power", "%.3f", calculatedPower);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            
            telemetry.update();
        }

    }

    /**
     * Get target X coordinate based on team
     * @return target X in mm
     */
    private double getTargetX() {
        return blueTeam ? 750 : 800.0;
    }

    /**
     * Get target Y coordinate based on team
     * @return target Y in mm
     */
    private double getTargetY() {
        return blueTeam ? 550.0 : 3257.0;
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
        
        double deltaX = getTargetX() - currentX;
        double deltaY = getTargetY() - currentY;
        
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Get the current battery voltage
     * @return battery voltage in volts, or REFERENCE_VOLTAGE if sensor not available
     */
    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        // If no voltage sensor found, return reference voltage (assume fully charged)
        return (minVoltage == Double.POSITIVE_INFINITY) ? REFERENCE_VOLTAGE : minVoltage;
    }

    /**
     * Calculate shooter power needed based on distance to target
     * Tuned based on Round 3 data - improved consistency across all ranges
     * @param distanceMM horizontal distance to target in mm
     * @return shooter power (0.0 to 1.0) - returns as percentage (0.0-1.0), multiply by 100 for display
     */
    private double calculateShooterPower(double distanceMM) {
        // Convert to inches for easier calculation
        double horizontalDistanceInches = distanceMM / 25.4;
        
        // Round 3 data analysis:
        // Distance range: 58-130 inches
        // Power range: 58-84% (0.58-0.84)
        // Key findings:
        // - Far distances (120+ in) need more power than current formula
        // - Higher voltages (13+ V) need less power (motors run faster)
        // - Need smoother, more consistent curve
        
        // Power curve based on Round 3 data points:
        // 58-60in: 58-60% power
        // 65-70in: 60-66% power  
        // 70-75in: 66-68% power
        // 75-85in: 68-72% power
        // 85-100in: 72-75% power
        // 100-115in: 75-78% power
        // 115-130in: 78-84% power
        
        double basePower;
        if (horizontalDistanceInches < 50.0) {
            // Very close: 58-60% final → base 0.725-0.750
            double closeFactor = horizontalDistanceInches / 50.0;
            basePower = 0.725 + closeFactor * 0.025;  // 0.725 to 0.750
        } else if (horizontalDistanceInches < 70.0) {
            // Close-medium: 60-68% final → base 0.750-0.850
            double midFactor = (horizontalDistanceInches - 50.0) / 20.0;
            basePower = 0.750 + midFactor * 0.100;  // 0.750 to 0.850
        } else if (horizontalDistanceInches < 85.0) {
            // Medium: 68-72% final → base 0.850-0.900
            double midFactor = (horizontalDistanceInches - 70.0) / 15.0;
            basePower = 0.850 + midFactor * 0.050;  // 0.850 to 0.900
        } else if (horizontalDistanceInches < 100.0) {
            // Medium-far: 72-75% final → base 0.900-0.938
            double midFarFactor = (horizontalDistanceInches - 85.0) / 15.0;
            basePower = 0.900 + midFarFactor * 0.038;  // 0.900 to 0.938
        } else if (horizontalDistanceInches < 115.0) {
            // Far: 75-78% final → base 0.938-0.975
            double farFactor = (horizontalDistanceInches - 100.0) / 15.0;
            basePower = 0.938 + farFactor * 0.037;  // 0.938 to 0.975
        } else {
            // Very far: 78-84% final → base 0.975-1.050
            // Steeper increase for very far shots
            double veryFarFactor = (horizontalDistanceInches - 115.0) / 20.0;
            basePower = 0.975 + Math.min(1.0, veryFarFactor) * 0.075;  // 0.975 to 1.050
        }
        
        // Clamp base power
        basePower = Math.max(0.72, Math.min(1.05, basePower));
        
        // Apply the shoot power constant
        double calculatedPower = basePower * SHOOT_POWER_CONSTANT;
        
        // Apply voltage compensation
        // Higher voltage = motors run faster = need less power
        // Lower voltage = motors run slower = need more power
        double batteryVoltage = getBatteryVoltage();
        double voltageRatio = REFERENCE_VOLTAGE / batteryVoltage;
        
        // Apply compensation factor
        // At high voltage (13.5V): ratio = 12.5/13.5 = 0.926, need to reduce power more
        // At low voltage (12.0V): ratio = 12.5/12.0 = 1.042, need to increase power
        voltageRatio = 1.0 + (voltageRatio - 1.0) * VOLTAGE_COMPENSATION_FACTOR;
        
        // Clamp to reasonable range - allow more reduction at high voltage
        voltageRatio = Math.max(0.88, Math.min(1.10, voltageRatio));
        calculatedPower = calculatedPower * voltageRatio;
        
        // Clamp to valid range
        return Math.max(0.0, Math.min(1.0, calculatedPower));
    }

    /**
     * ARCHIVED: Perform automatic shooting sequence:
     * 1. Calculate distance to target
     * 2. Calculate required shooter power
     * 3. Set shooter power
     * 4. Wait for motor to reach speed
     * 5. Activate shootpush to fire
     */
    /*
    private void performAutoShoot() {
        // Calculate distance to target
        double distance = calculateDistanceToTarget();
        
        // Calculate required shooter power
        double targetPower = calculateShooterPower(distance);
        
        // Get current shooter power
        double currentPower = shooterPower;
        
        // Get initial battery voltage for wait time compensation
        double initialBatteryVoltage = getBatteryVoltage();
        double initialVoltageRatio = REFERENCE_VOLTAGE / initialBatteryVoltage;
        initialVoltageRatio = 1.0 + (initialVoltageRatio - 1.0) * VOLTAGE_COMPENSATION_FACTOR;
        initialVoltageRatio = Math.max(0.7, Math.min(1.5, initialVoltageRatio));  // Clamp to reasonable range
        
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
        long waitTimeMs = (long)(baseWaitTime * directionFactor * initialVoltageRatio * 1000);
        
        // Set shooter to target power immediately
        shooterPower = targetPower;
        robot.shooter.setPower(shooterPower);
        
        // Wait for motor to reach speed
        if (waitTimeMs > 0) {
            sleep(waitTimeMs);
        }
        
        // Second voltage check before shooting - adjust if voltage changed significantly
        double currentBatteryVoltage = getBatteryVoltage();
        double voltageChange = Math.abs(currentBatteryVoltage - initialBatteryVoltage);
        
        if (voltageChange > VOLTAGE_CHANGE_THRESHOLD) {
            // Voltage changed significantly, recalculate and adjust
            double newVoltageRatio = REFERENCE_VOLTAGE / currentBatteryVoltage;
            newVoltageRatio = 1.0 + (newVoltageRatio - 1.0) * VOLTAGE_COMPENSATION_FACTOR;
            newVoltageRatio = Math.max(0.7, Math.min(1.5, newVoltageRatio));
            
            // Recalculate power with new voltage
            double basePower = calculateShooterPower(distance);  // This will use current voltage
            double adjustedPower = basePower;
            
            // If power needs to change significantly, adjust and wait again
            double powerAdjustment = Math.abs(adjustedPower - shooterPower);
            if (powerAdjustment > 0.05) {  // If change is more than 5%
                shooterPower = adjustedPower;
                robot.shooter.setPower(shooterPower);
                
                // Wait additional time for new power to stabilize
                long additionalWaitMs = (long)(powerAdjustment * WAIT_TIME_CONSTANT * newVoltageRatio * 1000);
                if (additionalWaitMs > 0) {
                    sleep(additionalWaitMs);
                }
            }
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
    */
}