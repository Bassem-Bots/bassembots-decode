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
    private static final double NET_HEIGHT_INCHES = 35;  // inches
    private static final double NET_HEIGHT_MM = NET_HEIGHT_INCHES * 25.4;  // Convert to mm
    private static final double LAUNCH_ANGLE_DEG = 65.0; //45 to 70  // Launch angle of shooter (degrees, steeper than 45)
    
    // Tuning constants - Adjusted based on Round 3 data analysis
    private static final double WAIT_TIME_CONSTANT = 8;  // seconds per unit of power (4-5 seconds from 0-1)
    private static final double SHOOT_POWER_CONSTANT = 0.50;  // Adjusted for better consistency (was 0.78)
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
    private double angleExtenderPosition = 0.0;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadLeftState = false;
    private boolean goalFacingExecuted = false;
    private boolean lastBButtonState = false;
    private boolean lastYButtonState = false;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();

        // Initialize angle extender position from current servo state
        angleExtenderPosition = robot.getAngleExtenderPosition();

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
            // Rotate to point at coordinates when X button is pressed (skip normal drive)
            // Also calculate and set the correct shooter power for that distance
            if (gamepad1.x) {
                navigation.rotateToPointAtCoordinates(getTargetX(), getTargetY(), 1);
                // Calculate distance and set shooter power & angle
                double distance = calculateDistanceToTarget();
                RobotControl.ShootingSolution solution = robot.calculateShootingSolution(distance, NET_HEIGHT_MM);
                
                shooterPower = solution.power;
                robot.setShooterVelocity(shooterPower);
                robot.setLauncherAngle(solution.angleDeg);
                
                // Sync local variable
                angleExtenderPosition = robot.getAngleExtenderPosition();
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

            // Shootpush controlled by right trigger
            if (gamepad1.right_trigger > 0) {
                robot.shootpush.setPower(1);
            } else {
                robot.shootpush.setPower(0);
            }
            
            // Preset positions (DPad/Y) override auto-aim temporarily
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

            // D-pad up: increase angle extender servo position
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !lastDpadUpState) {
                angleExtenderPosition = Math.min(1.0, angleExtenderPosition + 0.1);
                robot.setAngleExtenderPosition(angleExtenderPosition);
            }
            lastDpadUpState = currentDpadUp;

            // Start button: decrease angle extender servo position
            boolean currentStartButton = gamepad1.start;
            if (currentStartButton && !lastYButtonState) {
                angleExtenderPosition = Math.max(0.0, angleExtenderPosition - 0.1);
                robot.setAngleExtenderPosition(angleExtenderPosition);
            }
            lastYButtonState = currentStartButton;

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
            double targetX = getTargetX();
            double targetY = getTargetY();
            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double headingGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
            headingGoal = headingGoal + 180.0;
            headingGoal = headingGoal % 360;
            if (headingGoal > 180) headingGoal -= 360;
            if (headingGoal < -180) headingGoal += 360;
            headingGoal = -headingGoal;

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Robot Heading", "%.1f°", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Goal Heading", "%.1f°", Math.toDegrees(robot.calculateHeadingToGoal()));
            if (gamepad1.x) {
                telemetry.addData("HEADING GOAL", "%.1f°", headingGoal);
            }
            telemetry.addData("Speed", "%.1f%%", shooterPower*100);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Goal Facing Mode", goalFacingMode ? "ON" : "OFF");
            telemetry.addData("Angle Extender Pos", "%.2f", angleExtenderPosition);
            
            // Show distance and calculated solution
            double distanceToTarget = calculateDistanceToTarget();
            RobotControl.ShootingSolution sol = robot.calculateShootingSolution(distanceToTarget, NET_HEIGHT_MM);
            double batteryVoltage = getBatteryVoltage();
            
            telemetry.addData("Dist", "%.0f mm", distanceToTarget);
            telemetry.addData("Auto Calc", "P: %.2f, A: %.1f°", sol.power, sol.angleDeg);
            telemetry.addData("Batt", "%.2f V", batteryVoltage);
            telemetry.addData("Act Vel", "%.0f", robot.shooter.getVelocity());
            
            telemetry.update();
        }

    }

    /**
     * Get target X coordinate based on team
     * @return target X in mm
     */
    private double getTargetX() {
        return blueTeam ? 545 : 750.0;
    }

    /**
     * Get target Y coordinate based on team
     * @return target Y in mm
     */
    private double getTargetY() {
        return blueTeam ? 217 : 3257.0;
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
     * Deprecated: Use robot.calculateShootingSolution instead.
     * Kept for compatibility if referenced elsewhere.
     */
    private double calculateShooterPower(double distanceMM) {
        RobotControl.ShootingSolution sol = robot.calculateShootingSolution(distanceMM, NET_HEIGHT_MM);
        return sol.power;
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