package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.util.RobotControl;
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

@TeleOp(name="Main Code", group="Usethis")
public class MainCode extends LinearOpMode {
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

            // Use goal-facing mode if enabled, otherwise use manual yaw control
            if (goalFacingMode && !goalFacingExecuted) {
                robot.driveWithGoalFacing(axial, lateral, mod, 1.0);
                goalFacingExecuted = true;
            } else if (!goalFacingMode) {
                robot.controllerDrive(axial, lateral, yaw, mod);
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

            // Intake controlled by left trigger (forward) or X button (backward)
            if (gamepad1.x) {
                intakePower = 1;
            } else if (gamepad1.left_trigger > 0) {
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

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            //telemetry.addData("Arm motor target", robot.armTarget);
            telemetry.addData("Robot Heading", "%.1f°", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Goal Heading", "%.1f°", Math.toDegrees(robot.calculateHeadingToGoal()));
            telemetry.addData("Speed", "%.1f°", shooterPower*100);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Goal Facing Mode", goalFacingMode ? "ON (dpad_left to toggle)" : "OFF (dpad_left to toggle)");
            telemetry.update();
        }

    }
}