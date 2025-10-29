package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Locale;

@TeleOp(name="Main Code", group="Use this")
public class Code_Z extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControl robot = new RobotControl(this);
    //AutoControl auto = new AutoControl(this);
    private EnhancedNavigation navigation;
    private double mod = 1;
    private double slow = 1;
    private boolean fieldCentric = false;
    private double shooterPower = 0;
    private double intakePower = 0;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();
        
        // Initialize odometry
        odo = robot.odo;
        odo.setOffsets(6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Normal teleop control when not navigating to basket
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            robot.controllerDrive(axial, lateral, yaw, mod);

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }

            if (gamepad1.left_stick_button) {
                fieldCentric = (robot.fieldCentric) ? false : true;
            }
            if (!gamepad1.left_stick_button) {
                robot.fieldCentric = driveSetting
            }

            if (gamepad1.a) {
                shooterPower = 1;
            }

            if (gamepad1.b) {
                intakePower = 1;
            }

            if (gamepad1.x) {
                intakePower = -1;
            }

            if (gamepad1.y) {
                intakePower = 0;
            }

            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumperState) {
                shooterPower = Math.min(1.0, shooterPower + 0.1);
            }
            lastRightBumperState = currentRightBumper;

            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumperState) {
                shooterPower = Math.max(-1.0, shooterPower - 0.1);
            }
            lastLeftBumperState = currentLeftBumper;

            robot.shooter.setPower(shooterPower)
            robot.intake.setPower(intakePower)

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
            telemetry.addData("near basker", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("editing", odo.getHeading());
            if (isNavigating) {
                telemetry.addData("Navigation", "Moving to Basket");
            }
            
            // AprilTag telemetry
            displayAprilTagTelemetry();
            
            telemetry.update();
        }
    
    }
}