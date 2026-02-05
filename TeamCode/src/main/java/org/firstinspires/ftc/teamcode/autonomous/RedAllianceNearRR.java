package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.RobotControl;
import org.firstinspires.ftc.teamcode.util.roadRunner.MecanumDrive;

@Autonomous(name = "RedAllianceNearRR", group = "Competition")
public class RedAllianceNearRR extends LinearOpMode {

    private RobotControl robot;
    private MecanumDrive drive;

    // Coordinate Conversion: RR = Pedro - 72
    // Start: (135, 100) -> (63, 28)
    private final Pose2d startPose = new Pose2d(63, 28, Math.toRadians(0));

    // Shoot: (132, 93) -> (60, 21)
    // Heading: Pedro -130 deg.
    // RedMainRR used -25 deg for Pedro -108 deg. Difference is +83 deg.
    // Converting -130 + 83 = -47 deg.
    private final Pose2d shootPose = new Pose2d(60, 21, Math.toRadians(-47));

    // Final: (111, 38) -> (39, -34)
    private final Pose2d finalPose = new Pose2d(39, -34, Math.toRadians(0));

    public class FireShotsAction implements Action {
        private final int shots;
        private final float power;
        private boolean executed = false;

        public FireShotsAction(int shots, float power) {
            this.shots = shots;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!executed) {
                robot.shooter.setPower(power);
                safeSleep(3000); // Spin up

                for (int i = 0; i < shots; i++) {
                    if (!opModeIsActive())
                        break;

                    if (i == shots - 1) {
                        robot.intake.setPower(-1.0);
                    }
                    robot.shootpush.setPower(1.0);
                    safeSleep(500);

                    if (i > 0) {
                        safeSleep(600);
                    }

                    robot.intake.setPower(0);
                    robot.shootpush.setPower(0);

                    if (i == 0) {
                        safeSleep(200);
                    }
                    if (i != shots - 1)
                        safeSleep(1500);
                }
                robot.shooter.setPower(0);
                executed = true;
            }
            return false;
        }
    }

    public Action intakeOn() {
        return packet -> {
            robot.intake.setPower(-1);
            return false;
        };
    }

    public Action intakeOff() {
        return packet -> {
            robot.intake.setPower(0);
            return false;
        };
    }

    private void safeSleep(long ms) {
        try {
            sleep(ms);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        robot.init();

        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Status", "Initialized - RR RedAllianceNear");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

        // Path 1: Start -> Shoot
        Action trajectoryStartToShoot = drive.actionBuilder(startPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build();

        // Path 2: Shoot -> Final
        Action trajectoryShootToFinal = drive.actionBuilder(shootPose)
                .strafeToLinearHeading(finalPose.position, finalPose.heading)
                .build();

        // Execute Actions
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryStartToShoot,
                        // Power 0.72f from redAllianceNear.java
                        new FireShotsAction(3, 0.72f)));

        sleep(9000);

        Actions.runBlocking(
                trajectoryShootToFinal);

        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }
}
