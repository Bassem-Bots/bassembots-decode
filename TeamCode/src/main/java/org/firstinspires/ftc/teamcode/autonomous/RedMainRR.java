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

@Autonomous(name = "RedMainRR", group = "Competition")
public class RedMainRR extends LinearOpMode {

    private RobotControl robot;
    private MecanumDrive drive;

    // Coordinate Conversion: RR = Pedro - 72
    // Start: (135, 100) -> (63, 28)
    private final Pose2d startPose = new Pose2d(63, 28, Math.toRadians(0));

    // Shoot: (132, 93) -> (60, 21)
    private final Pose2d shootPose = new Pose2d(60, 21, Math.toRadians(-25));

    // RedSquare: (111, 45) -> (39, -27)
    private final Pose2d redSquare = new Pose2d(20, 59, Math.toRadians(0));

    // Pickup: (111, 28) -> (39, -44)
    private final Pose2d pickupPose = new Pose2d(61, 59, Math.toRadians(0));

    // Final: (111, 38) -> (39, -34)
    private final Pose2d finalPose = new Pose2d(39, 34, Math.toRadians(0));

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
                // This calls the blocking method from RobotControl
                // Using a blocking call inside Action.run() suspends the loop, but for shooting
                // while stopped it is acceptable.
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
            return false; // Done
        }
    }

    // Subsystem Actions
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

    public Action waitAction(long ms) {
        return packet -> {
            safeSleep(ms);
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
        // Initialize Hardware
        // NOTE: MecanumDrive initializes drive motors. RobotControl also initializes
        // them.
        // We initialize RobotControl first to set up everything, then MecanumDrive
        // might re-init drive motors.
        // This overlap is managed by ensuring we don't use RobotControl for driving.
        robot = new RobotControl(this);
        robot.init(); // Sets up intake, shooter, etc.

        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Status", "Initialized - RR RedMain");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

        // Build Paths

        // Path 1: Start -> Shoot
        Action trajectoryStartToShoot = drive.actionBuilder(startPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build();

        // Path 2: Shoot -> RedSquare
        // Note: shootPose heading is -108, RedSquare is 180.
        Action trajectoryShootToRed = drive.actionBuilder(shootPose)
                .strafeToLinearHeading(redSquare.position, redSquare.heading)
                .build();

        // Path 3: RedSquare -> Pickup
        Action trajectoryRedToPickup = drive.actionBuilder(redSquare)
                .strafeToLinearHeading(pickupPose.position, pickupPose.heading)
                .build();

        // Path 4: Pickup -> Shoot (Back to shootPose)
        Action trajectoryPickupToShoot = drive.actionBuilder(pickupPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build();

        // Path 5: Shoot -> Final
        Action trajectoryShootToFinal = drive.actionBuilder(shootPose)
                .strafeToLinearHeading(finalPose.position, finalPose.heading)
                .build();

        // Execute Actions
        // Based on redMain.java active code + commented out suggestions

        // Phase 1 & 2: Drive to Shoot and Fire 3
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryStartToShoot,
                        new FireShotsAction(3, 0.775f)));

        // Uncomment below to enable the full cycle (Logic from redMain commented
        // sections
         Actions.runBlocking(
         new SequentialAction(
         trajectoryShootToRed,
         intakeOn(),
         trajectoryRedToPickup,
         intakeOff(), // ensure intake is handled (redMain logic was mixed)
         trajectoryPickupToShoot,
         new FireShotsAction(2, 0.79f),
         trajectoryShootToFinal
         )
         );

        // Final sleep as per redMain active code
        sleep(9000);

        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }
}
