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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotControl;
import org.firstinspires.ftc.teamcode.util.roadRunner.MecanumDrive;

@Autonomous(name = "RedMainRR", group = "888888")
public class RedMainRR extends LinearOpMode {

    private RobotControl robot;
    private MecanumDrive drive;
    private ElapsedTime timer = new ElapsedTime();
    private double shooterPower = 0;

    // Coordinate Conversion: RR = Pedro - 72
    // Start: (135, 100) -> (63, 28)
    private final Pose2d startPose = new Pose2d(63, 28, Math.toRadians(0));

    // Shoot: (132, 93) -> (60, 21)
    private final Pose2d shootPose = new Pose2d(55, 21, Math.toRadians(-25.5));

    // RedSquare: (111, 45) -> (39, -27)
    private final Pose2d redSquare = new Pose2d(20, 57.7, Math.toRadians(0));

    // Pickup: (111, 28) -> (39, -44)
    private final Pose2d pickupPose = new Pose2d(62, 57.7, Math.toRadians(0));

    // Final: (111, 38) -> (39, -34)
    private final Pose2d finalPose = new Pose2d(39, 34, Math.toRadians(0));

    private final Pose2d humanPose = new Pose2d(54, 60, Math.toRadians(56.7));
    private final Pose2d humanPoset = new Pose2d(69, 63, Math.toRadians(62.7));

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
                shooterPower = power;
                safeSleep(1400); // Spin up

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

//                    if (i == 0) {
//                        safeSleep(200);
//                    }
                    if (i != shots - 1)
                        safeSleep(750);
                }
                robot.shooter.setPower(0);
                shooterPower = 0;
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
            timer.reset();
            while (timer.milliseconds() < ms) {
                robot.setShooterPower(shooterPower);
                telemetry.addData("shooterpower", shooterPower);
                telemetry.addData("power velocirty", robot.powerToVelocity(shooterPower));
                telemetry.addData("actual velocuirty", robot.getShooterVelocity());
                telemetry.update();
            }
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

        Action trajectoryShootToHuman = drive.actionBuilder(shootPose)
                .strafeToLinearHeading(humanPose.position, humanPose.heading)
                        .build();

        Action trajectoryHumanToShoot = drive.actionBuilder(humanPoset)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                        .build();

        Action trajectoryHumant = drive.actionBuilder(humanPose)
                .strafeToLinearHeading(humanPoset.position, humanPoset.heading)
                        .build();

        // Execute Actions
        // Based on redMain.java active code + commented out suggestions

        // Phase 1 & 2: Drive to Shoot and Fire 3
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryStartToShoot,
                        new FireShotsAction(3, 0.69f)));

        // Uncomment below to enable the full cycle (Logic from redMain commented
        // sections
         Actions.runBlocking(
         new SequentialAction(
         trajectoryShootToRed,
         intakeOn(),
         trajectoryRedToPickup,
         intakeOff(), // ensure intake is handled (redMain logic was mixed)
         trajectoryPickupToShoot,
         new FireShotsAction(2, 0.69f),
         intakeOn(),
         trajectoryShootToHuman,
         trajectoryHumant,
         waitAction(800),
         intakeOff(),
         trajectoryHumanToShoot,
         //intakeOff(),
         new FireShotsAction(2, 0.69f),
         trajectoryShootToFinal
         )
         );

        // Final sleep as per redMain active code
        sleep(9000);

        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }
}
