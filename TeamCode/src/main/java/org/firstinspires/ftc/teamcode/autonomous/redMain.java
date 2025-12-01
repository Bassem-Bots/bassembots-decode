package org.firstinspires.ftc.teamcode.autonomous; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.RobotControl;
import org.firstinspires.ftc.teamcode.util.pedroPathing.Constants;

@Autonomous(name = "RedMain", group = "Competition")
public class redMain extends LinearOpMode {

    private Follower follower;
    private RobotControl robot;

    // --------------------------------------------------------
    // COORDINATES (Blue Alliance, Positive Y)
    // --------------------------------------------------------

    // START: Assuming you start near the Backboard to shoot quickly
    private final Pose startPose = new Pose(135, 100 , Math.toRadians(0));
//    private final Pose startPrimePose = new Pose(134, 93, Math.toRadians(0));
    private final Pose finalPose = new Pose(144-33, 38, Math.toRadians(0));
    private final Pose redSquare = new Pose(144-33,45, -Math.toRadians(180));
    private final Pose pickupPose = new Pose(144-33, 28, -Math.toRadians(180));

    // SHOOT: Ideally this is where your shooter aims best at the goal
    private final Pose shootPose = new Pose(132, 93, Math.toRadians(-108));
    private final Pose shootPose2 = new Pose(-120, -36, Math.toDegrees(-(-140+-25)));

    // PICKUP: Location of the balls on the floor (Spike Mark or Stack)
    // Adjusted to be slightly away so we can drive *through* it or stop *at* it

    // Paths
    private Path startToShoot ;
    private PathChain shootToPickup, pickupToShoot, shootToStart, shootToRed, redToPickup, shootToFinal;

    public void buildPaths() {
        // Path 1: Drive from Start to Shooting Position
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

//        startToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
//                .setReversed()
//                .build();

        shootToRed = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, redSquare))
                .setLinearHeadingInterpolation(shootPose.getHeading(), redSquare.getHeading())
                .build();

        redToPickup = follower.pathBuilder()
                .addPath(new BezierLine(redSquare, pickupPose))
                .setLinearHeadingInterpolation(redSquare.getHeading(), pickupPose.getHeading())
                .build();

        pickupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFinal = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading())
                .build();

////        // Path 3: Drive from Pickup back to Shoot
//        pickupToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPose, shootPose2))
//                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose2.getHeading())
//                .build();
//
//        shootToStart = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose2, finalPose))
//                .setLinearHeadingInterpolation(shootPose2.getHeading(), finalPose.getHeading())
//                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Hardware
        robot = new RobotControl(this);
        robot.init();

        // 2. Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized - Holding 3 Balls");
        telemetry.update();

        waitForStart();

        // ----------------------------------------------------
        // PHASE 1: GO TO SHOOTING SPOT
        // ----------------------------------------------------
        follower.followPath(startToShoot);
        driveUntilDone("Driving to Shoot");

        // ----------------------------------------------------
        // PHASE 2: SHOOT 3 PRELOADED BALLS
        // ----------------------------------------------------
        if (opModeIsActive()) {
            telemetry.addData("Action", "Shooting 3 Balls");
            telemetry.update();

//             Fire 3 shots using our helper method
            fireShots(3, .697f);
        }

//        robot.intake.setPower(1);
//        sleep(600);
//        follower.followPath(shootToRed);
//        driveUntilDone("Driving to pickup");
//        sleep(1000);
//
//        follower.followPath(redToPickup);
//        driveUntilDone("Driving to pickup");
//        robot.intake.setPower(0);
//        follower.followPath(pickupToShoot);
//        driveUntilDone("Driving to shoot again");
//
//        if (opModeIsActive()) {
//            telemetry.addData("Action", "Shooting 2 Balls");
//            telemetry.update();
//
//             Fire 3 shots using our helper method
//            fireShots(2, .725f);
//        }
        robot.shooter.setPower(0);
        sleep(9000);
        // ----------------------------------------------------
        // PHASE 3: GO TO Final
        // ----------------------------------------------------
//        follower.followPath(shootToFinal);
//        driveUntilDone("Final position");

        // Start intake BEFORE we start moving, or right as we start
//        robot.intake.setPower(-1); // Turn on intake
//
//        follower.followPath(shootToPickup, true);
//        driveUntilDone("Driving to Pickup");
//
//        // We are at the pickup spot now.
//        // Optional: Move forward slightly or wait to ensure we grabbed the ball
//        sleep(500); // Give intake time to grab the ball
//        robot.intake.setPower(0); // Stop intake (don't want to carry it running?)
//
//        // ----------------------------------------------------
//        // PHASE 4: GO BACK TO SHOOT
//        // ----------------------------------------------------
//        follower.followPath(pickupToShoot,true);
//        driveUntilDone("Returning to Shoot");
//
//        // ----------------------------------------------------
//        // PHASE 5: SHOOT AGAIN
//        // ----------------------------------------------------
//        if (opModeIsActive()) {
//            telemetry.addData("Action", "Shooting Collected Ball");
//            telemetry.update();
//
//            // Fire 1 shot (or however many we picked up)
//            fireShots(1,.57f);
//        }
//
//        // ----------------------------------------------------
//        // PHASE 6: Go to start
//        // ----------------------------------------------------
//        follower.followPath(shootToStart,true);
//        driveUntilDone("Returning to Start");

        // End of Auto - Stop everything
        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }

    /**
     * Helper method to keep the main loop clean.
     * Handles spinning up the shooter and pushing the balls.
     */
    private void fireShots(int numberOfShots, float power) {
        // 1. Spin up the shooter
        robot.shooter.setPower(power);
        telemetry.addData("fsrdgth", robot.shooter.getPower());

        sleep(4000); // Wait for flywheel to get to speed

        // 2. Cycle the pusher for each ball

        for (int i = 0; i < numberOfShots; i++) {
            if (!opModeIsActive()) break;

            // Push ball in

            if(i ==numberOfShots-1)
            {
                robot.intake.setPower(-1.0);
            }
            robot.shootpush.setPower(1.0);
            sleep(300);
            if (i >0){
                sleep(600);

            }

            // If your pusher needs to reverse to retract, change 0 to -1.0
            robot.intake.setPower(0);
            robot.shootpush.setPower(0);
            if (i == 0){
                sleep(200);
            }
            if (i!=numberOfShots-1)sleep(4000);
            // Wait for gravity to load the next ball
        }

        // 3. Stop the shooter after all shots

    }


    /**
     * Helper method to block code execution while the robot drives.
     * Keeps the Follower updating.
     */
    private void driveUntilDone(String statusMessage) {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Status", statusMessage);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }
}
