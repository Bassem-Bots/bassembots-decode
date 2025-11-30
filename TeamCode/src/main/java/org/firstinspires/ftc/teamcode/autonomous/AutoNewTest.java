package org.firstinspires.ftc.teamcode.autonomous; // make sure this aligns with class location
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//
//
//@Autonomous(name = "Example Auto", group = "Examples")
//public class AutoNewTest extends OpMode {
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    private final Pose startPose = new Pose(24, 120, Math.toRadians(270)); // Start Pose of our robot.
//    private final Pose scorePose = new Pose(24, 130, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose pickup1Pose = new Pose(30, 90, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(36, 96, Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(42, 90, Math.toRadians(270)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
//
//    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//    /* Here is an example for Constant Interpolation
//    scorePreload.setConstantInterpolation(startPose.getHeading()); */
//
//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3, true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    /**
//     * These change the states of the paths and actions. It will also reset the timers of the individual switches
//     **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//    /**
//     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
//     **/
//    @Override
//    public void loop() {
//
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//}
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Changed to LinearOpMode
//
//import org.firstinspires.ftc.teamcode.util.RobotControl;
//
//@Autonomous(name = "Blue Left Auto (Linear)", group = "Competition")
//public class AutoNewTest extends LinearOpMode { // Extends LinearOpMode now
//
//    private Follower follower;
//    private RobotControl robot;
//    private Timer pathTimer;
//
//    // --- COORDINATES ---
//    private final Pose startPose = new Pose(-36, 60, Math.toRadians(0));
//    private final Pose spikeMarkPose = new Pose(-24, 60, Math.toRadians(0));
//    private final Pose scorePose = new Pose(50, 36, Math.toRadians(0));
//    private final Pose parkPose = new Pose(50, 60, Math.toRadians(0));
//    private final Pose curveControlPose = new Pose(0, 60, 0);
//
//    private Path scorePreload;
//    private PathChain driveToBackdrop, park;
//
//    public void buildPaths() {
//        // Path 1: Start -> Spike Mark
//        scorePreload = new Path(new BezierLine(startPose, spikeMarkPose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkPose.getHeading());
//
//        // Path 2: Spike Mark -> Backdrop
//        driveToBackdrop = follower.pathBuilder()
//                .addPath(new BezierCurve(spikeMarkPose, curveControlPose, scorePose))
//                .setLinearHeadingInterpolation(spikeMarkPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        // Path 3: Backdrop -> Park
//        park = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, parkPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
//                .build();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // 1. Initialize
//        pathTimer = new Timer();
//        robot = new RobotControl(this); // This now works because we are in LinearOpMode!
//        robot.init();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        // ---------------- STEP 1: DRIVE TO SPIKE MARK ----------------
//        follower.followPath(scorePreload);
//
//        // This loop keeps the robot moving until it reaches the destination
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update(); // CRITICAL: Must call update in the loop
//            telemetry.addData("Action", "Driving to Spike Mark");
//            telemetry.update();
//        }
//
//        // ---------------- STEP 2: DROP PIXEL ----------------
//        if (opModeIsActive()) {
//            robot.intake.setPower(-0.5); // Eject
//            sleep(1000);                 // Wait 1 second (simpler than timers!)
//            robot.intake.setPower(0);    // Stop
//        }
//
//        // ---------------- STEP 3: DRIVE TO BACKDROP ----------------
//        follower.followPath(driveToBackdrop, true); // true = hold position at end
//
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addData("Action", "Driving to Backdrop");
//            telemetry.update();
//        }
//
//        // ---------------- STEP 4: SCORE ----------------
//        if (opModeIsActive()) {
//            // Spin up
//            robot.shooter.setPower(1.0);
//            sleep(500);
//
//            // Push
//            robot.shootpush.setPower(1.0);
//            sleep(1000);
//
//            // Stop
//            robot.shooter.setPower(0);
//            robot.shootpush.setPower(0);
//        }
//
//        // ---------------- STEP 5: PARK ----------------
//        follower.followPath(park, true);
//
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addData("Action", "Parking");
//            telemetry.update();
//        }
//
//        // End of Auto
//    }
//}



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.RobotControl;
import org.firstinspires.ftc.teamcode.util.pedroPathing.Constants;

@Autonomous(name = "Blue Auto: Shoot 3", group = "Competition")
public class AutoNewTest extends LinearOpMode {

    private Follower follower;
    private RobotControl robot;

    // --------------------------------------------------------
    // COORDINATES (Blue Alliance, Positive Y)
    // --------------------------------------------------------

    // START: Assuming you start near the Backboard to shoot quickly
    private final Pose startPose = new Pose(9, 60, Math.toRadians(180));

    // SHOOT: Ideally this is where your shooter aims best at the goal
    private final Pose shootPose = new Pose(68, 36, Math.toRadians(90));

    // PICKUP: Location of the balls on the floor (Spike Mark or Stack)
    // Adjusted to be slightly away so we can drive *through* it or stop *at* it
    private final Pose pickupPose = new Pose(34, 63, Math.toRadians(0));

    // Paths
    private Path startToShoot;
    private PathChain shootToPickup, pickupToShoot;

    public void buildPaths() {
        // Path 1: Drive from Start to Shooting Position
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // Path 2: Drive from Shoot to Pickup
        // We use a curve to make it smooth
        shootToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        // Path 3: Drive from Pickup back to Shoot
        pickupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();
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

            // Fire 3 shots using our helper method
            fireShots(3);
        }

        // ----------------------------------------------------
        // PHASE 3: GO TO PICKUP (AND INTAKE)
        // ----------------------------------------------------
        // Start intake BEFORE we start moving, or right as we start
        robot.intake.setPower(1); // Turn on intake

        follower.followPath(shootToPickup, true);
        driveUntilDone("Driving to Pickup");

        // We are at the pickup spot now.
        // Optional: Move forward slightly or wait to ensure we grabbed the ball
        sleep(500); // Give intake time to grab the ball
        robot.intake.setPower(0); // Stop intake (don't want to carry it running?)

        // ----------------------------------------------------
        // PHASE 4: GO BACK TO SHOOT
        // ----------------------------------------------------
        follower.setMaxPower(0.67);
        follower.followPath(pickupToShoot, true);
        driveUntilDone("Returning to Shoot");

        // ----------------------------------------------------
        // PHASE 5: SHOOT AGAIN
        // ----------------------------------------------------
        if (opModeIsActive()) {
            telemetry.addData("Action", "Shooting Collected Ball");
            telemetry.update();

            // Fire 1 shot (or however many we picked up)
            fireShots(1);
        }

        // End of Auto - Stop everything
        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }

    /**
     * Helper method to keep the main loop clean.
     * Handles spinning up the shooter and pushing the balls.
     */
    private void fireShots(int numberOfShots) {
        // 1. Spin up the shooter
        robot.shooter.setPower(.67);
        telemetry.addData("fsrdgth", robot.shooter.getPower());

        sleep(3400); // Wait for flywheel to get to speed

        // 2. Cycle the pusher for each ball

        for (int i = 0; i < numberOfShots; i++) {
            if (!opModeIsActive()) break;

            // Push ball in
            if(i ==2)
            {
                robot.intake.setPower(-1.0);
            }
            robot.shootpush.setPower(1.0);
            sleep(600);
            if (i >0){
                sleep(600);

            }

            // If your pusher needs to reverse to retract, change 0 to -1.0
            robot.intake.setPower(0);
            robot.shootpush.setPower(0);
            sleep(3900); // Wait for gravity to load the next ball
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