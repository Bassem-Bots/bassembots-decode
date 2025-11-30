package org.firstinspires.ftc.teamcode.autonomous; // make sure this aligns with class location
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    private final Pose shootPose2 = new Pose(100, 36, Math.toDegrees(-140+-25));

    // PICKUP: Location of the balls on the floor (Spike Mark or Stack)
    // Adjusted to be slightly away so we can drive *through* it or stop *at* it
    private final Pose pickupPose = new Pose(27, 65, Math.toRadians(10));

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

//        // Path 3: Drive from Pickup back to Shoot
        pickupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose2))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose2.getHeading())
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
            fireShots(3, .65f);
        }

        // ----------------------------------------------------
        // PHASE 3: GO TO PICKUP (AND INTAKE)
        // ----------------------------------------------------
        // Start intake BEFORE we start moving, or right as we start
        robot.intake.setPower(-1); // Turn on intake

        follower.followPath(shootToPickup, true);
        driveUntilDone("Driving to Pickup");

        // We are at the pickup spot now.
        // Optional: Move forward slightly or wait to ensure we grabbed the ball
        sleep(500); // Give intake time to grab the ball
        robot.intake.setPower(0); // Stop intake (don't want to carry it running?)

        // ----------------------------------------------------
        // PHASE 4: GO BACK TO SHOOT
        // ----------------------------------------------------
        follower.followPath(pickupToShoot,true);
        driveUntilDone("Returning to Shoot");

        // ----------------------------------------------------
        // PHASE 5: SHOOT AGAIN
        // ----------------------------------------------------
        if (opModeIsActive()) {
            telemetry.addData("Action", "Shooting Collected Ball");
            telemetry.update();

            // Fire 1 shot (or however many we picked up)
            fireShots(1,.59f);
        }

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

            sleep(3900);
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