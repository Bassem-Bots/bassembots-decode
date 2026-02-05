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

@Autonomous(name = "HOME RED Team Auto 5 Balls - Dec 6", group = "Home2")
public class redTestingPickupDec5 extends LinearOpMode {

    private Follower follower;
    private RobotControl robot;

    // --------------------------------------------------------
    // COORDINATES (Blue Alliance, Positive Y)
    // --------------------------------------------------------

    // START: Assuming you start near the Backboard to shoot quickly
    private final Pose startPose = new Pose(135, 100 , Math.toRadians(0));
    private final Pose moveUp = new Pose(144-50, 100, Math.toRadians(0));
    private final Pose moveSide = new Pose(144-50,122, -Math.toRadians(0));
    private final Pose pickupPose = new Pose(144-10, 122, -Math.toRadians(0));

    // SHOOT: Ideally this is where your shooter aims best at the goal
    private final Pose shootPose = new Pose(132, 93, Math.toRadians(-100));
    private final Pose shootPose2 = new Pose(122, 93, Math.toRadians(-75));

    // PICKUP: Location of the balls on the floor (Spike Mark or Stack)
    // Adjusted to be slightly away so we can drive *through* it or stop *at* it

    // Paths
    private Path startToShoot ;
    private PathChain shootToUp, upToSide, sideToPickup, pickupToShoot;

    public void buildPaths() {
        // Path 1: Drive from Start to Shooting Position
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        shootToUp = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, moveUp))
                .setLinearHeadingInterpolation(shootPose.getHeading(), moveUp.getHeading())
                .build();

        upToSide = follower.pathBuilder()
                .addPath(new BezierLine(moveUp, moveSide))
                .setLinearHeadingInterpolation(moveUp.getHeading(), moveSide.getHeading())
                .build();

        sideToPickup = follower.pathBuilder()
                .addPath(new BezierLine(moveSide, pickupPose))
                .setLinearHeadingInterpolation(moveSide.getHeading(), pickupPose.getHeading())
                .build();

        pickupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose2))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose2.getHeading())
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
        robot.shooter.setPower(0);
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
            fireShots(3, .69f, 1);
        }

        robot.shooter.setPower(0.45);


        robot.intake.setPower(-1.20);

        follower.followPath(shootToUp);
        driveUntilDone("moving up" );

        follower.followPath(upToSide);
        driveUntilDone("moving to side");

        follower.followPath(sideToPickup);
        driveUntilDone("moving to pickup");
        robot.intake.setPower(0);

        follower.followPath(pickupToShoot);
        driveUntilDone("moving to shoot");

        if (opModeIsActive()) {
            telemetry.addData("Action", "Shooting 3 Balls");
            telemetry.update();

//             Fire 2 shots using our helper method
            fireShots(2, .69f, 2);
        }

        robot.shooter.setPower(0);
        robot.intake.setPower(0);
    }

    /**
     * Helper method to keep the main loop clean.
     * Handles spinning up the shooter and pushing the balls.
     */
    private void fireShots(int numberOfShots, float power, int shotpartnumber) {
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
            if (shotpartnumber==2){
                if (i ==0){
                    sleep(600);
                }
            }
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
