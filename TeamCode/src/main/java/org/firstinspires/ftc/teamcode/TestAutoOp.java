package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test Auto Op", group="Test")
public class TestAutoOp extends LinearOpMode {
    
    private RobotControl robot = new RobotControl(this);
    private AprilTagHelper aprilTagHelper;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Test sequence parameters
    private static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.3;
    private static final int TARGET_TAG_ID = 1;
    
    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();
        
        // Initialize AprilTag detection
        aprilTagHelper = new AprilTagHelper();
        try {
            aprilTagHelper.init(hardwareMap);
            telemetry.addData("AprilTag", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("AprilTag", "Failed to initialize: " + e.getMessage());
        }
        
        telemetry.addData("Status", "Test Auto Ready");
        telemetry.addData("Sequence", "Drive Forward → Turn → Search for AprilTag");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // Test sequence
            testDriveForward();
            testTurnLeft();
            testAprilTagSearch();
            testDriveBackward();
            
            telemetry.addData("Status", "Test Auto Complete!");
            telemetry.update();
        }
        
        // Clean up
        robot.resetDrive();
        if (aprilTagHelper != null) {
            aprilTagHelper.close();
        }
    }
    
    /**
     * Test driving forward for 2 seconds
     */
    private void testDriveForward() {
        telemetry.addData("Test Phase", "Driving Forward");
        telemetry.update();
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.0) {
            robot.controllerDrive(DRIVE_POWER, 0, 0, 1.0);
            
            telemetry.addData("Phase", "Drive Forward");
            telemetry.addData("Time", "%.1f / 2.0 sec", runtime.seconds());
            telemetry.update();
        }
        
        robot.resetDrive();
        sleep(500); // Brief pause between tests
    }
    
    /**
     * Test turning left for 1.5 seconds
     */
    private void testTurnLeft() {
        telemetry.addData("Test Phase", "Turning Left");
        telemetry.update();
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            robot.controllerDrive(0, 0, -TURN_POWER, 1.0);
            
            telemetry.addData("Phase", "Turn Left");
            telemetry.addData("Time", "%.1f / 1.5 sec", runtime.seconds());
            telemetry.update();
        }
        
        robot.resetDrive();
        sleep(500);
    }
    
    /**
     * Test AprilTag detection while slowly rotating
     */
    private void testAprilTagSearch() {
        telemetry.addData("Test Phase", "Searching for AprilTag");
        telemetry.update();
        
        runtime.reset();
        boolean tagFound = false;
        
        while (opModeIsActive() && runtime.seconds() < 5.0 && !tagFound) {
            // Slowly rotate to search for tags
            robot.controllerDrive(0, 0, 0.2, 1.0);
            
            // Check for AprilTag detection
            if (aprilTagHelper != null && aprilTagHelper.isInitialized()) {
                int detectionCount = aprilTagHelper.getDetectionCount();
                
                telemetry.addData("Phase", "AprilTag Search");
                telemetry.addData("Time", "%.1f / 5.0 sec", runtime.seconds());
                telemetry.addData("Tags Detected", detectionCount);
                
                if (aprilTagHelper.isTagDetected(TARGET_TAG_ID)) {
                    tagFound = true;
                    double distance = aprilTagHelper.getDistanceToTag(TARGET_TAG_ID);
                    double bearing = aprilTagHelper.getBearingToTag(TARGET_TAG_ID);
                    
                    telemetry.addData("TARGET FOUND!", "Tag ID " + TARGET_TAG_ID);
                    telemetry.addData("Distance", "%.1f inches", distance);
                    telemetry.addData("Bearing", "%.1f degrees", bearing);
                    
                    robot.resetDrive();
                    sleep(2000); // Hold position to show detection
                } else if (detectionCount > 0) {
                    telemetry.addData("Other Tags", "Found but not target");
                }
            } else {
                telemetry.addData("AprilTag", "Not initialized");
            }
            
            telemetry.update();
        }
        
        robot.resetDrive();
        
        if (!tagFound) {
            telemetry.addData("Result", "No target tag found");
            telemetry.update();
            sleep(1000);
        }
    }
    
    /**
     * Test driving backward to return to start
     */
    private void testDriveBackward() {
        telemetry.addData("Test Phase", "Returning to Start");
        telemetry.update();
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.0) {
            robot.controllerDrive(-DRIVE_POWER, 0, 0, 1.0);
            
            telemetry.addData("Phase", "Drive Backward");
            telemetry.addData("Time", "%.1f / 2.0 sec", runtime.seconds());
            telemetry.update();
        }
        
        robot.resetDrive();
    }
}