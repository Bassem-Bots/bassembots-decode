package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag Autonomous OpMode
 * This OpMode demonstrates autonomous navigation using AprilTag detection.
 * The robot will search for and navigate to specific AprilTags.
 */
@Autonomous(name = "AprilTag Autonomous", group = "Autonomous")
public class AprilTagAutonomous extends LinearOpMode {

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Motor components (adjust names to match your robot configuration)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Navigation constants
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final double DESIRED_DISTANCE = 12.0; // inches from AprilTag
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        initAprilTag();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence
            // Look for AprilTag ID 1 and navigate to it
            navigateToAprilTag(1);
            
            // You can add more navigation commands here
            // navigateToAprilTag(2);
            // performTask();
        }

        // Clean up
        visionPortal.close();
    }

    /**
     * Initialize robot hardware
     */
    private void initHardware() {
        // Initialize motors (adjust names to match your robot configuration)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions (adjust based on your robot's configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Navigate to a specific AprilTag
     * @param targetId The ID of the AprilTag to navigate to
     */
    private void navigateToAprilTag(int targetId) {
        AprilTagDetection targetTag = null;
        
        // Search for the target AprilTag
        telemetry.addData("Searching for AprilTag", targetId);
        telemetry.update();
        
        // Rotate to search for the tag
        double searchStartTime = getRuntime();
        while (opModeIsActive() && targetTag == null && (getRuntime() - searchStartTime) < 10.0) {
            targetTag = getAprilTagById(targetId);
            if (targetTag == null) {
                // Slowly rotate to search for the tag
                setDrivePower(0, 0, TURN_SPEED);
                sleep(100);
            }
        }
        
        if (targetTag == null) {
            telemetry.addData("AprilTag", "Target not found");
            telemetry.update();
            setDrivePower(0, 0, 0);
            return;
        }

        // Navigate to the AprilTag
        telemetry.addData("AprilTag", "Navigating to target %d", targetId);
        telemetry.update();
        
        while (opModeIsActive() && targetTag != null) {
            // Get fresh detection data
            targetTag = getAprilTagById(targetId);
            
            if (targetTag != null) {
                // Calculate drive powers based on AprilTag position
                double rangeError = (targetTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = targetTag.ftcPose.bearing;
                double yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.addData("AprilTag", "Range %5.1f, Bearing %3.0f, Yaw %3.0f", 
                        targetTag.ftcPose.range, targetTag.ftcPose.bearing, targetTag.ftcPose.yaw);
                telemetry.update();

                // Apply desired axes motions to the drivetrain
                setDrivePower(drive, strafe, turn);

                // Check if we're close enough to the target
                if (Math.abs(rangeError) < 2.0) {
                    break; // We're close enough
                }
            } else {
                // Lost sight of the tag, stop and search again
                setDrivePower(0, 0, TURN_SPEED * 0.5);
            }
            
            sleep(10);
        }
        
        // Stop the robot
        setDrivePower(0, 0, 0);
        telemetry.addData("Navigation", "Complete");
        telemetry.update();
    }

    /**
     * Get AprilTag detection by ID
     */
    private AprilTagDetection getAprilTagById(int targetId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Set drive power for mecanum wheels
     * @param drive Forward/backward power
     * @param strafe Left/right power  
     * @param turn Rotation power
     */
    private void setDrivePower(double drive, double strafe, double turn) {
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Utility class for range clipping
     */
    public static class Range {
        public static double clip(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}