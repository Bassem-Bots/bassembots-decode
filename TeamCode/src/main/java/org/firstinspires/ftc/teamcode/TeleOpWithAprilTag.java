package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp with AprilTag Integration
 * This example shows how to add AprilTag detection to an existing TeleOp OpMode
 * using the AprilTagHelper utility class.
 */
@TeleOp(name = "TeleOp with AprilTag", group = "TeleOp")
public class TeleOpWithAprilTag extends OpMode {

    // Drive motors (adjust names to match your configuration)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // AprilTag helper
    private AprilTagHelper aprilTagHelper;

    // Target AprilTag ID to track
    private int targetTagId = 1;

    @Override
    public void init() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions (adjust based on your robot)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize AprilTag detection
        aprilTagHelper = new AprilTagHelper();
        aprilTagHelper.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("AprilTag", "Ready to detect tags");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Standard TeleOp driving
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right
        double turn = gamepad1.right_stick_x;    // Rotation

        // Calculate motor powers for mecanum drive
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        // Normalize powers
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // AprilTag controls
        handleAprilTagControls();

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Handle AprilTag-related gamepad controls
     */
    private void handleAprilTagControls() {
        // Change target AprilTag ID with D-pad
        if (gamepad1.dpad_up && targetTagId < 10) {
            targetTagId++;
            sleep(200); // Debounce
        } else if (gamepad1.dpad_down && targetTagId > 1) {
            targetTagId--;
            sleep(200); // Debounce
        }

        // Control camera streaming
        if (gamepad1.y) {
            aprilTagHelper.resumeStreaming();
        } else if (gamepad1.x) {
            aprilTagHelper.stopStreaming();
        }

        // Adjust detection quality vs speed
        if (gamepad1.right_bumper) {
            aprilTagHelper.setDecimation(1); // High quality, slower
        } else if (gamepad1.left_bumper) {
            aprilTagHelper.setDecimation(3); // Lower quality, faster
        }
    }

    /**
     * Display telemetry information
     */
    private void displayTelemetry() {
        // Basic drive info
        telemetry.addData("Drive", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
                leftFrontDrive.getPower(), rightFrontDrive.getPower(),
                leftBackDrive.getPower(), rightBackDrive.getPower());

        // AprilTag info
        telemetry.addData("Target AprilTag", targetTagId);
        telemetry.addData("Tags Detected", aprilTagHelper.getDetectionCount());

        // Detailed info for target tag
        if (aprilTagHelper.isTagDetected(targetTagId)) {
            telemetry.addData("Target Status", "DETECTED");
            telemetry.addData("Distance", "%.1f inches", aprilTagHelper.getDistanceToTag(targetTagId));
            telemetry.addData("Bearing", "%.1f degrees", aprilTagHelper.getBearingToTag(targetTagId));
            telemetry.addData("Yaw", "%.1f degrees", aprilTagHelper.getYawToTag(targetTagId));
        } else {
            telemetry.addData("Target Status", "NOT DETECTED");
        }

        // Controls help
        telemetry.addLine();
        telemetry.addData("Controls", "D-pad up/down: Change target tag");
        telemetry.addData("", "Y: Resume camera, X: Stop camera");
        telemetry.addData("", "RB: High quality, LB: Fast mode");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up AprilTag resources
        if (aprilTagHelper != null) {
            aprilTagHelper.close();
        }
    }

    /**
     * Simple sleep method for debouncing
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}