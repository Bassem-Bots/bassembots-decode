package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TestOp", group="Linear OpMode")
public class TestOp extends LinearOpMode {

    private DcMotor skibidi = null;
    private DcMotor skibidii = null;
    private DcMotor skibidiii = null;
    private DcMotor skibidiiii = null;
    
    private AprilTagHelper aprilTagHelper;
    private int targetTagId = 1;

    @Override
    public void runOpMode() {
        skibidi = hardwareMap.get(DcMotor.class, "left_front_drive");
        skibidii = hardwareMap.get(DcMotor.class, "left_back_drive");
        skibidiii = hardwareMap.get(DcMotor.class, "right_front_drive");
        skibidiiii = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize AprilTag detection
        aprilTagHelper = new AprilTagHelper();
        try {
            aprilTagHelper.init(hardwareMap);
            telemetry.addData("AprilTag", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("AprilTag", "Failed to initialize: " + e.getMessage());
        }
        
        telemetry.addData("Status", "TestOp Ready - Press gamepad buttons to test");
        telemetry.update();
        
        waitForStart();

        while (opModeIsActive()) {
            // Motor control tests
            if (gamepad1.a) {
                // Test all motors forward
                skibidi.setPower(0.5);
                skibidii.setPower(0.5);
                skibidiii.setPower(0.5);
                skibidiiii.setPower(0.5);
            } else if (gamepad1.b) {
                // Test all motors backward
                skibidi.setPower(-0.5);
                skibidii.setPower(-0.5);
                skibidiii.setPower(-0.5);
                skibidiiii.setPower(-0.5);
            } else {
                // Stop motors
                skibidi.setPower(0);
                skibidii.setPower(0);
                skibidiii.setPower(0);
                skibidiiii.setPower(0);
            }
            
            // AprilTag controls
            if (gamepad1.dpad_up && targetTagId < 10) {
                targetTagId++;
                sleep(200);
            } else if (gamepad1.dpad_down && targetTagId > 1) {
                targetTagId--;
                sleep(200);
            }
            
            // Display telemetry
            telemetry.addData("Motor Test", "A=Forward, B=Backward");
            telemetry.addData("Target AprilTag", targetTagId);
            
            if (aprilTagHelper != null && aprilTagHelper.isInitialized()) {
                telemetry.addData("Tags Detected", aprilTagHelper.getDetectionCount());
                
                if (aprilTagHelper.isTagDetected(targetTagId)) {
                    telemetry.addData("Target Status", "DETECTED");
                    telemetry.addData("Distance", "%.1f inches", aprilTagHelper.getDistanceToTag(targetTagId));
                    telemetry.addData("Bearing", "%.1f degrees", aprilTagHelper.getBearingToTag(targetTagId));
                } else {
                    telemetry.addData("Target Status", "NOT DETECTED");
                }
            }
            
            telemetry.update();
        }

        // Clean up
        skibidi.setPower(0);
        skibidii.setPower(0);
        skibidiii.setPower(0);
        skibidiiii.setPower(0);
        
        if (aprilTagHelper != null) {
            aprilTagHelper.close();
        }
    }
}
