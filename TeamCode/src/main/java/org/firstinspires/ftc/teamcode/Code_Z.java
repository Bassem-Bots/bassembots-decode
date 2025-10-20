package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Locale;

@TeleOp(name="Final code", group="Linear OpMode")
public class Code_Z extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControl robot = new RobotControl(this);
    //AutoControl auto = new AutoControl(this);
    private EnhancedNavigation navigation;
    private AprilTagHelper aprilTagHelper;
    private double mod = 1;
    private double slow = 1;
    private boolean isNavigating = false;
    private boolean bing = false;
    private int targetTagId = 1;
    private boolean aprilTagAssist = false;

    // Basket coordinates (adjust these based on your field setup)
    private final double BASKET_X = robot.BASKET_X_TELE;  // Using constant from RobotControl
    private final double BASKET_Y = robot.BASKET_Y_TELE;  // Using constant from RobotControl
    private static final double BASKET_HEADING = -40.0;
    private double specPhase = 0;

    private double navX = 0;
    private double navY = 0;
    private double navH = 0;// Degrees
    private boolean isEditing = false;
    private boolean nearBasket = false;
    double basketXError = BASKET_X;
    double basketYError = BASKET_Y;
    double basketHError = BASKET_HEADING;

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
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Normal teleop control when not navigating to basket
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // AprilTag assist mode
            if (aprilTagAssist && aprilTagHelper.isTagDetected(targetTagId)) {
                AprilTagDetection detection = aprilTagHelper.getAprilTagById(targetTagId);
                if (detection != null && detection.ftcPose != null) {
                    // Adjust drive inputs based on AprilTag position
                    double rangeError = detection.ftcPose.range - 12.0; // Target 12 inches
                    double bearingError = detection.ftcPose.bearing;
                    
                    // Mix manual control with AprilTag assistance
                    axial = axial * 0.7 + (rangeError * 0.02) * 0.3;
                    lateral = lateral * 0.7 + (-bearingError * 0.01) * 0.3;
                }
            }

            robot.controllerDrive(axial, lateral, yaw, mod);

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }
            
            // AprilTag controls
            handleAprilTagControls();


            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            //telemetry.addData("Arm motor target", robot.armTarget);
            telemetry.addData("near basker", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("editing", odo.getHeading());
            telemetry.addData("basket h errpr", basketHError);
            if (isNavigating) {
                telemetry.addData("Navigation", "Moving to Basket");
            }
            
            // AprilTag telemetry
            displayAprilTagTelemetry();
            
            telemetry.update();
        }
        
        // Clean up AprilTag resources
        if (aprilTagHelper != null) {
            aprilTagHelper.close();
        }
    }
    
    /**
     * Handle AprilTag-related gamepad controls
     */
    private void handleAprilTagControls() {
        // Toggle AprilTag assist mode with A button
        if (gamepad1.a) {
            aprilTagAssist = !aprilTagAssist;
            sleep(200); // Debounce
        }
        
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
     * Display AprilTag telemetry information
     */
    private void displayAprilTagTelemetry() {
        if (aprilTagHelper != null && aprilTagHelper.isInitialized()) {
            telemetry.addLine("=== AprilTag Info ===");
            telemetry.addData("Assist Mode", aprilTagAssist ? "ON" : "OFF");
            telemetry.addData("Target Tag", targetTagId);
            telemetry.addData("Tags Detected", aprilTagHelper.getDetectionCount());

            // Detailed info for target tag
            if (aprilTagHelper.isTagDetected(targetTagId)) {
                telemetry.addData("Target Status", "DETECTED");
                telemetry.addData("Distance", "%.1f inches", aprilTagHelper.getDistanceToTag(targetTagId));
                telemetry.addData("Bearing", "%.1f degrees", aprilTagHelper.getBearingToTag(targetTagId));
            } else {
                telemetry.addData("Target Status", "NOT DETECTED");
            }

            // Controls help
            telemetry.addLine("Controls: A=Toggle Assist, D-pad=Change Tag");
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