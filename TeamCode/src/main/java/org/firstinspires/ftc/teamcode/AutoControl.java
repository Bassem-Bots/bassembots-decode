package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

public class AutoControl {

    private LinearOpMode myOpMode = null;
    public RobotControl robot;
    public GoBildaPinpointDriver odo;
    public EnhancedNavigation navigation;
    public AprilTagHelper aprilTagHelper;

    // Field coordinates (in mm)


    public double power = 0.6;

    public double skibidi = 1.0;

    public double specArmTarget = -2050;
    public double weird = 0.6;
    private boolean isTeleOp = false;


    private static double cPower = 1;// seconds
    private static final double specArmAdjust = -150;


    public AutoControl (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public void initialize(){
        robot = new RobotControl(myOpMode);
        robot.init();
        robot.resetEncoders();
        cPower = power;

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        navigation = new EnhancedNavigation(robot, odo);
        
        // Initialize AprilTag detection
        aprilTagHelper = new AprilTagHelper();
        try {
            aprilTagHelper.init(myOpMode.hardwareMap);
            myOpMode.telemetry.addData("AprilTag", "Initialized successfully");
        } catch (Exception e) {
            myOpMode.telemetry.addData("AprilTag", "Failed to initialize: " + e.getMessage());
        }
        myOpMode.telemetry.update();
    }

    public void setup(){
        robot.armTarget = -1000;
        robot.controllerDrive(0, 1, 0, 1);
    }


    public void updateTelemetry(){
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("Current", data);
        myOpMode.telemetry.addData("arm pos", robot.armMotor.getCurrentPosition());
        myOpMode.telemetry.addData("target", robot.armTarget);
        
        // Add AprilTag telemetry if available
        if (aprilTagHelper != null && aprilTagHelper.isInitialized()) {
            myOpMode.telemetry.addData("AprilTags Detected", aprilTagHelper.getDetectionCount());
        }
        
        myOpMode.telemetry.update();
    }

    public void moveTo(double x, double y, double h, double p, double t, double c){
        navigation.resetController();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if(isTeleOp){
            t = 5;
        }
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
            if (navigation.navigateToPosition(x, y, h, p)) {
                break;
            }
            updateTelemetry();
            robot.armControl(0, c);
        }
        robot.resetDrive();
    }

    public void wait(double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
            robot.armControl(0, 1);
        }
    }

    public void teleopInitialize(RobotControl r, GoBildaPinpointDriver o, EnhancedNavigation n){
        robot = r;
        odo = o;
        navigation = n;
        isTeleOp = true;
        power = 1;
        weird = 1;
    }
    
    /**
     * Navigate to a specific AprilTag
     * @param targetId The ID of the AprilTag to navigate to
     * @param desiredDistance Distance to maintain from the tag (inches)
     * @param timeout Maximum time to spend searching/navigating (seconds)
     * @return true if successfully reached the tag, false if timeout or not found
     */
    public boolean navigateToAprilTag(int targetId, double desiredDistance, double timeout) {
        AprilTagDetection targetTag = null;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        
        myOpMode.telemetry.addData("AprilTag Navigation", "Searching for tag %d", targetId);
        myOpMode.telemetry.update();
        
        // Search phase - rotate to find the tag
        while (myOpMode.opModeIsActive() && targetTag == null && timer.seconds() < timeout * 0.3) {
            if (isTeleOp && myOpMode.gamepad1.right_stick_button) {
                return false;
            }
            
            targetTag = aprilTagHelper.getAprilTagById(targetId);
            if (targetTag == null) {
                // Slowly rotate to search for the tag
                robot.controllerDrive(0, 0, 0.3, 1);
                myOpMode.sleep(50);
            }
            updateTelemetry();
        }
        
        if (targetTag == null) {
            myOpMode.telemetry.addData("AprilTag", "Tag %d not found", targetId);
            myOpMode.telemetry.update();
            robot.resetDrive();
            return false;
        }

        // Navigation phase - move to the tag
        myOpMode.telemetry.addData("AprilTag", "Navigating to tag %d", targetId);
        myOpMode.telemetry.update();
        
        while (myOpMode.opModeIsActive() && timer.seconds() < timeout) {
            if (isTeleOp && myOpMode.gamepad1.right_stick_button) {
                break;
            }
            
            // Get fresh detection data
            targetTag = aprilTagHelper.getAprilTagById(targetId);
            
            if (targetTag != null && targetTag.ftcPose != null) {
                // Calculate errors
                double rangeError = targetTag.ftcPose.range - desiredDistance;
                double bearingError = targetTag.ftcPose.bearing;
                double yawError = targetTag.ftcPose.yaw;

                // Calculate drive powers
                double drive = Math.max(-0.5, Math.min(0.5, rangeError * 0.02));
                double strafe = Math.max(-0.5, Math.min(0.5, -bearingError * 0.015));
                double turn = Math.max(-0.3, Math.min(0.3, yawError * 0.01));

                robot.controllerDrive(drive, strafe, turn, 1);

                myOpMode.telemetry.addData("AprilTag", "Range: %.1f, Bearing: %.1f, Yaw: %.1f", 
                        targetTag.ftcPose.range, targetTag.ftcPose.bearing, targetTag.ftcPose.yaw);
                myOpMode.telemetry.addData("Drive Powers", "D: %.2f, S: %.2f, T: %.2f", drive, strafe, turn);

                // Check if we're close enough
                if (Math.abs(rangeError) < 2.0 && Math.abs(bearingError) < 5.0) {
                    robot.resetDrive();
                    myOpMode.telemetry.addData("AprilTag Navigation", "SUCCESS - Reached tag %d", targetId);
                    myOpMode.telemetry.update();
                    return true;
                }
            } else {
                // Lost sight of the tag, search again
                robot.controllerDrive(0, 0, 0.2, 1);
            }
            
            updateTelemetry();
            myOpMode.sleep(20);
        }
        
        robot.resetDrive();
        myOpMode.telemetry.addData("AprilTag Navigation", "TIMEOUT - Could not reach tag %d", targetId);
        myOpMode.telemetry.update();
        return false;
    }
    
    /**
     * Search for and align with an AprilTag without moving closer
     * @param targetId The ID of the AprilTag to align with
     * @param timeout Maximum time to spend aligning (seconds)
     * @return true if successfully aligned, false if timeout or not found
     */
    public boolean alignWithAprilTag(int targetId, double timeout) {
        AprilTagDetection targetTag = null;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        
        while (myOpMode.opModeIsActive() && timer.seconds() < timeout) {
            if (isTeleOp && myOpMode.gamepad1.right_stick_button) {
                break;
            }
            
            targetTag = aprilTagHelper.getAprilTagById(targetId);
            
            if (targetTag != null && targetTag.ftcPose != null) {
                double bearingError = targetTag.ftcPose.bearing;
                double yawError = targetTag.ftcPose.yaw;
                
                // Only rotate to align
                double turn = Math.max(-0.3, Math.min(0.3, bearingError * 0.015));
                
                robot.controllerDrive(0, 0, turn, 1);
                
                // Check if aligned
                if (Math.abs(bearingError) < 3.0 && Math.abs(yawError) < 5.0) {
                    robot.resetDrive();
                    return true;
                }
            } else {
                // Search for tag
                robot.controllerDrive(0, 0, 0.2, 1);
            }
            
            updateTelemetry();
            myOpMode.sleep(20);
        }
        
        robot.resetDrive();
        return false;
    }
    
    /**
     * Get the distance to a specific AprilTag
     * @param targetId The ID of the AprilTag
     * @return Distance in inches, or -1 if not detected
     */
    public double getAprilTagDistance(int targetId) {
        return aprilTagHelper.getDistanceToTag(targetId);
    }
    
    /**
     * Check if a specific AprilTag is detected
     * @param targetId The ID of the AprilTag
     * @return true if detected, false otherwise
     */
    public boolean isAprilTagDetected(int targetId) {
        return aprilTagHelper.isTagDetected(targetId);
    }
    
    /**
     * Clean up AprilTag resources
     */
    public void cleanup() {
        if (aprilTagHelper != null) {
            aprilTagHelper.close();
        }
    }
}