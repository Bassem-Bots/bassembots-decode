package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor Test", group="Test")
public class MotorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testMotor;
    private double motorSpeed = 0.0;
    private boolean motorRunning = false;
    private boolean lastAButtonState = false;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // Initialize the motor
        testMotor = hardwareMap.get(DcMotor.class, "testMotor"); // Change "testMotor" to your motor's name
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 👈 Add this line
        testMotor.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "A = Toggle Motor, Bumpers = Adjust Speed");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Handle button A - Toggle motor on/off at full speed
            boolean currentAButton = gamepad1.a;
            if (currentAButton && !lastAButtonState) {
                motorRunning = !motorRunning;
                if (motorRunning) {
                    motorSpeed = 1.0; // Full speed
                } else {
                    motorSpeed = 0.0; // Stop
                }
            }
            lastAButtonState = currentAButton;

            // Handle right bumper - Increase speed
            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumperState) {
                motorSpeed = Math.min(1.0, motorSpeed + 0.1);
                motorRunning = true;
            }
            lastRightBumperState = currentRightBumper;

            // Handle left bumper - Decrease speed
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumperState) {
                motorSpeed = Math.max(-1.0, motorSpeed - 0.1);
                motorRunning = true;
            }

            lastLeftBumperState = currentLeftBumper;

            // Apply motor power
            if (motorRunning)
                testMotor.setPower(motorSpeed);
            else
                testMotor.setPower(0);

            // Display telemetry
            telemetry.addData("Motor Status", motorRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Motor Speed", String.format("%.2f", motorSpeed));
            telemetry.addData("Motor Power", String.format("%.2f", testMotor.getPower()));
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("=== Controls ===");
            telemetry.addData("A Button", "Toggle Motor On/Off");
            telemetry.addData("Right Bumper", "Increase Speed (+0.1)");
            telemetry.addData("Left Bumper", "Decrease Speed (-0.1)");
            telemetry.update();
        }

        // Stop motor when OpMode ends
        testMotor.setPower(0);
    }
}