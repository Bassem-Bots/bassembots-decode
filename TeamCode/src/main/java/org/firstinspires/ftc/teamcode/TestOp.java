package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestOp", group="Linear OpMode")
public class TestOp extends LinearOpMode {

    private DcMotor skibidi = null;
    private DcMotor skibidii = null;

    @Override
    public void runOpMode() {
        skibidi = hardwareMap.get(DcMotor.class, "left");
        skibidii = hardwareMap.get(DcMotor.class, "right");

        waitForStart();
        skibidi.setPower(-1);
        skibidii.setPower(1);

        while (opModeIsActive()) {
            telemetry.addData("sigma", skibidi.getCurrentPosition());
            telemetry.update();
        }

    }
}
