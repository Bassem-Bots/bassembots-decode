package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotControl {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor shooter = null;
    public DcMotor intake = null;
    // public DcMotor armMotor = null;
    public GoBildaPinpointDriver odo = null;
    public boolean fieldCentric = false;

    // public static final double ARM_HANG_POS = -1500;
    // public static final int ARM_HIGH = -2800;
    // public static final int MAX_EXTENSION = -2150;
    // public static final double TICKS_PER_DEGREE = (double) 2500 / 360;
    // public static final double TICKS_PER_MM = 1;
    // public static final double BASKET_X_AUTO = 0;
    // public static final double BASKET_Y_AUTO = 0;
    // public static final double BASKET_X_TELE = 920;
    // public static final double BASKET_Y_TELE = -260;

    // public int controlOn = 1;
    // public int armTarget;
    // private ElapsedTime armTimer;
    private EnhancedNavigation navigation;
    // public double da;
    // public boolean isTeleop = false;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotControl(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        odo = myOpMode.hardwareMap.get(
                GoBildaPinpointDriver.class,
                "odo"
        );
        navigation = new EnhancedNavigation(this, odo);
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(
                DcMotor.class,
                "left_front_drive"
        );
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(
                DcMotor.class,
                "right_front_drive"
        );
        rightBackDrive = myOpMode.hardwareMap.get(
                DcMotor.class,
                "right_back_drive"
        );
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        shooter = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        //armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm_motor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

        //armTarget = armMotor.getCurrentPosition();
        //armMotor.setTargetPosition(armTarget);
    }

//    public void resetEncoders() {
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    public void controllerDrive(
            double axial,
            double lateral,
            double yaw,
            double mod
    ) {
        double theta = (fieldCentric) ? odo.getPosition().getHeading(AngleUnit.RADIANS) : Math.atan2(axial, lateral);
        double power = Math.hypot(axial, lateral);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double mmm = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos) / mmm + yaw;
        double rightFrontPower = (power * sin) / mmm - yaw;
        double leftBackPower = (power * sin) / mmm + yaw;
        double rightBackPower = (power * cos) / mmm - yaw;

        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower *= mod;
        rightFrontPower *= mod;
        leftBackPower *= mod;
        rightBackPower *= mod;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // public void armControl(double power, double cPower) {
    //     da = armTimer.seconds() < 0.5 ? armTimer.seconds() : 0;
    //     armTimer.reset();
    //     armTarget += (int) (2100.0 * power * da);
    //     double armP = navigation.clamp(
    //             navigation.calculateArmPIDF(
    //                     armMotor.getCurrentPosition(),
    //                     armTarget,
    //                     da
    //             ),
    //             -cPower,
    //             cPower
    //     );
    //     if (isTeleop) {
    //         double xi = armTarget > -1200 && armTarget < -500
    //                 ? ((armTarget + 300) / 3000.) + 0.6
    //                 : 1;
    //         myOpMode.telemetry.addData("xi", xi);
    //     }
    //     armMotor.setPower(armP);
    // }

    public void resetDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void MethRobot() {
        double distanceX = 2; // Q
        double distanceY = 1.2; // W
        final double maxHeight = 1.5; // D
        final double gravity = 9.8; // G

        double capotchinoassassino = Math.sqrt(2 * gravity * maxHeight); // Z
        double brrbrrpatapim = Math.sqrt((2 * gravity * maxHeight) - (2 *gravity * distanceY)) + capotchinoassassino; // B
        double lirililarila = Math.atan((capotchinoassassino * brrbrrpatapim) / (distanceX * gravity)); // N
        double bomberdinocrocodilo = Math.sqrt(Math.pow(distanceX * gravity/brrbrrpatapim, 2) + (2*gravity*maxHeight));

//        armTarget = (int) (lirililarila*1000);
        double shooterP = bomberdinocrocodilo;
    }
}
