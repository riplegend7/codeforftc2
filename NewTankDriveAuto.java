package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;


/**
    Feeder: Y
    Movement: joysticks (Uses tank)
    Servo: Right_Bumper & Left_Bumper
    FlyWheel: Right_Trigger & Left_Trigger
    SpinThing: X & B
 */
@TeleOp(name="TankDrive", group="Linear OpMode")
public class NewTankDriveAuto extends LinearOpMode {

    // Variables
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor flywheel = null;
    private DcMotor Feeder = null;
    private DcMotor spinthing = null;
    private CRServo servo = null;
    private boolean flywheelRunning = false;
    private double flywheelStartTime = 0;
    private static final int farVelocity = 1900;
    private static final int bankVelocity = 1300;
    private static final int pressVel = 800;
    private boolean flywheeltoggle = false;
    private boolean awaspressed = false;
    private boolean feedertoggle = false;
    private boolean ywaspressed = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Stuff in the box of wisdom
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        Feeder = hardwareMap.get(DcMotor.class, "Feeder");
        spinthing = hardwareMap.get(DcMotor.class, "spinthing");
        servo = hardwareMap.get(CRServo.class, "servo");

        // Direction to move motor
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for start button
        waitForStart();
        runtime.reset();

        // Run until stop is pressed
        while (opModeIsActive()) {

            // Tank mode: each stick controls one side
            double leftPower  = -gamepad1.left_stick_y * 0.70;
            double rightPower = -gamepad1.right_stick_y * 0.70;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Use triggers to move flywheel
            double rt = gamepad1.right_trigger * 0.70;
            double lt = gamepad1.left_trigger;

            if (rt > 0.05) {
                // spin forward; trigger amount is throttle
                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flywheel.setPower(rt); // use 1.0 for fixed full speed
            } else if (lt > 0.05) {
                // active brake to a stop
                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                flywheel.setPower(0);
            } else {
                // coast to stop
                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flywheel.setPower(0);
            }


            // A toggle code
            if (gamepad1.a & !awaspressed) {
                flywheeltoggle = !flywheeltoggle;
            }

            if (flywheeltoggle) {
                flywheel.setPower(1);
            } else {
                flywheel.setPower(0);
            }

            if (gamepad1.y & !ywaspressed) {
                feedertoggle = !feedertoggle;
            }

            if (feedertoggle) {
                Feeder.setPower(-1);
            } else {
                Feeder.setPower(0);
            }

            if (gamepad1.x) {
                spinthing.setPower(0.1);
            } else if (gamepad1.b) {
                spinthing.setPower(-0.1);
            } else if (gamepad1.right_bumper) {
                servo.setPower(1);
            } else if (gamepad1.left_bumper) {
                servo.setPower(-1);
            }

            // Detect when flywheel turns on
            if (!flywheelRunning && Math.abs(flywheel.getPower()) > 0.05) {
                flywheelStartTime = runtime.seconds();
                flywheelRunning = true;

            }
        }

}

}