package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Activate the FTC Dashboard by browsing to http://192.168.43.1:8080/dash

@TeleOp(name = "Main Drive", group = "Drive")
@Config //Enables FTC Dashboard configuration variables.
public class MainDrive extends LinearOpMode {

    // ============================
    // Drive Motors
    // ============================
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    // ============================
    // Launcher / Shooter
    // ============================
    private DcMotorEx clockwiseMotor;
    private CRServo clockwiseServo;
    private CRServo counterClockwiseServo;

    // Max drive speed
    public static final double MAX_SPEED = 0.7;

    // ============================
    // Launcher Toggle
    // ============================
    private boolean launcherOn = false;
    private boolean triggerPreviouslyPressed = false;

    // ============================
    // Shooter FSM
    // ============================
    enum ShootState { IDLE, FIRING, COOLDOWN }
    ShootState shootState = ShootState.IDLE;
    double stateStartTime = 0;

    public static double FIRE_TIME = 0.25;
    public static double COOLDOWN = 2.2;

    public static double FLYWHEEL_TICKS_PER_SECOND = 512;

    @Override
    public void runOpMode() {

        // ============================
        // Hardware Mapping
        // ============================
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        clockwiseMotor = hardwareMap.get(DcMotorEx.class, "clockwiseMotor");
        clockwiseServo = hardwareMap.get(CRServo.class, "clockwiseServo");
        counterClockwiseServo = hardwareMap.get(CRServo.class, "counterClockwiseServo");

        // ============================
        // Motor Directions (Mecanum)
        // ============================
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        clockwiseMotor.setDirection(DcMotor.Direction.REVERSE);

        // ============================
        // Behaviors
        // ============================
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clockwiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clockwiseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clockwiseMotor.setVelocityPIDFCoefficients(5,0,1,15);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ============================
        // Main Loop
        // ============================
        while (opModeIsActive()) {

            // ============================
            // MECANUM DRIVE
            // ============================
            double y  = gamepad1.left_stick_y; // Forward/back
            double x  = -gamepad1.left_stick_x; // Strafe
            double rx = -gamepad1.right_stick_x;  // Rotate

            // Deadzones
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            // Slow mode
            double speedScale = gamepad1.left_bumper ? 0.40 : 1.0;
            y  *= speedScale;
            x  *= speedScale;
            rx *= speedScale;

            // Mecanum math
            double lf = y + x + rx;
            double rf = y - x - rx;
            double lr = y - x + rx;
            double rr = y + x - rx;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lr), Math.abs(rr))
            );

            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lr /= max;
                rr /= max;
            }

            // Apply power
            leftFront.setPower(lf * MAX_SPEED);
            rightFront.setPower(rf * MAX_SPEED);
            leftRear.setPower(lr * MAX_SPEED);
            rightRear.setPower(rr * MAX_SPEED);

            // ============================
            // LAUNCHER MOTOR CONTROL
            // ============================
            boolean triggerPressed = gamepad1.right_trigger > 0.1;

            if (triggerPressed && !triggerPreviouslyPressed) {
                launcherOn = !launcherOn;
            }
            triggerPreviouslyPressed = triggerPressed;

            boolean reversePressed = gamepad1.right_bumper;

            if (reversePressed) {
                clockwiseMotor.setPower(-0.2);
            } else {
                clockwiseMotor.setVelocity(launcherOn ? FLYWHEEL_TICKS_PER_SECOND : 0);//.setPower(launcherOn ? 0.65 : 0.0);
            }

            // ============================
            // SERVO AUTO SHOOT
            // ============================
            boolean shootPressed = gamepad1.left_trigger > 0.1;

            switch (shootState) {

                case IDLE:
                    if (shootPressed) {
                        shootState = ShootState.FIRING;
                        stateStartTime = getRuntime();
                        clockwiseServo.setPower(-1.0);
                        counterClockwiseServo.setPower(1.0);
                    } else {
                        clockwiseServo.setPower(0);
                        counterClockwiseServo.setPower(0);
                    }
                    break;

                case FIRING:
                    if (getRuntime() - stateStartTime >= FIRE_TIME) {
                        clockwiseServo.setPower(0);
                        counterClockwiseServo.setPower(0);
                        shootState = ShootState.COOLDOWN;
                        stateStartTime = getRuntime();
                    }
                    break;

                case COOLDOWN:
                    if (getRuntime() - stateStartTime >= COOLDOWN) {
                        shootState = ShootState.IDLE;
                    }
                    break;
            }

            // ============================
            // TELEMETRY
            // ============================
            telemetry.addData("LF RF", "%.2f | %.2f", lf, rf);
            telemetry.addData("LR RR", "%.2f | %.2f", lr, rr);
            telemetry.addData("Launcher", reversePressed ? "REVERSING" : (launcherOn ? "ON" : "OFF"));
            telemetry.addData("Slow Mode", speedScale == 0.4 ? "ON" : "OFF");
            telemetry.addData("Shooter State", shootState);
            telemetry.addData("Target FW TPS: ", FLYWHEEL_TICKS_PER_SECOND);
            telemetry.addData("Actual FW TPS: ", clockwiseMotor.getVelocity());
            telemetry.update();
        }
    }
}
