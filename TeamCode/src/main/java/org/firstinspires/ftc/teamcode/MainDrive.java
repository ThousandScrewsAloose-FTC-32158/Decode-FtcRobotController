package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Main Drive", group = "Drive")
public class MainDrive extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor clockwiseMotor;
    private CRServo clockwiseServo;
    private CRServo counterClockwiseServo;

    // Max driving speed
    private static final double MAX_SPEED = 0.7;

    // Manual motor correction
    private static final double LEFT_CORRECTION = 1.0;
    private static final double RIGHT_CORRECTION = 0.97;

    // Launcher motor toggle state
    private boolean launcherOn = false;
    private boolean triggerPreviouslyPressed = false;

    // Encoder constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ============================
    // Servo Shot Timing Variables
    // ============================

    enum ShootState { IDLE, FIRING, COOLDOWN }
    ShootState shootState = ShootState.IDLE;
    double stateStartTime = 0;

    final double FIRE_TIME = 0.25;      // Servo spin duration
    final double COOLDOWN = 2.2;        // Lockout between shots


    @Override
    public void runOpMode() {

        // Map hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        clockwiseMotor = hardwareMap.get(DcMotor.class, "clockwiseMotor");
        clockwiseServo = hardwareMap.get(CRServo.class, "clockwiseServo");
        counterClockwiseServo = hardwareMap.get(CRServo.class, "counterClockwiseServo");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        clockwiseMotor.setDirection(DcMotor.Direction.REVERSE);

        // Behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clockwiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Encoder modes
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clockwiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // --- Joystick input ---
            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Deadzones
            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            // --- Slow mode (L1 = 40%) ---
            double speedScale = (gamepad1.left_bumper) ? 0.40 : 1.0;
            forward *= speedScale;
            turn *= speedScale;

            // --- Drive power ---
            double leftPower = (forward + turn) * MAX_SPEED * LEFT_CORRECTION;
            double rightPower = (forward - turn) * MAX_SPEED * RIGHT_CORRECTION;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);

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
                clockwiseMotor.setPower(launcherOn ? .65: 0.0);
            }

            // ============================
            //      SERVO AUTO SHOT
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

            // -----------------------
            // Telemetry
            // -----------------------

            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);

            if (reversePressed) {
                telemetry.addData("Launcher", "REVERSING (-0.2)");
            } else {
                telemetry.addData("Launcher", launcherOn ? "CLOCKWISE (0.6)" : "OFF");
            }

            telemetry.addData("Slow Mode", speedScale == 0.4 ? "ON" : "OFF");
            telemetry.addData("Shooter State", shootState.toString());

            telemetry.update();
        }
    }

    // Helper auto move
    private void moveForwardInches(double inches, double power) {
        int newLeftTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);

        while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy())) {
            telemetry.addData("Moving Forward", "%.1f inches", inches);
            telemetry.addData("Left Pos", leftFront.getCurrentPosition());
            telemetry.addData("Right Pos", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
