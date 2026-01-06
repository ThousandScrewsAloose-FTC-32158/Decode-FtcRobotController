package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Blue Auto Left", group = "Auto")
public class BlueAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor shooterMotor;

    private CRServo clockwiseServo;
    private CRServo counterClockwiseServo;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    final long FIRE_TIME = 250;
    final long PAUSE_TIME = 2000;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        shooterMotor = hardwareMap.get(DcMotor.class, "clockwiseMotor");

        clockwiseServo = hardwareMap.get(CRServo.class, "clockwiseServo");
        counterClockwiseServo = hardwareMap.get(CRServo.class, "counterClockwiseServo");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Blue Auto Backward Left READY");
        telemetry.update();

        waitForStart();

        // Shooter on first
        shooterMotor.setPower(0.65);
        sleep(700);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- AUTO ----
        driveBackward(60, 0.6);
        turnLeft(70, 0.5);
        driveBackward(40.4, 0.6);
        shootThreeTimes();
        turnLeft(-60, 0.5);
        driveBackward(-30, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    private void driveBackward(double inches, double power) {
        int leftTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftTarget);
        rightFront.setTargetPosition(rightTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
            telemetry.addData("Reversing", "%.1f in", inches);
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnLeft(double degrees, double power) {
        double INCHES_PER_DEGREE = 0.10;
        double inches = degrees * INCHES_PER_DEGREE;

        int leftTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftTarget);
        rightFront.setTargetPosition(rightTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);

        while (opModeIsActive() && leftFront.isBusy()) {
            telemetry.addData("Turning LEFT", "%.1f°", degrees);
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void shootThreeTimes() {
        for (int i = 0; i < 4; i++) {
            clockwiseServo.setPower(-1.0);
            counterClockwiseServo.setPower(1.0);
            sleep(FIRE_TIME);

            clockwiseServo.setPower(0);
            counterClockwiseServo.setPower(0);

            sleep(PAUSE_TIME);
        }
        shooterMotor.setPower(0);
    }
}
