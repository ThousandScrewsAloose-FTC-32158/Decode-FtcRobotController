package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public abstract class Auto extends LinearOpMode {

    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftRear;
    protected DcMotor rightRear;
    protected DcMotorEx shooterMotor;

    protected CRServo clockwiseServo;
    protected CRServo counterClockwiseServo;

    protected static final double COUNTS_PER_MOTOR_REV = 537.7;
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;
    protected static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected long FIRE_TIME = 250;
    protected long PAUSE_TIME = 2000;

    public static double FLYWHEEL_TICKS_PER_SECOND = 1050;

    protected void initMotors()
    {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "clockwiseMotor");

        clockwiseServo = hardwareMap.get(CRServo.class, "clockwiseServo");
        counterClockwiseServo = hardwareMap.get(CRServo.class, "counterClockwiseServo");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(5, 0, 1, 15);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    protected boolean isBusy()
    {
        return leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy();
    }

    protected void driveBackward(double inches, double power) {
        int leftFrontTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int leftRearTarget = leftRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightRearTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        setTargetPosition(leftFrontTarget, leftRearTarget, rightFrontTarget, rightRearTarget);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        while (opModeIsActive() && isBusy()) {
            telemetry.addData("Reversing", "%.1f in", inches);
            telemetry.update();
        }

        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void turnLeft(double degrees, double power) {
        double INCHES_PER_DEGREE = 0.10;
        double inches = degrees * INCHES_PER_DEGREE;

        int leftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int leftRearTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightRearTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        setTargetPosition(leftFrontTarget, leftRearTarget, rightFrontTarget, rightRearTarget);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        while (opModeIsActive() && isBusy()) {
            telemetry.addData("Turning LEFT", "%.1f°", degrees);
            telemetry.update();
        }

        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void turnRight(double degrees, double power) {
        double INCHES_PER_DEGREE = 0.10;
        double inches = degrees * INCHES_PER_DEGREE;

        int leftFrontTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int leftRearTarget = leftRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int rightRearTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        setTargetPosition(leftFrontTarget, leftRearTarget, rightFrontTarget, rightRearTarget);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        while (opModeIsActive() && isBusy()) {
            telemetry.addData("Turning LEFT", "%.1f°", degrees);
            telemetry.update();
        }

        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void shootThreeTimes() {
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

    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb)
    {
        leftFront.setZeroPowerBehavior(zpb);
        leftRear.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        rightRear.setZeroPowerBehavior(zpb);
    }

    protected void setTargetPosition(int lf, int lr, int rf, int rr)
    {
        leftFront.setTargetPosition(lf);
        leftRear.setTargetPosition(lr);
        rightFront.setTargetPosition(rf);
        rightRear.setTargetPosition(rr);
    }

    protected void setMode(DcMotor.RunMode runMode)
    {
        leftFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightFront.setMode(runMode);
        rightRear.setMode(runMode);
    }

    protected void setPower(double power)
    {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    protected void addShooterMoterTelemetry()
    {
        telemetry.addData("Target FW TPS", FLYWHEEL_TICKS_PER_SECOND);
        telemetry.addData("Actual FW TPS",
                (512.0 / 1200.0) * shooterMotor.getVelocity());
    }
}
