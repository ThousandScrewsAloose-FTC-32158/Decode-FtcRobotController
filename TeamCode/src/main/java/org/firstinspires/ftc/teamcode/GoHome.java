
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Go Home", group="A_Teleop")
//@Disabled
@Config //Enables FTC Dashboard configuration variables.  http://192.168.43.1:8080/dash

public class GoHome extends LinearOpMode {

    public static double YAW_EPSILON = 2.0; //Degrees
    public static double XY_EPSILON = 0.5;  //CM

    static class MotorPower {
        public double LeftFront;
        public double RightFront;
        public double LeftBack;
        public double RightBack;
    }

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver pinpoint;

    static class PerfTimer
    {
        public void Start() { m_StartTime = System.nanoTime(); }

        public void Stop() { m_EndTime = System.nanoTime();}

        public double MinMs() { return m_MinMs; }

        public double MaxMs() { return m_MaxMs; }

        public double ElapsedMs()
        {
            double elapsed = (m_EndTime - m_StartTime) / 1000000.0;
            if(elapsed > m_MaxMs)
            {
                m_MaxMs = elapsed;
            }

            if(elapsed < m_MinMs)
            {
                m_MinMs = elapsed;
            }
            return Math.round(elapsed * 1000.0) / 1000.0;
        }

        private long m_StartTime;
        private long m_EndTime;

        private double m_MinMs = Double.MAX_VALUE;
        private double m_MaxMs = Double.MIN_VALUE;
    }

    private final PerfTimer m_LoopTimer = new PerfTimer();
    private final PerfTimer m_ControlTimer = new PerfTimer();

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        Telemetry.Item telYawPower = telemetry.addData("yawPower", 0.0);
        Telemetry.Item telXPower = telemetry.addData("xPower", 0.0);
        Telemetry.Item telYPower = telemetry.addData("yPower", 0.0);
        Telemetry.Item telXYScale = telemetry.addData("xyScale", 0.0);
        Telemetry.Item telCurrentYaw = telemetry.addData("currentYaw", 0.0);
        Telemetry.Item telCurrentX = telemetry.addData("currentX", 0.0);
        Telemetry.Item telCurrentY = telemetry.addData("currentY", 0.0);
        Telemetry.Item telLateral = telemetry.addData("lateral", 0.0);
        Telemetry.Item telAxial = telemetry.addData("axial", 0.0);
        Telemetry.Item telYaw = telemetry.addData("yaw", 0.0);
        Telemetry.Item telPinpointX = telemetry.addData("X coordinate (IN)", 0.0);
        Telemetry.Item telPinpointY = telemetry.addData("Y coordinate (IN)", 0.0);
        Telemetry.Item telPinpointAngle = telemetry.addData("Heading angle (DEGREES)", 0.0);
        Telemetry.Item telRuntime = telemetry.addData("Status", "Run Time: ", 0);
        Telemetry.Item telLoopTimer = telemetry.addData("Loop ms: ", 0.0);
        Telemetry.Item telControlTimer = telemetry.addData("Control ms: ", 0.0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            m_LoopTimer.Start();

            Pose2D pose2D = pinpoint.getPosition();

            double currentYaw = pose2D.getHeading(AngleUnit.DEGREES);
            double currentX = pose2D.getX(DistanceUnit.CM);
            double currentY = pose2D.getY(DistanceUnit.CM);

            telCurrentYaw.setValue(currentYaw);
            telCurrentX.setValue(currentX);
            telCurrentY.setValue(currentY);

            MotorPower mp;
            if (gamepad1.dpad_down)
            {
                double yawPower = 0;
                double xPower = 0;
                double yPower = 0;

                double MAX_DIST = 50.0; //CM
                double MAX_SCALE = 1.0;
                double MIN_SCALE = 0.05;

                double distFromHome = Math.sqrt((currentX * currentX) + (currentY * currentY));

                double xyScale = distFromHome / MAX_DIST;
                xyScale = Math.min(MAX_SCALE, xyScale);
                xyScale = Math.max(MIN_SCALE, xyScale);

                telXYScale.setValue(xyScale);

                if (currentYaw > YAW_EPSILON)
                {
                    yawPower = 0.3;
                }
                else if (currentYaw < -YAW_EPSILON)
                {
                    yawPower = -0.3;
                }

                if(currentX > XY_EPSILON)
                {
                    xPower = -0.3 * xyScale;
                }
                else if(currentX < XY_EPSILON)
                {
                    xPower = 0.3 * xyScale;
                }

                if(currentY > XY_EPSILON)
                {
                    yPower = -0.3 * xyScale;
                }
                else if(currentY < XY_EPSILON)
                {
                    yPower = 0.3 * xyScale;
                }

                telYawPower.setValue(yawPower);
                telXPower.setValue(xPower);
                telYPower.setValue(yPower);
                mp = ComputeMotorPower(xPower, yPower, 0);//yawPower);
            }
            else {
                m_ControlTimer.Start();
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                telLateral.setValue(lateral);
                telAxial.setValue(axial);
                telYaw.setValue(yaw);
                mp = ComputeMotorPower(axial, lateral, yaw);
                m_ControlTimer.Stop();
            }

            //Pinpoint code
            if (gamepad1.dpad_up) {
                // You could use readings from April Tags here to give a new known position to the pinpoint
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            pinpoint.update();

            telPinpointX.setValue(Math.round(pose2D.getX(DistanceUnit.INCH) * 100) / 100.0);
            telPinpointY.setValue(Math.round(pose2D.getY(DistanceUnit.INCH) * 100) / 100.0);
            //telPinpointAngle.setValue(Math.round(pose2D.getHeading(AngleUnit.DEGREES) * 100) / 100.0);
            telPinpointAngle.setValue(pose2D.getHeading(AngleUnit.DEGREES));

            // Send calculated power to wheels
            frontLeftDrive.setPower(mp.LeftFront);
            frontRightDrive.setPower(mp.RightFront);
            backLeftDrive.setPower(mp.LeftBack);
            backRightDrive.setPower(mp.RightBack);

            // Show the elapsed game time and wheel power.
            telRuntime.setValue(runtime);

            m_LoopTimer.Stop();
            telLoopTimer.setValue(m_LoopTimer.ElapsedMs());
            telControlTimer.setValue(m_ControlTimer.ElapsedMs());

            telemetry.update();
        }
    }

    public void configurePinpoint() {
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(35, 35, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    MotorPower ComputeMotorPower(double axial, double lateral, double yaw) {
        double max;
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }
        MotorPower mp = new MotorPower();
        mp.LeftBack = backLeftPower;
        mp.RightBack = backRightPower;
        mp.LeftFront = frontLeftPower;
        mp.RightFront = frontRightPower;
        return mp;
    }
}

