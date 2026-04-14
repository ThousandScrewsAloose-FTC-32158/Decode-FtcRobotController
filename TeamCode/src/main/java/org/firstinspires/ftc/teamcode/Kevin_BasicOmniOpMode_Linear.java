/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Pinpoint Drive", group="A_Teleop")
//@Disabled
@Config //Enables FTC Dashboard configuration variables.  http://192.168.43.1:8080/dash

public class Kevin_BasicOmniOpMode_Linear extends LinearOpMode {

    public static double YAW_EPSILON = 2.0; //Degrees

    class MotorPower {
        public double LeftFront;
        public double RightFront;
        public double LeftBack;
        public double RightBack;
    }

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
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

    private PerfTimer m_LoopTimer = new PerfTimer();
    private PerfTimer m_ControlTimer = new PerfTimer();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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

        Telemetry.Item telDeltaYaw = telemetry.addData("detlaYaw", 0.0);
        Telemetry.Item telCurrentYaw = telemetry.addData("currentYaw", 0.0);
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

            double currentYaw = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
            //while(currentYaw > 360) currentYaw -= 360;
            //while(currentYaw < -360) currentYaw += 360;
            pinpoint.setHeading(currentYaw, AngleUnit.DEGREES);
            telCurrentYaw.setValue(currentYaw);

            MotorPower mp;
            if (gamepad1.dpad_down) {

                double deltaYaw = 0;
                if (currentYaw > YAW_EPSILON)
                {
                    deltaYaw = 100/360.0;
                } else if (currentYaw < -YAW_EPSILON) {
                    deltaYaw = - 100/360.0;
                }

                telDeltaYaw.setValue(deltaYaw);
                mp = ComputeMotorPower(0, 0, deltaYaw);
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

            //Pinpooint code
            if (gamepad1.dpad_up) {
                // You could use readings from April Tags here to give a new known position to the pinpoint
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            telPinpointX.setValue(Math.round(pose2D.getX(DistanceUnit.INCH) * 100) / 100.0);
            telPinpointY.setValue(Math.round(pose2D.getY(DistanceUnit.INCH) * 100) / 100.0);
            telPinpointAngle.setValue(Math.round(pose2D.getHeading(AngleUnit.DEGREES) * 100) / 100.0);

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
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

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

