package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name = "Blue Auto Left Top", group = "Auto")
@Config
public class BlueAutotop extends Auto {
    public static double DRIVEBACKWARD1 = -29;
    public static double DRIVEBACKWARD2 =20;
    public static double TURNRIGHT = -70;
    public static double FLYWHEELSPEED = 700;

    @Override
    public void runOpMode() {

        FLYWHEEL_TICKS_PER_SECOND = FLYWHEELSPEED;
        initMotors();

        telemetry.addLine("Blue Auto Backward Left READY");
        telemetry.update();

        waitForStart();

        // Shooter on first
        shooterMotor.setVelocity(FLYWHEEL_TICKS_PER_SECOND);
        //shooterMotor.setPower(0.65);
        sleep(700);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- AUTO ----
        driveBackward(DRIVEBACKWARD1, 0.6);
        shootThreeTimes();
        turnLeft(TURNRIGHT, 0.6);
        driveBackward(DRIVEBACKWARD2, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }
}
