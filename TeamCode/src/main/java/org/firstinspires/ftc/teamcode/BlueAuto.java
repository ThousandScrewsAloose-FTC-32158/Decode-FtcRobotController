package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;


@Autonomous(name = "Blue Auto Left", group = "Auto")
@Config
public class BlueAuto extends Auto {

    public static double DRIVEBACKWARD1 = 80;
    public static double DRIVEBACKWARD2 = 28;
    public static double DRIVEBACKWARD3 = -30;
    public static double TURNLEFT1 = 105;
    public static double TURNLEFT2 = -60;

    @Override
    public void runOpMode() {

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
        turnLeft(TURNLEFT1, 0.5);
        driveBackward(DRIVEBACKWARD2, 0.6);
        shootThreeTimes();
        turnLeft(TURNLEFT2, 0.5);
        driveBackward(DRIVEBACKWARD3, 0.6);

        telemetry.addLine("AUTO DONE");
        addShooterMoterTelemetry();
        telemetry.update();
    }
}
