package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name = "Red Auto Right Top", group = "Auto")
@Config
public class RedAutotop extends Auto {
    public static double DRIVEBACKWARD1 = -29;
    public static double DRIVEBACKWARD2 =20;
    public static double TURNLEFT = -70;
    public static double FLYWHEELSPEED = 700;

    @Override
    public void runOpMode() {

        initMotors();

        telemetry.addLine("Red Auto Backward Right READY");
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
        turnRight(TURNLEFT, 0.6);
        driveBackward(DRIVEBACKWARD2, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }
}
