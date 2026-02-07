package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Auto Left", group = "Auto")
public class BlueAuto extends Auto {

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
        driveBackward(60, 0.6);
        turnLeft(70, 0.5);
        driveBackward(40.4, 0.6);
        shootThreeTimes();
        turnLeft(-60, 0.5);
        driveBackward(-30, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }
}
