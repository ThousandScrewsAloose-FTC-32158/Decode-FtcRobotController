package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Auto Left Top", group = "Auto")
public class BlueAutotop extends Auto {

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
        driveBackward(-30, 0.6);
        shootThreeTimes();
        turnLeft(-60, 0.6);
        driveBackward(20, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }
}
