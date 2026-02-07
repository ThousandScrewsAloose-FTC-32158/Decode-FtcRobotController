package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Red Auto Right", group = "Auto")
public class RedAuto extends Auto {

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
        driveBackward(70, 0.6);
        turnRight(65, 0.5);
        driveBackward(40.4, 0.6);
        shootThreeTimes();
        turnRight(-60, 0.5);
        driveBackward(-30, 0.6);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }
}
