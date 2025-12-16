package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

/*
 * This OpMode demonstrates some basic FTC Dashboard functionality.
 *
 * Camera Streaming
 * Telemetry
 * Configuration Variables
 *
 * Pull in FTC Dashboard code by adding the following to
 *
 * Activate the FTC Dashboard by browsing to http://192.168.43.1:8080/dash
 *
 */

@TeleOp(name = "Dashboard: FTC Dashboard Example", group = "Dashboard")
//@Disabled
@Config //Enables FTC Dashboard configuration variables.
public class FtcDashBoardExample extends LinearOpMode
{
    //Members declared "public static" will appear as config vars in FTC Dashboard.
    //For example, a variable can be added to tune motor power.
    //Or, a variable can be added for each motor to tune each motor's power independently.
    //Variables can be added for tuning controller sensitivity, changing control layout,
    //or just about anything.
    //Config variables can be changed through the dashboard without having to recompile
    //and redeploy code to the robot.
    public static double MOTOR_POWER = 1;

    //This class is used to implement webcam streaming on the dashboard.
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource
    {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration)
        {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos)
        {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext)
        {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation)
        {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Create the camera streaming processor.
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        //Start streaming the webcam to the dashboard.
        FtcDashboard.getInstance().startCameraStream(processor, 0);

        //Initialize telemetry so it goes to both the dashboard and to the driver station.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {
            //Print a line of telemetry showing the current motor power.
            //From the dashboard web page try changing the MOTOR_POWER config var
            //and see how it updates in the telemetry section.
            telemetry.addLine(String.format(Locale.US, "MOTOR_POWER: %f", MOTOR_POWER));
            telemetry.update();
            sleep(100L);
        }
    }
}
