package org.firstinspires.ftc.teamcode.Auto.Recognition;


import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.Recognition.AtagYellow;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "AtagYellowOpMode", group = "Concept")
public class AtagYellowOpMode extends LinearOpMode {

    private AtagYellow atagYellow ;
    public static int desieredTag = 5;

    @Override
    public void runOpMode() {



        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)

        .build();

        atagYellow = new AtagYellow(aprilTagProcessor, desieredTag);


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(atagYellow)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);

        // Wait for start button to be pressed
        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", atagYellow.location);
            telemetry.addData("right zone average: ", atagYellow.rightZoneAverage);
            telemetry.addData("left zone average: ", atagYellow.leftZoneAverage);
//            telemetry.addData("does it see", atagYellow.doesitseedesieredatag);
//            telemetry.addData("Location", atagYellow.location);
            telemetry.update();
           // visionPortal.stopStreaming();

        }

        waitForStart();

        while (opModeIsActive()) {

//            telemetry.addData("Location", atagYellow.location);
            telemetry.addData("location", atagYellow.location);
            telemetry.addData("right zone average: ", atagYellow.rightZoneAverage);
            telemetry.addData("left zone average: ", atagYellow.leftZoneAverage);
            telemetry.update();
            sleep(100);


        }

        // Stop the camera
        visionPortal.close();
    }
}