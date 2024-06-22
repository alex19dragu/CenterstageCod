package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.SensorPublisher;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(group = "Auto", name = "TestAuto")

public class testbreak extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        collectAngleController collectAngleController = new collectAngleController();
        robotMap r = new robotMap(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;

        collectAngleController.update(r);

        SensorPublisher sensorPublisher = new SensorPublisher(r);

        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));
        TrajectorySequence testTrajectory = drive.trajectorySequenceBuilder(start_pose)
                .forward(50)
                .build();

        drive.setPoseEstimate(start_pose);

        waitForStart();
        sensorPublisher.startPublishing();

        drive.followTrajectorySequenceAsync(testTrajectory);

        while (opModeIsActive() && !isStopRequested()) {
            if (sensorPublisher.getSensorState()) {
                drive.breakFollowing();
                break;
            }
            drive.update();
            telemetry.addData("Sensor State", sensorPublisher.getSensorState());
            telemetry.update();
        }

        sensorPublisher.stopPublishing();
    }
}
