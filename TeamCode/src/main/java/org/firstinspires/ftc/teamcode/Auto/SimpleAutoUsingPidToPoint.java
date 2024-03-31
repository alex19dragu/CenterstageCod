package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.P2P.Pose;
import org.firstinspires.ftc.teamcode.Auto.P2P.funnypidtopoint;

@Photon
@Config
@Autonomous(group = "Auto", name = "TargetedAuto")
public class SimpleAutoUsingPidToPoint extends LinearOpMode {

    enum State {
        START,
        MOVE_TO_PURPLE_CENTER,
        MOVE_TO_YELLOW_CENTER,
        END
    }

    private State currentState;

    // Starting position
    private static final double x_start = 0, y_start = 0, angle_start = Math.toRadians(0);

    // Purple center
    private static final double x_purple_preload_center = 0, y_purple_preload_center = -50, angle_purple_preload_center = Math.toRadians(0);

    // Yellow center
    private static final double x_yellow_preload_center = 0, y_yellow_preload_center = 0, angle_yellow_preload_center = Math.toRadians(0);

    @Override
    public void runOpMode() {
        funnypidtopoint pidToPoint = new funnypidtopoint(hardwareMap);

        // Define target poses
        Pose startPose = new Pose(x_start, y_start, angle_start);
        Pose purpleCenterPose = new Pose(x_purple_preload_center, y_purple_preload_center, angle_purple_preload_center);
        Pose yellowCenterPose = new Pose(x_yellow_preload_center, y_yellow_preload_center, angle_yellow_preload_center);

        pidToPoint.drive.setPose(startPose); // If your drive class has a method to set the initial pose

        waitForStart();

        currentState = State.START;

        while (opModeIsActive()) {

            switch (currentState) {
                case START:
                    pidToPoint.setTargetPose(purpleCenterPose);
                    currentState = State.MOVE_TO_PURPLE_CENTER;
                    break;
                case MOVE_TO_PURPLE_CENTER:
                    if (pidToPoint.hasReachedTarget()) {
                        pidToPoint.setTargetPose(yellowCenterPose);
                        currentState = State.MOVE_TO_YELLOW_CENTER;
                    }
                    break;
                case MOVE_TO_YELLOW_CENTER:

                    if (pidToPoint.hasReachedTarget()) {
                        currentState = State.END;
                    }
                    break;
                case END:
                    pidToPoint.stopMotors();
                    requestOpModeStop();
                    break;
            }

            pidToPoint.update();

            telemetry.addData("paralel", hardwareMap.dcMotor.get("collect").getCurrentPosition());
            telemetry.addData("perpendicular", hardwareMap.dcMotor.get("extendoRight").getCurrentPosition());
            telemetry.update();

        }


    }
}

