package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.BlueNearAutoController;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "BlueNear")

public class BlueNear extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE,
        GO_SCORE_YELLOW,
        YELLOW_DROP,
        GO_COLLECT,
        CHECK_GO_COLLECT,
        COLLECT,
        COLLECT_PREPARE,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXELS,
COLLECT_VERIF_PIXELS_V2,
        GO_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE,
        SCORE_CYCLE,
        FAIL_SAFE,
        FAIL_SAFE_WRONG_HEADING,
        FAIL_SAFE_ONE_PIXEL,
        FAIL_SAFE_WRONG_HEADING_ONE_PIXEL,
        NOTHING
    }

    public static double x_start = 16, y_start = 62, angle_start = 90;


    /**
     * purple
     */

    public static double x_purple_preload_right = 1.6, y_purple_preload_right = 27, angle_purple_preload_right = 90;
    public static double x_purple_preload_center = 11.8, y_purple_preload_center = 24, angle_purple_preload_center = 90;
    public static double x_purple_preload_left = 19.5, y_purple_preload_left = 42, angle_purple_preload_left = 90;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 41, y_yellow_preload_right = 38, angle_yellow_preload_right = 180;
    public static double x_yellow_preload_center = 41, y_yellow_preload_center = 33, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 44, y_yellow_preload_left = 36, angle_yellow_preload_left = 180;

    public static double x_inter_yellow = 12, y_inter_yellow = 36, angle_inter_yellow = 180;

    /**
     * interns
     */

    // First cycle
    public static double x_inter_score_first_cycle_left = 29, y_inter_score_first_cycle_left = 58, angle_inter_score_first_cycle_left = 180;
    public static double x_inter_score_first_cycle_left_spline = 10, y_inter_score_first_cycle_left_spline = 58, angle_inter_score_first_cycle_left_spline = 180;
    public static double x_inter_collect_first_cycle_left_no_heanding = -10, y_inter_collect_first_cycle_left_no_heanding = 58, angle_inter_collect_first_cycle_left_no_heanding = 180;
    public static double x_inter_score_first_cycle_left_heading = -25, y_inter_score_first_cycle_left_heading = 58, angle_inter_score_first_cycle_left_heading = 204;

    public static double x_inter_score_first_cycle_center = 23, y_inter_score_first_cycle_center = 59, angle_inter_score_first_cycle_center = 180;
    public static double x_inter_score_first_cycle_center_spline = 10, y_inter_score_first_cycle_center_spline = 59, angle_inter_score_first_cycle_center_spline = 180;
    public static double x_inter_collect_first_cycle_center_no_heanding = -10, y_inter_collect_first_cycle_center_no_heanding = 59, angle_inter_collect_first_cycle_center_no_heanding = 180;
    public static double x_inter_score_first_cycle_center_heading = -25, y_inter_score_first_cycle_center_heading = 59, angle_inter_score_first_cycle_center_heading = 203;

    public static double x_inter_score_first_cycle_right = 23, y_inter_score_first_cycle_right = 59, angle_inter_score_first_cycle_right = 180;
    public static double x_inter_score_first_cycle_right_spline = 10, y_inter_score_first_cycle_right_spline = 59, angle_inter_score_first_cycle_right_spline = 180;
    public static double x_inter_collect_first_cycle_right_no_heanding = -10, y_inter_collect_first_cycle_right_no_heanding = 59, angle_inter_collect_first_cycle_right_no_heanding = 180;
    public static double x_inter_score_first_cycle_right_heading = -25, y_inter_score_first_cycle_right_heading = 59, angle_inter_score_first_cycle_right_heading = 205;

    // Second cycle
    public static double x_inter_score_second_cycle_left = 23, y_inter_score_second_cycle_left = 59, angle_inter_score_second_cycle_left = 180;
    public static double x_inter_score_second_cycle_left_spline = 10, y_inter_score_second_cycle_left_spline = 59, angle_inter_score_second_cycle_left_spline = 180;
    public static double x_inter_collect_second_cycle_left_no_heanding = -10, y_inter_collect_second_cycle_left_no_heanding = 59, angle_inter_collect_second_cycle_left_no_heanding = 180;
    public static double x_inter_score_second_cycle_left_heading = -25, y_inter_score_second_cycle_left_heading = 59, angle_inter_score_second_cycle_left_heading = 205;

    public static double x_inter_score_second_cycle_center = 23, y_inter_score_second_cycle_center = 59, angle_inter_score_second_cycle_center = 180;
    public static double x_inter_score_second_cycle_center_spline = 10, y_inter_score_second_cycle_center_spline = 59, angle_inter_score_second_cycle_center_spline = 180;
    public static double x_inter_collect_second_cycle_center_no_heanding = -10, y_inter_collect_second_cycle_center_no_heanding = 59, angle_inter_collect_second_cycle_center_no_heanding = 180;
    public static double x_inter_score_second_cycle_center_heading = -25, y_inter_score_second_cycle_center_heading = 59, angle_inter_score_second_cycle_center_heading = 205;

    public static double x_inter_score_second_cycle_right = 23, y_inter_score_second_cycle_right = 59, angle_inter_score_second_cycle_right = 180;
    public static double x_inter_score_second_cycle_right_spline = 10, y_inter_score_second_cycle_right_spline = 59, angle_inter_score_second_cycle_right_spline = 180;
    public static double x_inter_collect_second_cycle_right_no_heanding = -10, y_inter_collect_second_cycle_right_no_heanding = 59, angle_inter_collect_second_cycle_right_no_heanding = 180;
    public static double x_inter_score_second_cycle_right_heading = -25, y_inter_score_second_cycle_right_heading = 59, angle_inter_score_second_cycle_right_heading = 205;

    // Third cycle
    public static double x_inter_score_third_cycle_left = 23, y_inter_score_third_cycle_left = 59, angle_inter_score_third_cycle_left = 180;
    public static double x_inter_score_third_cycle_left_spline = 10, y_inter_score_third_cycle_left_spline = 59, angle_inter_score_third_cycle_left_spline = 180;
    public static double x_inter_collect_third_cycle_left_no_heanding = -10, y_inter_collect_third_cycle_left_no_heanding = 59, angle_inter_collect_third_cycle_left_no_heanding = 180;
    public static double x_inter_score_third_cycle_left_heading = -25, y_inter_score_third_cycle_left_heading = 59, angle_inter_score_third_cycle_left_heading = 205;

    public static double x_inter_score_third_cycle_center = 23, y_inter_score_third_cycle_center = 59, angle_inter_score_third_cycle_center = 180;
    public static double x_inter_score_third_cycle_center_spline = 10, y_inter_score_third_cycle_center_spline = 59, angle_inter_score_third_cycle_center_spline = 180;
    public static double x_inter_collect_third_cycle_center_no_heanding = -10, y_inter_collect_third_cycle_center_no_heanding = 59, angle_inter_collect_third_cycle_center_no_heanding = 180;
    public static double x_inter_score_third_cycle_center_heading = -25, y_inter_score_third_cycle_center_heading = 59, angle_inter_score_third_cycle_center_heading = 205;

    public static double x_inter_score_third_cycle_right = 23, y_inter_score_third_cycle_right = 59, angle_inter_score_third_cycle_right = 180;
    public static double x_inter_score_third_cycle_right_spline = 10, y_inter_score_third_cycle_right_spline = 59, angle_inter_score_third_cycle_right_spline = 180;
    public static double x_inter_collect_third_cycle_right_no_heanding = -10, y_inter_collect_third_cycle_right_no_heanding = 59, angle_inter_collect_third_cycle_right_no_heanding = 180;
    public static double x_inter_score_third_cycle_right_heading = -25, y_inter_score_third_cycle_right_heading = 59, angle_inter_score_third_cycle_right_heading = 205;


    // First cycle
    public static double x_score_first_cycle_left = 47, y_score_first_cycle_left = 44, angle_score_first_cycle_left = 175;
    public static double x_score_first_cycle_center = 47, y_score_first_cycle_center = 41, angle_score_first_cycle_center = 150;
    public static double x_score_first_cycle_right = 47, y_score_first_cycle_right = 41, angle_score_first_cycle_right = 130;

    // Second cycle
    public static double x_score_second_cycle_left = 47, y_score_second_cycle_left = 43, angle_score_second_cycle_left = 150;
    public static double x_score_second_cycle_center = 47, y_score_second_cycle_center = 41, angle_score_second_cycle_center = 130;
    public static double x_score_second_cycle_right = 47, y_score_second_cycle_right = 41, angle_score_second_cycle_right = 130;

    // Third cycle
    public static double x_score_third_cycle_left = 47, y_score_third_cycle_left = 41, angle_score_third_cycle_left = 130;
    public static double x_score_third_cycle_center = 47, y_score_third_cycle_center = 41, angle_score_third_cycle_center = 130;
    public static double x_score_third_cycle_right = 47, y_score_third_cycle_right = 41, angle_score_third_cycle_right = 130;





    public static int caz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RedPipelineStackMaster redLeft = new RedPipelineStackMaster(this);
        redLeft.observeStick();




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap r = new robotMap(hardwareMap);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        collectAngleController collectAngle = new collectAngleController();
        doorController door = new doorController();
        fourbarController fourbar = new fourbarController();
        latchLeftController latchLeft = new latchLeftController();
        latchRightController latchRight = new latchRightController();
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();

        BlueNearAutoController blueNearAutoController = new BlueNearAutoController();


        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.CLOSED;
        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
        lift.CS = liftController.liftStatus.INITIALIZE;
        extendo.CS = extendoController.extendoStatus.RETRACTED;
        drone.CS = droneController.droneStatus.SECURED;

//
        drive.update();
        clawAngle.update(r);
        clawFlip.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        blueNearAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

        collectAngle.update(r);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));

        Pose2d inter_yellow = new Pose2d(x_inter_yellow, y_inter_yellow, Math.toRadians(angle_inter_yellow));

        // First cycle poses
        Pose2d inter_score_first_cycle_left = new Pose2d(x_inter_score_first_cycle_left, y_inter_score_first_cycle_left, Math.toRadians(angle_inter_score_first_cycle_left));
        Pose2d inter_score_first_cycle_left_spline = new Pose2d(x_inter_score_first_cycle_left_spline, y_inter_score_first_cycle_left_spline, Math.toRadians(angle_inter_score_first_cycle_left_spline));
        Pose2d inter_collect_first_cycle_left_no_heading = new Pose2d(x_inter_collect_first_cycle_left_no_heanding, y_inter_collect_first_cycle_left_no_heanding, Math.toRadians(angle_inter_collect_first_cycle_left_no_heanding));
        Pose2d inter_collect_first_cycle_left_heading = new Pose2d(x_inter_score_first_cycle_left_heading, y_inter_score_first_cycle_left_heading, Math.toRadians(angle_inter_score_first_cycle_left_heading));

        Pose2d inter_score_first_cycle_center = new Pose2d(x_inter_score_first_cycle_center, y_inter_score_first_cycle_center, Math.toRadians(angle_inter_score_first_cycle_center));
        Pose2d inter_score_first_cycle_center_spline = new Pose2d(x_inter_score_first_cycle_center_spline, y_inter_score_first_cycle_center_spline, Math.toRadians(angle_inter_score_first_cycle_center_spline));
        Pose2d inter_collect_first_cycle_center_no_heading = new Pose2d(x_inter_collect_first_cycle_center_no_heanding, y_inter_collect_first_cycle_center_no_heanding, Math.toRadians(angle_inter_collect_first_cycle_center_no_heanding));
        Pose2d inter_collect_first_cycle_center_heading = new Pose2d(x_inter_score_first_cycle_center_heading, y_inter_score_first_cycle_center_heading, Math.toRadians(angle_inter_score_first_cycle_center_heading));

        Pose2d inter_score_first_cycle_right = new Pose2d(x_inter_score_first_cycle_right, y_inter_score_first_cycle_right, Math.toRadians(angle_inter_score_first_cycle_right));
        Pose2d inter_score_first_cycle_right_spline = new Pose2d(x_inter_score_first_cycle_right_spline, y_inter_score_first_cycle_right_spline, Math.toRadians(angle_inter_score_first_cycle_right_spline));
        Pose2d inter_collect_first_cycle_right_no_heading = new Pose2d(x_inter_collect_first_cycle_right_no_heanding, y_inter_collect_first_cycle_right_no_heanding, Math.toRadians(angle_inter_collect_first_cycle_right_no_heanding));
        Pose2d inter_collect_first_cycle_right_heading = new Pose2d(x_inter_score_first_cycle_right_heading, y_inter_score_first_cycle_right_heading, Math.toRadians(angle_inter_score_first_cycle_right_heading));

// Second cycle poses
                // Second cycle - left positions
             Pose2d inter_score_second_cycle_left = new Pose2d(x_inter_score_second_cycle_left, y_inter_score_second_cycle_left, Math.toRadians(angle_inter_score_second_cycle_left));
        Pose2d inter_score_second_cycle_left_spline = new Pose2d(x_inter_score_second_cycle_left_spline, y_inter_score_second_cycle_left_spline, Math.toRadians(angle_inter_score_second_cycle_left_spline));
        Pose2d inter_collect_second_cycle_left_no_heading = new Pose2d(x_inter_collect_second_cycle_left_no_heanding, y_inter_collect_second_cycle_left_no_heanding, Math.toRadians(angle_inter_collect_second_cycle_left_no_heanding));
        Pose2d inter_collect_second_cycle_left_heading = new Pose2d(x_inter_score_second_cycle_left_heading, y_inter_score_second_cycle_left_heading, Math.toRadians(angle_inter_score_second_cycle_left_heading));

        Pose2d inter_score_second_cycle_center = new Pose2d(x_inter_score_second_cycle_center, y_inter_score_second_cycle_center, Math.toRadians(angle_inter_score_second_cycle_center));
        Pose2d inter_score_second_cycle_center_spline = new Pose2d(x_inter_score_second_cycle_center_spline, y_inter_score_second_cycle_center_spline, Math.toRadians(angle_inter_score_second_cycle_center_spline));
        Pose2d inter_collect_second_cycle_center_no_heading = new Pose2d(x_inter_collect_second_cycle_center_no_heanding, y_inter_collect_second_cycle_center_no_heanding, Math.toRadians(angle_inter_collect_second_cycle_center_no_heanding));
        Pose2d inter_collect_second_cycle_center_heading = new Pose2d(x_inter_score_second_cycle_center_heading, y_inter_score_second_cycle_center_heading, Math.toRadians(angle_inter_score_second_cycle_center_heading));

        Pose2d inter_score_second_cycle_right = new Pose2d(x_inter_score_second_cycle_right, y_inter_score_second_cycle_right, Math.toRadians(angle_inter_score_second_cycle_right));
        Pose2d inter_score_second_cycle_right_spline = new Pose2d(x_inter_score_second_cycle_right_spline, y_inter_score_second_cycle_right_spline, Math.toRadians(angle_inter_score_second_cycle_right_spline));
        Pose2d inter_collect_second_cycle_right_no_heading = new Pose2d(x_inter_collect_second_cycle_right_no_heanding, y_inter_collect_second_cycle_right_no_heanding, Math.toRadians(angle_inter_collect_second_cycle_right_no_heanding));
        Pose2d inter_collect_second_cycle_right_heading = new Pose2d(x_inter_score_second_cycle_right_heading, y_inter_score_second_cycle_right_heading, Math.toRadians(angle_inter_score_second_cycle_right_heading));

// Third cycle poses
        Pose2d inter_score_third_cycle_left = new Pose2d(x_inter_score_third_cycle_left, y_inter_score_third_cycle_left, Math.toRadians(angle_inter_score_third_cycle_left));
        Pose2d inter_score_third_cycle_left_spline = new Pose2d(x_inter_score_third_cycle_left_spline, y_inter_score_third_cycle_left_spline, Math.toRadians(angle_inter_score_third_cycle_left_spline));
        Pose2d inter_collect_third_cycle_left_no_heading = new Pose2d(x_inter_collect_third_cycle_left_no_heanding, y_inter_collect_third_cycle_left_no_heanding, Math.toRadians(angle_inter_collect_third_cycle_left_no_heanding));
        Pose2d inter_collect_third_cycle_left_heading = new Pose2d(x_inter_score_third_cycle_left_heading, y_inter_score_third_cycle_left_heading, Math.toRadians(angle_inter_score_third_cycle_left_heading));

        Pose2d inter_score_third_cycle_center = new Pose2d(x_inter_score_third_cycle_center, y_inter_score_third_cycle_center, Math.toRadians(angle_inter_score_third_cycle_center));
        Pose2d inter_score_third_cycle_center_spline = new Pose2d(x_inter_score_third_cycle_center_spline, y_inter_score_third_cycle_center_spline, Math.toRadians(angle_inter_score_third_cycle_center_spline));
        Pose2d inter_collect_third_cycle_center_no_heading = new Pose2d(x_inter_collect_third_cycle_center_no_heanding, y_inter_collect_third_cycle_center_no_heanding, Math.toRadians(angle_inter_collect_third_cycle_center_no_heanding));
        Pose2d inter_collect_third_cycle_center_heading = new Pose2d(x_inter_score_third_cycle_center_heading, y_inter_score_third_cycle_center_heading, Math.toRadians(angle_inter_score_third_cycle_center_heading));

        Pose2d inter_score_third_cycle_right = new Pose2d(x_inter_score_third_cycle_right, y_inter_score_third_cycle_right, Math.toRadians(angle_inter_score_third_cycle_right));
        Pose2d inter_score_third_cycle_right_spline = new Pose2d(x_inter_score_third_cycle_right_spline, y_inter_score_third_cycle_right_spline, Math.toRadians(angle_inter_score_third_cycle_right_spline));
        Pose2d inter_collect_third_cycle_right_no_heading = new Pose2d(x_inter_collect_third_cycle_right_no_heanding, y_inter_collect_third_cycle_right_no_heanding, Math.toRadians(angle_inter_collect_third_cycle_right_no_heanding));
        Pose2d inter_collect_third_cycle_right_heading = new Pose2d(x_inter_score_third_cycle_right_heading, y_inter_score_third_cycle_right_heading, Math.toRadians(angle_inter_score_third_cycle_right_heading));

        // First cycle Pose2d
        Pose2d score_first_cycle_left = new Pose2d(x_score_first_cycle_left, y_score_first_cycle_left, Math.toRadians(angle_score_first_cycle_left));
        Pose2d score_first_cycle_center = new Pose2d(x_score_first_cycle_center, y_score_first_cycle_center, Math.toRadians(angle_score_first_cycle_center));
        Pose2d score_first_cycle_right = new Pose2d(x_score_first_cycle_right, y_score_first_cycle_right, Math.toRadians(angle_score_first_cycle_right));

// Second cycle Pose2d
        Pose2d score_second_cycle_left = new Pose2d(x_score_second_cycle_left, y_score_second_cycle_left, Math.toRadians(angle_score_second_cycle_left));
        Pose2d score_second_cycle_center = new Pose2d(x_score_second_cycle_center, y_score_second_cycle_center, Math.toRadians(angle_score_second_cycle_center));
        Pose2d score_second_cycle_right = new Pose2d(x_score_second_cycle_right, y_score_second_cycle_right, Math.toRadians(angle_score_second_cycle_right));

// Third cycle Pose2d
        Pose2d score_third_cycle_left = new Pose2d(x_score_third_cycle_left, y_score_third_cycle_left, Math.toRadians(angle_score_third_cycle_left));
        Pose2d score_third_cycle_center = new Pose2d(x_score_third_cycle_center, y_score_third_cycle_center, Math.toRadians(angle_score_third_cycle_center));
        Pose2d score_third_cycle_right = new Pose2d(x_score_third_cycle_right, y_score_third_cycle_right, Math.toRadians(angle_score_third_cycle_right));



        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
             //   .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(purpleLeft)
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
              //  .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(yellowLeft)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
            //    .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(yellowCenter)
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(yellowRight)
                .build();


        TrajectorySequence YELLOW_DROP_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                .back(12)
                .build();

        TrajectorySequence YELLOW_DROP_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .back(12)
                .build();


        TrajectorySequence YELLOW_DROP_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .back(12)
                .build();

        // First cycle trajectories
        TrajectorySequence COLLECT_FIRST_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .lineToLinearHeading(inter_score_first_cycle_left)
                .splineToLinearHeading(inter_score_first_cycle_left_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_first_cycle_left_no_heading)
                .lineToLinearHeading(inter_collect_first_cycle_left_heading)
                .build();

        TrajectorySequence COLLECT_FIRST_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .lineToLinearHeading(inter_score_first_cycle_center)
                .splineToLinearHeading(inter_score_first_cycle_center_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_first_cycle_center_no_heading)
                .lineToLinearHeading(inter_collect_first_cycle_center_heading)
                .build();

        TrajectorySequence COLLECT_FIRST_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                .lineToLinearHeading(inter_score_first_cycle_right)
                .splineToLinearHeading(inter_score_first_cycle_right_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_first_cycle_right_no_heading)
                .lineToLinearHeading(inter_collect_first_cycle_right_heading)
                .build();

// Second cycle trajectories
        TrajectorySequence COLLECT_SECOND_LEFT = drive.trajectorySequenceBuilder(score_first_cycle_left)
                .lineToLinearHeading(inter_score_second_cycle_left)
                .lineToLinearHeading(inter_score_second_cycle_left_spline)
                .lineToLinearHeading(inter_collect_second_cycle_left_no_heading)
                .lineToLinearHeading(inter_collect_second_cycle_left_heading)
                .build();

        TrajectorySequence COLLECT_SECOND_CENTER = drive.trajectorySequenceBuilder(score_first_cycle_center)
                .lineToLinearHeading(inter_score_second_cycle_center)
                .splineToLinearHeading(inter_score_second_cycle_center_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_second_cycle_center_no_heading)
                .lineToLinearHeading(inter_collect_second_cycle_center_heading)
                .build();

        TrajectorySequence COLLECT_SECOND_RIGHT = drive.trajectorySequenceBuilder(score_first_cycle_right)
                .lineToLinearHeading(inter_score_second_cycle_right)
                .splineToLinearHeading(inter_score_second_cycle_right_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_second_cycle_right_no_heading)
                .lineToLinearHeading(inter_collect_second_cycle_right_heading)
                .build();

// Third cycle trajectories
        TrajectorySequence COLLECT_THIRD_LEFT = drive.trajectorySequenceBuilder(score_second_cycle_left)
                .lineToLinearHeading(inter_score_third_cycle_left)
                .lineToLinearHeading(inter_score_third_cycle_left_spline)
                .lineToLinearHeading(inter_collect_third_cycle_left_no_heading)
                .lineToLinearHeading(inter_collect_third_cycle_left_heading)
                .build();

        TrajectorySequence COLLECT_THIRD_CENTER = drive.trajectorySequenceBuilder(score_second_cycle_center)
                .lineToLinearHeading(inter_score_third_cycle_center)
                .splineToLinearHeading(inter_score_third_cycle_center_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_third_cycle_center_no_heading)
                .lineToLinearHeading(inter_collect_third_cycle_center_heading)
                .build();

        TrajectorySequence COLLECT_THIRD_RIGHT = drive.trajectorySequenceBuilder(score_second_cycle_right)
                .lineToLinearHeading(inter_score_third_cycle_right)
                .splineToLinearHeading(inter_score_third_cycle_right_spline, Math.toRadians(180))
                .lineToLinearHeading(inter_collect_third_cycle_right_no_heading)
                .lineToLinearHeading(inter_collect_third_cycle_right_heading)
                .build();

        // First cycle trajectories
        TrajectorySequence SCORE_FIRST_LEFT = drive.trajectorySequenceBuilder(COLLECT_FIRST_LEFT.end())
                .lineToLinearHeading(inter_collect_first_cycle_left_no_heading)
                .lineToLinearHeading(inter_score_first_cycle_left)
                .lineToLinearHeading(score_first_cycle_left)
                .build();

        TrajectorySequence SCORE_FIRST_CENTER = drive.trajectorySequenceBuilder(COLLECT_FIRST_CENTER.end())
                .lineToLinearHeading(inter_collect_first_cycle_center_no_heading)
                .lineToLinearHeading(inter_score_first_cycle_center)
                .lineToLinearHeading(score_first_cycle_center)
                .build();

        TrajectorySequence SCORE_FIRST_RIGHT = drive.trajectorySequenceBuilder(COLLECT_FIRST_RIGHT.end())
                .lineToLinearHeading(inter_collect_first_cycle_right_no_heading)
                .lineToLinearHeading(inter_score_first_cycle_right)
                .splineToLinearHeading(score_first_cycle_right, Math.toRadians(0))
                .build();

// Second cycle trajectories
        TrajectorySequence SCORE_SECOND_LEFT = drive.trajectorySequenceBuilder(COLLECT_SECOND_LEFT.end())
                .lineToLinearHeading(inter_collect_second_cycle_left_no_heading)
                .lineToLinearHeading(inter_score_second_cycle_left)
                .lineToLinearHeading(score_second_cycle_left)
                .build();

        TrajectorySequence SCORE_SECOND_CENTER = drive.trajectorySequenceBuilder(COLLECT_SECOND_CENTER.end())
                .lineToLinearHeading(inter_collect_second_cycle_center_no_heading)
                .lineToLinearHeading(inter_score_second_cycle_center)
                .splineToLinearHeading(score_second_cycle_center, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_SECOND_RIGHT = drive.trajectorySequenceBuilder(COLLECT_SECOND_RIGHT.end())
                .lineToLinearHeading(inter_collect_second_cycle_right_no_heading)
                .lineToLinearHeading(inter_score_second_cycle_right)
                .splineToLinearHeading(score_second_cycle_right, Math.toRadians(0))
                .build();

// Third cycle trajectories
        TrajectorySequence SCORE_THIRD_LEFT = drive.trajectorySequenceBuilder(COLLECT_THIRD_LEFT.end())
                .lineToLinearHeading(inter_collect_third_cycle_left_no_heading)
                .lineToLinearHeading(inter_score_third_cycle_left)
                .lineToLinearHeading(score_third_cycle_left)
                .build();

        TrajectorySequence SCORE_THIRD_CENTER = drive.trajectorySequenceBuilder(COLLECT_THIRD_CENTER.end())
                .lineToLinearHeading(inter_collect_third_cycle_center_no_heading)
                .lineToLinearHeading(inter_score_third_cycle_center_spline)
                .splineToLinearHeading(score_third_cycle_center, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_THIRD_RIGHT = drive.trajectorySequenceBuilder(COLLECT_THIRD_RIGHT.end())
                .lineToLinearHeading(inter_collect_third_cycle_right_no_heading)
                .lineToLinearHeading(inter_score_third_cycle_right_spline)
                .splineToLinearHeading(score_third_cycle_right, Math.toRadians(0))
                .build();



        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;


        int nrcicluri = 0;
        double loopTime = 0;
        ElapsedTime purple_timer = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime failsafe = new ElapsedTime();
        double purple[] = {3.25, 2.8, 3};


        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;


        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(redLeft.opencvred.getWhichSide() == "left"){
                caz = 0;
            } else if (redLeft.opencvred.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", redLeft.opencvred.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();
            int extendopos = r.extendoLeft.getCurrentPosition();


            MotorConfigurationType motorConfigurationType = r.extendoRight.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoRight.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendoLeft.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoLeft.setMotorType(motorConfigurationType);

            motorConfigurationType = r.lift.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.lift.setMotorType(motorConfigurationType);

            //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);

            switch (status) {

                case START: {

                    switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                            break;
                        }
                    }

                    purple_timer.reset();
                    blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.PURPLE;
                    status = STROBOT.PURPLE;
                    break;
                }

                case PURPLE:
                {
                    if(purple_timer.seconds() > 1.5)
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;
                }

                case GO_SCORE_YELLOW:
                {
                    if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.PURPLE_DROP_DONE)
                    {
                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                                break;
                            }
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                                break;

                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                                break;
                            }
                        }
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN;
                        status= STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
                    if(!drive.isBusy() && blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.SCORE_YELLOW_DONE)
                    {

                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_LEFT);
                                break;
                            }
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_CENTER);
                                break;

                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_RIGHT);
                                break;
                            }
                        }
                                status = STROBOT.CHECK_GO_COLLECT;
                    }
                    break;
                }

                case CHECK_GO_COLLECT:
                {
                    if(r.back.getDistance( DistanceUnit.CM) < 24  )
                    {
                            BlueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.LATCH_DROP;
                            status = STROBOT.GO_COLLECT;
                    }
                    break;
                }

                case GO_COLLECT:
                {

                    switch (nrcicluri) {
                        case 0: // Assuming this is for the first cycle
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_RIGHT);
                                    break;
                            }
                            break;


                        default:
                            // Handle unexpected number of cycles
                            System.out.println("Unexpected number of cycles: " + nrcicluri);
                    }
prepare_collect.reset();
                    status = STROBOT.COLLECT_PREPARE;
                    break;
                }

                case COLLECT_PREPARE:
                {
                    if(prepare_collect.seconds() > 0.1)
                    { blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.COLLECT_PREPARE;
                    status= STROBOT.COLLECT_EXTENDO;}
                    break;
                }

                case COLLECT_EXTENDO:
                {
                    if(!drive.isBusy())
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                               failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXELS;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXELS:
                {
                    if(!r.pixelLeft.getState() || !r.pixelRight.getState())
                    {
                        collectAngle.collectAngle_i -=1;
                        status = STROBOT.COLLECT_VERIF_PIXELS_V2;
                    }else if(failsafe.seconds() > 1.6 && (r.pixelLeft.getState() && r.pixelRight.getState()))
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.FAIL_SAFE;
                        status = STROBOT.FAIL_SAFE;
                    }
                    break;
                }

                case FAIL_SAFE:
                {
                    if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.FAIL_SAFE_DONE)
                    {extendo.x =0;
                        failsafe.reset();
                        status = STROBOT.COLLECT_VERIF_PIXELS_V2;
                    } else if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.FAIL_SAFE_WRONG_HEADING)
                    {extendo.x =0;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        r.collect.setPower(0);
                        failsafe.reset();
                        status = STROBOT.FAIL_SAFE_WRONG_HEADING;
                    }
                    break;
                }

                case FAIL_SAFE_WRONG_HEADING:
                {
                    if(failsafe.seconds() > 1.5)
                    {   collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXELS;}
                    break;
                }


                case COLLECT_VERIF_PIXELS_V2:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        extendo_timer.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    }else if(failsafe.seconds() > 0.5)
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.FAIL_SAFE_ONE_PIXEL;
                        status = STROBOT.FAIL_SAFE_ONE_PIXEL;
                    }
                    break;
                }

                case FAIL_SAFE_ONE_PIXEL:
                {
                    if(BlueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.FAIL_SAFE_DONE_ONE_PIXEL)
                    {extendo.x =0;
                        failsafe.reset();
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        status = STROBOT.GO_SCORE_CYCLE;
                    } else if(BlueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.FAIL_SAFE_WRONG_HEADING_ONE_PIXEL)
                    {extendo.x =0;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        r.collect.setPower(0);
                        failsafe.reset();
                        status = STROBOT.FAIL_SAFE_WRONG_HEADING_ONE_PIXEL;
                    }
                    break;
                }

                case FAIL_SAFE_WRONG_HEADING_ONE_PIXEL:
                {
                    if(failsafe.seconds() > 1.5)
                    {   collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 3;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 1;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXELS_V2;}
                    break;
                }

                case GO_SCORE_CYCLE:
                {

                    switch (nrcicluri) {
                        case 0:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_RIGHT);
                                    break;
                            }
                            break;

                        case 1:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_RIGHT);
                                    break;
                            }
                            break;
                        case 2:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_RIGHT);
                                    break;
                            }
                            break;

                    }
                   blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.TRANSFER_BEGIN;
                      status = STROBOT.PREPARE_SCORE_CYCLE;
                    break;
                }

                case PREPARE_SCORE_CYCLE:
                {r.collect.setPower(0);
                    if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 10)
                    {
                      blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri +=1;
                        score.reset();
                      status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if( blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.SCORE_CYCLE_DONE && !drive.isBusy())
                    {
                       blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.LATCH_DROP;

                        switch (nrcicluri) {
                            case 1:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(COLLECT_SECOND_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(COLLECT_SECOND_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(COLLECT_SECOND_RIGHT);
                                        break;
                                }
                                break;

                            case 2: // This is for the third cycle
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(COLLECT_THIRD_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(COLLECT_THIRD_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(COLLECT_THIRD_RIGHT);
                                        break;
                                }
                                break;



                            default:
                                drive.followTrajectorySequenceAsync(COLLECT_THIRD_RIGHT);
                                break;

                        }
                        prepare_collect.reset();
                        status = STROBOT.COLLECT_PREPARE;
                    }

                    break;
                }



            }

            drive.update();
            clawAngle.update(r);
            clawFlip.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            drone.update(r);
            lift.update(r, position, voltage);
            extendo.update(r, extendopos, 1, voltage);
            collectAngle.update(r);
            blueNearAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);



//            telemetry.addData("status", status);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("flip", clawFlip.CS);
//            telemetry.addData("angle", clawAngle.CS);
//            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);

            // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
            telemetry.addData("robotcontroller", blueNearAutoController.CurrentStatus);
            telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("x", extendo.x);
            telemetry.addData("CS", lift.CS);
            telemetry.addData("liftcp", lift.CurrentPosition);
            telemetry.addData("extendocp", r.extendoLeft.getCurrentPosition());
            //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
