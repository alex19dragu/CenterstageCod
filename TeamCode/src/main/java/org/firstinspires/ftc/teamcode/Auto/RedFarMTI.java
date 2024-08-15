package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.Auto.Recognition.Globals.yellow_drop.left;
import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.Side.blue;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe;
import org.firstinspires.ftc.teamcode.Auto.Recognition.BluePipelineStackMaster;

import org.firstinspires.ftc.teamcode.Auto.Recognition.Globals;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.SensorPublisher;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.droneLatchController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

@Photon
@Config
@Autonomous(group = "Auto", name = "RedFarBTC")

public class RedFarMTI extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE,
        SYSTEMS_PURPLE,
        COLLECT_PURPLE,
        VERIF_TIMER,
        GO_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW_v2,
        VERIF_PURPLE_SCORE,
        YELLOW_DROP,
        WAIT_A_LIL_BIT_PLS,

        PREPARE_COLLECT,
        PREPARE_COLLECT_YELLOW,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXLES,
        COLLECT_VERIF_PIXLES_V2,
        atag_bby,

        GO_SCORE_CYCLE,
        GO_SCORE_CYCLE_FUNNY_JAVA,
        GO_SCORE_CYCLE_FUNNY_JAVA_GROUND,
        PREPARE_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE_ON_GROUND,
        SCORE_CYCLE,
        SCORE_CYCLE_GROUND,
        FAIL_SAFE,
        FAIL_SAFE_HEADER_VERIF,
        FAIL_SAFE_2,
        FAIL_SAFE_HEADER_VERIF_2,
        FAILSAFE_PURPLE,
        RETRACT_AND_RERTY,
        RETRY_TIMER_RESET,
        RETRACT_AND_RERTY_v2,
        RETRY_TIMER_RESET_v2,
        GO_COLLECT,
        GO_PARK,
        GO_SCORE_ON_GROUND,
        WAIT_alliance,
        BREAK_TRAJ,

        PARK,


        NOTHING
    }

    public static int clawanglefrompipeline = 0;

    public static double x_start = -43, y_start = -61, angle_start = 270;

    /**
     * purple
     */

    public static double x_purple_preload_right = -41, y_purple_preload_right = -26.5, angle_purple_preload_right = 175;
    public static double x_purple_preload_center = -49, y_purple_preload_center = -22, angle_purple_preload_center = 181;
    public static double x_purple_preload_left = -61.5, y_purple_preload_left = -32, angle_purple_preload_left = 184;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 45, y_yellow_preload_right = -35, angle_yellow_preload_right = 180;
    public static double x_yellow_preload_center = 45, y_yellow_preload_center = -30, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 46, y_yellow_preload_left = -25, angle_yellow_preload_left = 180;
    // public static double x_yellow_inter =

    /**
     * collect
     */


    // Cycle 2
    public static double x_inter_collect_cycle_2_right = 30, y_inter_collect_cycle_2_right = -7, angle_inter_collect_cycle_2_right = 180;
    public static double x_collect_cycle_2_right = -28, y_collect_cycle_2_right = -7, angle_collect_cycle_2_right = 180;

    public static double x_inter_collect_cycle_2_center = 30, y_inter_collect_cycle_2_center = -7, angle_inter_collect_cycle_2_center = 180;
    public static double x_collect_cycle_2_center = -30, y_collect_cycle_2_center = -7, angle_collect_cycle_2_center = 180;

    public static double x_inter_collect_cycle_2_left = 30, y_inter_collect_cycle_2_left = -7, angle_inter_collect_cycle_2_left = 180;
    public static double x_collect_cycle_2_left = -28, y_collect_cycle_2_left = -7, angle_collect_cycle_2_left = 180;

    // Cycle 3
    public static double x_inter_collect_cycle_3_right = 30, y_inter_collect_cycle_3_right = -7, angle_inter_collect_cycle_3_right = 180;
    public static double x_collect_cycle_3_right = -28, y_collect_cycle_3_right = -7, angle_collect_cycle_3_right = 180;

    public static double x_inter_collect_cycle_3_center = 30, y_inter_collect_cycle_3_center = -8, angle_inter_collect_cycle_3_center = 180;
    public static double x_collect_cycle_3_center = -30, y_collect_cycle_3_center = -8, angle_collect_cycle_3_center = 180;

    public static double x_inter_collect_cycle_3_left = 30, y_inter_collect_cycle_3_left = -7, angle_inter_collect_cycle_3_left = 180;
    public static double x_collect_cycle_3_left = -28, y_collect_cycle_3_left = -7, angle_collect_cycle_3_left = 180;

    public static double retry_x = -24.5, retry_y = -10, retry_angle = 180;

    /**
     * score
     */


    // Second cycle scoring positions
    public static double x_score_second_cycle_right = 50, y_score_second_cycle_right = 18, angle_score_second_angle_right = 205;
    public static double x_score_second_cycle_center = 50, y_score_second_cycle_center = 18, angle_score_second_angle_center = 205;
    public static double x_score_second_cycle_left = 50, y_score_second_cycle_left = 22.5, angle_score_second_angle_left = 200;

    // Third cycle scoring positions
    public static double x_score_third_cycle_right = 49.5, y_score_third_cycle_right = 16, angle_score_third_angle_right = 205;
    public static double x_score_third_cycle_center = 49.5, y_score_third_cycle_center = 16, angle_score_third_angle_center = 205;
    public static double x_score_third_cycle_left = 49.5, y_score_third_cycle_left = 21.5, angle_score_third_angle_left = 200;


    public static double x_score_forth_cycle_right = 49.5, y_score_forth_cycle_right = 16, angle_score_forth_angle_right = 205;
    public static double x_score_forth_cycle_center = 49.5, y_score_forth_cycle_center = 16, angle_score_forth_angle_center = 205;
    public static double x_score_forth_cycle_left = 49.5, y_score_forth_cycle_left = 21.5, angle_score_forth_angle_left = 197;


    /**
     * intern collect
     */

    public static double x_inter_collect_first_cycle = -44, y_inter_collect_first_cycle = -9, angle_inter_collect_first_cycle = 180;

    public static double x_inter_collect_first_cycle_left = -55, y_inter_collect_first_cycle_left = -8, angle_inter_collect_first_cycle_left = 180;

    public static double x_inter_collect_first_cycle_center = -57.5, y_inter_collect_first_cycle_center = -9, angle_inter_collect_first_cycle_center = 180;

    /**
     * intern score
     */

    public static double x_inter_score_first_cycle = 23, y_inter_score_first_cycle = -7, angle_inter_score_first_cycle = 180;
    public static double x_inter_score_first_cycle_left = 23, y_inter_score_first_cycle_left = -7, angle_inter_score_first_cycle_left = 180;

    public static int caz = 0;
    public static double limit = 1;
    boolean forced = false;
    public static int tries = 0;
    public static int tries_purple = 0;
    //  private YellowPipeline yellowPipeline;

    //public static int desieredtag = 0;
    public static boolean checkatag = true;
    public static boolean youcango = true;
    public static boolean traj_failsafe = false;


    @Override
    public void runOpMode() throws InterruptedException {

        checkatag = false;
        youcango = true;
        traj_failsafe = false;

        RedPipelineStackMaster redFar = new RedPipelineStackMaster(this);
        redFar.observeStick();

        //DistanceSensorCalibrator calibrator;

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
        failsafe failsafecontroller = new failsafe();
        droneLatchController droneLatch = new droneLatchController();

        RedFarAutoController redFarAutoController = new RedFarAutoController();

        SensorPublisher sensorPublisher = new SensorPublisher(r);


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
        failsafecontroller.CurrentStatus = failsafe.failsafeStatus.NOTHING;
        droneLatch.CS = droneLatchController.droneLatchStatus.SECURED;
        // angle mai mare la al treilea ciclu
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
        extendo.update(r, 0, 1, voltage, sensorPublisher);
        redFarAutoController.update(r, lift, fourbar, clawAngle, clawFlip, collectAngle, door, extendo, latchLeft, latchRight);
        failsafecontroller.update(r, lift, fourbar, clawAngle, clawFlip, collectAngle, door, extendo, latchLeft, latchRight);
        droneLatch.update(r);

        collectAngle.update(r);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start, Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d interCollectFirstCycleleft = new Pose2d(x_inter_collect_first_cycle_left, y_inter_collect_first_cycle_left, Math.toRadians(angle_inter_collect_first_cycle_left));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));


        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight, SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter, SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleLeft, SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
                .lineToLinearHeading(interCollectFirstCycleleft, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    youcango = true;
                })
//                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
//                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
                .lineToLinearHeading(new Pose2d(-49, -8, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    youcango = true;
                })
//                .lineToLinearHeading(interCollectFirstCycleCenter)
//                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
//                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    youcango = true;
                })
                .build();

        TrajectorySequence YELLOW_LEFT2 = drive.trajectorySequenceBuilder(YELLOW_LEFT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(0, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(15, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    checkatag = true;
                })
                .splineToLinearHeading(new Pose2d(35, -25, Math.toRadians(180)), Math.toRadians(300), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -27, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(9, 9, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(yellowLeft, Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
////                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
////                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
//                .build();

        TrajectorySequence YELLOW_CENTER2 = drive.trajectorySequenceBuilder(YELLOW_CENTER.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(0, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(15, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .UNSTABLE_addTemporalMarkerOffset(-0.5, ()  -> {checkatag = true;})
                // .waitSeconds(0.01)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    checkatag = true;
                })
                .splineToLinearHeading(new Pose2d(35, -27, Math.toRadians(180)), Math.toRadians(300), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46.2, -29.7, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //.waitSeconds(0.1)
//                .lineToLinearHeading(yellowCenter,
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.UNSTABLE_addTemporalMarkerOffset(-0.5, ()  -> {checkatag = true;})
//                .lineToLinearHeading(interCollectFirstCycleCenter)
//                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
//                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
                .build();

        TrajectorySequence YELLOW_RIGHT2 = drive.trajectorySequenceBuilder(YELLOW_RIGHT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(0, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(15, -8, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    checkatag = true;
                })
                .splineToLinearHeading(new Pose2d(35, -32.7, Math.toRadians(180)), Math.toRadians(300), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -35.7, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(5, 9, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(yellowRight, Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();


        TrajectorySequence COLLECT_CYCLE_2_RIGHT = drive.trajectorySequenceBuilder(YELLOW_RIGHT2.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-37.5, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(Math.toRadians(240))
//                .splineToLinearHeading(new Pose2d(15, 10, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(-25, 10, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

        TrajectorySequence COLLECT_CYCLE_2_CENTER = drive.trajectorySequenceBuilder(YELLOW_CENTER2.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-37.5, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence COLLECT_CYCLE_2_LEFT = drive.trajectorySequenceBuilder(YELLOW_LEFT2.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -9.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-37.5, -9.5, Math.toRadians(184)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(Math.toRadians(240))
//                .splineToLinearHeading(new Pose2d(15, 11.5, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(-25, 11.5, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_RIGHT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46.2, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(score_second_cycle_right)
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_CENTER.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(score_second_cycle_center)
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_LEFT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // .lineToLinearHeading(score_second_cycle_left)
                .build();

        TrajectorySequence COLLECT_CYCLE_3_RIGHT = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_RIGHT.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-38, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(Math.toRadians(240))
//                .splineToLinearHeading(new Pose2d(20, 10, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(-25.5, 10, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

        TrajectorySequence COLLECT_CYCLE_3_CENTER = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_CENTER.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-38, -10, Math.toRadians(182)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence COLLECT_CYCLE_3_LEFT = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_LEFT.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(25, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-22.5, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-38, -10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//                .setTangent(Math.toRadians(240))
//                .splineToLinearHeading(new Pose2d(15, 11.5, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(-25, 11.5, Math.toRadians(180)), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();


        TrajectorySequence SCORE_THIRD_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_RIGHT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46.2, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(score_third_cycle_right)
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_CENTER.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46.2, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // .lineToLinearHeading(score_third_cycle_center)
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_LEFT.end())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9.5, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -30, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.lineToLinearHeading(score_third_cycle_left)
                .build();

        TrajectorySequence COLLECT_CYCLE_4_RIGHT = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_RIGHT.end())
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-22.5, 7, Math.toRadians(168)), Math.toRadians(180))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(25, -9, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(0, -9, Math.toRadians(180)), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-27.3, -9, Math.toRadians(195)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence COLLECT_CYCLE_4_CENTER = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_CENTER.end())
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-22, 6.75, Math.toRadians(169)), Math.toRadians(180))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(25, -9, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(0, -9, Math.toRadians(180)), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-26.8, -9, Math.toRadians(195)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence COLLECT_CYCLE_4_LEFT = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_LEFT.end())
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-22, 4, Math.toRadians(165)), Math.toRadians(180))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(25, -9, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(0, -9, Math.toRadians(180)), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-26.8, -9, Math.toRadians(193)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence Fail_safe_case_2 = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_CENTER.end())
//                .lineToLinearHeading(new Pose2d(-26.8, 9.2, Math.toRadians(156)))
//                .lineToLinearHeading(new Pose2d(-27, 9.3, Math.toRadians(176)))
//                .lineToLinearHeading(new Pose2d(-27.1, 9.4, Math.toRadians(166)))
                .turn(Math.toRadians(40))
                .turn(Math.toRadians(-60))
                .turn(Math.toRadians(20))
                .build();

        TrajectorySequence Fail_safe_case_1 = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_CENTER.end())
                .turn(Math.toRadians(40))
                .turn(Math.toRadians(-60))
                .turn(Math.toRadians(20))
//                .lineToLinearHeading(new Pose2d(-25, 14.6, Math.toRadians(170)))
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(-25, 14.7, Math.toRadians(190)))
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(-25, 14.8, Math.toRadians(180)))
                .build();


        TrajectorySequence SCORE_FORTH_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_RIGHT.end())
                //    .lineToLinearHeading(score_forth_cycle_right)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -31, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence SCORE_FORTH_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_CENTER.end())
                // .lineToLinearHeading(score_forth_cycle_center)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -31, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence SCORE_FORTH_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_LEFT.end())
                //  .lineToLinearHeading(score_forth_cycle_left)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(46, -31, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence ParkBun = drive.trajectorySequenceBuilder(SCORE_FORTH_CYCLE_CENTER.end())
                .lineToLinearHeading(new Pose2d(42, -8, Math.toRadians(180)))
                .build();

        TrajectorySequence ParkBun2 = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_CENTER.end())
                .lineToLinearHeading(new Pose2d(42, -8, Math.toRadians(180)))
                .build();

        TrajectorySequence RETRY = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_CENTER.end())
                .lineToLinearHeading(new Pose2d(retry_x, retry_y, Math.toRadians(retry_angle)))
                .build();


        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;


        int nrcicluri = 0;
        double loopTime = 0;
        double extendo_timer_i[] = {1.8, 1.8, 2.9};

        double extendo_timer_i_purple[] = {1.9, 1.9, 1.1};

        double pos_trash[] = {0, 100, 200};

        double break_traj_trash[] = {-59.5, -38.5, -53.5};

        ElapsedTime transfer = new ElapsedTime();
        ElapsedTime prepare_score_yellowqe = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime verif_pixels = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime verif = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime failsafe = new ElapsedTime();
        ElapsedTime failsafe2 = new ElapsedTime();
        ElapsedTime header = new ElapsedTime();
        ElapsedTime park = new ElapsedTime();
        ElapsedTime park_systems = new ElapsedTime();
        ElapsedTime failsafe_purple = new ElapsedTime();
        ElapsedTime retry = new ElapsedTime();
        ElapsedTime alilbitup = new ElapsedTime();
        ElapsedTime goscrcycl = new ElapsedTime();
        ElapsedTime skibidi = new ElapsedTime();
        ElapsedTime stai = new ElapsedTime();
        ElapsedTime asteapta = new ElapsedTime();

        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;
        tries = 0;
        tries_purple = 0;


//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
//               .setLensIntrinsics(520.035, 520.035, 288.093, 269.186)
//                .build();

        //     yellowPipeline = new YellowPipeline(aprilTag, Globals.desieredtag);
        Globals.alliance = Globals.Alliance.RED;
        Globals.canirelocalize = false;
        Globals.waitformiddle = false;
        checkatag = false;



//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .addProcessor(aprilTag)
//                .addProcessor(yellowPipeline)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .build();
//
//        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//        }
//
//        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
//        exposure.setMode(ExposureControl.Mode.Manual);
//        exposure.setExposure(3, TimeUnit.MILLISECONDS);
//
//        GainControl gain = visionPortal.getCameraControl(GainControl.class);
//        gain.setGain(255);

        //  yellowPipeline.diditsee = false;
        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if (redFar.opencvred.getWhichSide() == "left") {
                Globals.desieredtag = 4;
                clawAngleController.auto = clawAngleController.score[4];
                Globals.yellow_drop_side = left;
                extendoController.purple_max = 0;
                Globals.is_left = true;
                caz = 0;
            } else if (redFar.opencvred.getWhichSide() == "center") {
                caz = 1;
                Globals.desieredtag = 5;
                clawAngleController.auto = clawAngleController.score[3];
                Globals.yellow_drop_side = left;
                extendoController.purple_max = 340;
                Globals.is_left = false;
            } else {
                Globals.desieredtag = 6;
                clawAngleController.auto = clawAngleController.score[3];
                extendoController.purple_max = 550;
                Globals.yellow_drop_side = left;
                Globals.is_left = false;
                caz = 2;
            }
            telemetry.addData("case", redFar.opencvred.getWhichSide());
            telemetry.update();

            sleep(50);
        }
        sensorPublisher.startPublishing();

        waitForStart();
        park.reset();
//        double[] rawReadings = {28.5, 27.3, 26.2, 24.9, 24.3, 23.6, 28.4, 29.5, 30.6, 31.2, 31.3, 31.6, 32.3, 33.4, 34.5, 35.3, 36.1, 37.8, 37.9, 38};
//        double[] actualDistances = {23, 22, 21.5, 21, 20.5, 20, 22.5, 24, 24.4, 24.6, 25.2, 25.5, 26, 26.6, 27.2, 28.1, 28.9, 29.6, 30, 30.5};
//        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);


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


                    switch (caz) {
                        case 0: {
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                            RedFarAutoController.claw_caz = 2;
                            RedFarAutoController.funny_or_notblue = false;
                            clawAngle.clawAnglePurple_i = 1;
                            extendo.caz = 2;
                            break;
                        }
                        case 1: {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            RedFarAutoController.funny_or_notblue = false;
                            clawAngle.clawAnglePurple_i = 2;
                            RedFarAutoController.claw_caz = 1;

                            extendo.caz = 1;
                            break;

                        }
                        case 2: {
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                            RedFarAutoController.funny_or_notblue = false;
                            clawAngle.clawAnglePurple_i = 1;
                            RedFarAutoController.claw_caz = 2;

                            extendo.caz = 0;
                            break;
                        }
                    }

                    preload.reset();
                    status = STROBOT.SYSTEMS_PURPLE;
                    break;
                }

                case SYSTEMS_PURPLE: {
                    if (preload.seconds() > 0.269) {
                        //  blueRight.stopCamera();
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE;
                        extendo_timer.reset();
                        status = STROBOT.PURPLE;
                    }
                    break;
                }

                case PURPLE: {
                    if (extendo_timer.seconds() > extendo_timer_i_purple[caz]) {
                        r.collect.setPower(1);
                        preload.reset();
                        extendo.CS = extendoController.extendoStatus.PURPLE;
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        collectAngle.collectAngle_i = 4;
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case COLLECT_PURPLE: {
                    if ((!r.pixelLeft.getState() && !r.pixelRight.getState()) || ( sensorPublisher.getSensorState() && extendopos > pos_trash[caz])) {
                        skibidi.reset();
                        status = STROBOT.WAIT_A_LIL_BIT_PLS;
                    } else if (preload.seconds() > 2 && tries < 3 && (r.pixelLeft.getState() || r.pixelRight.getState())) {
                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_PURPLE;

                        status = STROBOT.FAILSAFE_PURPLE;
                    } else if (tries_purple >= 3) {
                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        verif.reset();
                        status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;

                }

                case WAIT_A_LIL_BIT_PLS:
                {
                    if(skibidi.seconds() > 0.15)
                    {  collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        verif.reset();
                        status = STROBOT.GO_SCORE_YELLOW;}
                    break;
                }

                case FAILSAFE_PURPLE: {
                    if (failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE_PURPLE) {
                        tries_purple += 1;
                        preload.reset();
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case GO_SCORE_YELLOW: {
                    switch (caz) {
                        case 0: {
                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                            if (verif.seconds() > 0.1) {
                                r.collect.setPower(-1);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                        case 1: {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            if (verif.seconds() > 0.1) {
                                r.collect.setPower(-1);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;

                        }
                        case 2: {
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                            if (verif.seconds() > 0.1) {
                                r.collect.setPower(-1);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                    }

                    break;

                }

                case PREPARE_SCORE_YELLOW: {
                    asteapta.reset();
                    if (transfer.seconds() > 0.35) {
                        r.collect.setPower(-1);
                    }

                    if (transfer.seconds() > 0.65) {
                        r.collect.setPower(0);
                    }

                    if (transfer.seconds() > 0.85) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        prepare_score_yellowqe.reset();
                        status = STROBOT.WAIT_alliance;
                    }
                    break;
                }

                case WAIT_alliance: {//visionPortal.resumeStreaming();
                    if ((drive.getPoseEstimate().getY() > -15 || !drive.isBusy()) && asteapta.seconds()>1 ) {
                        switch (caz) {
                            case 0: {
                                drive.followTrajectorySequenceAsync(YELLOW_LEFT2);
                                status = STROBOT.PREPARE_SCORE_YELLOW_v2;
                                break;
                            }
                            case 1: {
                                drive.followTrajectorySequenceAsync(YELLOW_CENTER2);
                                status = STROBOT.PREPARE_SCORE_YELLOW_v2;
                                break;

                            }
                            case 2: {
                                drive.followTrajectorySequenceAsync(YELLOW_RIGHT2);

                                status = STROBOT.PREPARE_SCORE_YELLOW_v2;
                                break;
                            }
                        }
                    }

                    break;
                }

                case PREPARE_SCORE_YELLOW_v2: {
                    if (drive.getPoseEstimate().getX() >= 15 && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN_BLUE;
                        status = STROBOT.VERIF_PURPLE_SCORE;
                    }

                    break;
                }

//                case atag_bby: {
//                    if (checkatag = true) {
//                            visionPortal.resumeStreaming();
//                    }
//                    if (yellowPipeline.diditsee == true) {
//
//                            //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), yellowPipeline.poseFromTag(drive.getPoseEstimate(), Globals.aprilTagDetection).getY(), drive.getPoseEstimate().getHeading()));
//                            // drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(180)));
//                            visionPortal.stopStreaming();
//
//
//
//                    }
//                    if ((yellowPipeline.diditsee == true || !drive.isBusy()) && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE_BLUE) {
//                        // visionPortal.stopStreaming();
//                        redFarAutoController.claw_timer.reset();
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_CLAW;
//                        status = STROBOT.VERIF_PURPLE_SCORE;
//                    }
//                    break;
//                }

                case VERIF_PURPLE_SCORE: {

                    if ((drive.getPoseEstimate().getX() >= 44.7 || !drive.isBusy()) && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE) {

                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP: {

                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP_YELLOW;

                    switch (caz) {
                        case 0: {
                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_LEFT);
                            break;
                        }

                        case 1: {
                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_CENTER);
                            break;
                        }

                        case 2: {
                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_RIGHT);
                            break;
                        }

                    }

                    prepare_collect.reset();
                    status = STROBOT.PREPARE_COLLECT_YELLOW;

                    break;
                }
                case PREPARE_COLLECT_YELLOW: {
                    if (prepare_collect.seconds() > 0.4) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                    }
                    break;
                }

                case PREPARE_COLLECT: {
                    r.collect.setPower(0);
                    if (prepare_collect.seconds() > 0.24) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                    }
                    break;
                }

                case COLLECT_EXTENDO: {
                    if (park.seconds() < 27) {
                        if (extendo_timer.seconds() > (extendo_timer_i[nrcicluri] -0.4) && drive.getPoseEstimate().getX() <= 15) {
                            collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                            r.collect.setPower(1);

                            switch (nrcicluri) {
                                case 0: {
                                    collectAngle.collectAngle_i = 4;
                                    extendo.CS = extendoController.extendoStatus.CYCLE;
                                    failsafe.reset();
                                    break;
                                }
                                case 1: {
                                    collectAngle.collectAngle_i = 2;
                                    extendo.CS = extendoController.extendoStatus.CYCLE;
                                    failsafe.reset();
                                    break;
                                }
                                case 2: {
                                    if (caz == 0) {
                                        collectAngle.collectAngle_i = 4;
                                    } else {
                                        collectAngle.collectAngle_i = 3;
                                    }
                                    extendo.CS = extendoController.extendoStatus.CYCLE;
                                    failsafe.reset();
                                    break;
                                }

                            }
                            status = STROBOT.BREAK_TRAJ;
                        }

                    } else {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_PARK;
                    }
                    break;
                }

                case BREAK_TRAJ:
                {
                    if((sensorPublisher.getSensorState() && drive.getPoseEstimate().getX()<=-22) || !drive.isBusy()) {
                        drive.breakFollowing();
                        limit = 1;
                        drive.setMotorPowers(0,0,0,0);
                        traj_failsafe = false;
                        failsafe.reset();
                        failsafe2.reset();
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES: {


                    if (park.seconds() < 26) {

                        if ((!r.pixelLeft.getState() || !r.pixelRight.getState())) {
                            failsafe2.reset();
                            collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i - 1);
                            tries = 0;
                            limit = 0.7;
                            traj_failsafe = false;
                            status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                        } else if (failsafe.seconds() > limit && tries <=2 && (r.pixelLeft.getState() && r.pixelRight.getState()) && traj_failsafe == false) {
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.FAIL_SAFE;
                        } else if ((tries > 2 || collectAngle.collectAngle_i == 0) && traj_failsafe == false) {

                            traj_failsafe = true;
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.RETRACT_AND_RERTY;
                        } else if(failsafe.seconds() > limit && tries <=4 && (r.pixelLeft.getState() && r.pixelRight.getState()) && traj_failsafe == true)
                        {
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.FAIL_SAFE;
                        }

                        else if (tries > 4 && !drive.isBusy()) {
                            traj_failsafe = false;
                            tries = 0;
                            status = STROBOT.GO_SCORE_CYCLE;
                        }
                    } else {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_SCORE_ON_GROUND;
                    }
                    break;
                }

                case FAIL_SAFE: {
                    if (failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE) {
                        limit = 0.7;
                        tries +=1;
                        failsafe.reset();
                        failsafe2.reset();
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    break;
                }

                case RETRACT_AND_RERTY:
                {
                    switch (nrcicluri)
                    {
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(Fail_safe_case_2);
                            status = STROBOT.COLLECT_VERIF_PIXLES;
                            break;
                        }
                        default:
                        {
                            drive.followTrajectorySequenceAsync(Fail_safe_case_1);
                            status = STROBOT.COLLECT_VERIF_PIXLES;
                            break;
                        }
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES_V2: {
                    if (park.seconds() < 26) {
                        if (!r.pixelLeft.getState() && !r.pixelRight.getState()) {
                            extendo.CS = extendoController.extendoStatus.RETRACTED;
                            extendo_timer.reset();
                            goscrcycl.reset();
                            tries = 0;
                            status = STROBOT.GO_SCORE_CYCLE;
                        } else if (failsafe.seconds() > limit && tries <=2 && (r.pixelLeft.getState() || r.pixelRight.getState()) && traj_failsafe == false) {
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.FAIL_SAFE;
                        } else if (tries > 2 && traj_failsafe == false) {

                            traj_failsafe = true;
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.RETRACT_AND_RERTY;
                        } else if(failsafe.seconds() > limit && tries <=4 && (r.pixelLeft.getState() || r.pixelRight.getState()) && traj_failsafe == true)
                        {
                            failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                            status = STROBOT.FAIL_SAFE;
                        }

                        else if (tries > 4 && !drive.isBusy()) {
                            traj_failsafe = false;
                            tries = 0;
                            status = STROBOT.GO_SCORE_CYCLE;
                        }

                    } else {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_SCORE_ON_GROUND;
                    }
                    break;
                }

                case GO_SCORE_CYCLE: {
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    switch (nrcicluri) {
                        case 0:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_RIGHT);
                                    break;
                            }
                            break;

                        case 1:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_RIGHT);
                                    break;
                            }
                            break;

                        case 2:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_RIGHT);
                                    break;
                            }
                            break;


                    }
                    extendo_timer.reset();
                    status = STROBOT.GO_SCORE_CYCLE_FUNNY_JAVA;


                    break;
                }

                case GO_PARK: {
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    drive.followTrajectorySequenceAsync(ParkBun2);
                    status = STROBOT.NOTHING;
                    break;
                }

                case GO_SCORE_ON_GROUND: {
                    if (goscrcycl.seconds() > 0.25) {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        drive.followTrajectorySequenceAsync(ParkBun2);
                        extendo_timer.reset();
                        status = STROBOT.GO_SCORE_CYCLE_FUNNY_JAVA_GROUND;
                    }
                    break;
                }

                case GO_SCORE_CYCLE_FUNNY_JAVA_GROUND: {
                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                    if (extendo_timer.seconds() > 0.2) {
                        r.collect.setPower(-0.8);
                    }
                    if (extendo_timer.seconds() > 0.35) {
                        r.collect.setPower(0);
                    }
                    if (extendo_timer.seconds() > 0.7) {
                        r.collect.setPower(0);
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        status = STROBOT.PREPARE_SCORE_CYCLE_ON_GROUND;
                    }

                    break;
                }

                case GO_SCORE_CYCLE_FUNNY_JAVA: {
                    if (extendo_timer.seconds() > 0.2) {
                        r.collect.setPower(-0.65);
                    }
                    if (extendo_timer.seconds() > 0.35) {
                        r.collect.setPower(1);
                    }
                    if(extendo_timer.seconds() > 0.7)
                    {
                        r.collect.setPower(0);
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        status = STROBOT.PREPARE_SCORE_CYCLE;
                    }

                    break;
                }

                case PREPARE_SCORE_CYCLE: {

                    if (redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 0) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri += 1;
                        stai.reset();
                        redFarAutoController.nr_cycle += 110;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case PREPARE_SCORE_CYCLE_ON_GROUND: {
                    r.collect.setPower(0);
                    if (redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 0) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_ON_GROUND;
                        nrcicluri += 1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE_GROUND;
                    }
                    break;
                }

                case SCORE_CYCLE: {
                    if(stai.seconds() > 0.35)
                    {
                        r.collect.setPower(-0.7);
                    }
                    if (redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_CYCLE_DONE && (drive.getPoseEstimate().getX() > 44.8 || !drive.isBusy())) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;
                        r.collect.setPower(0);
                        status = STROBOT.GO_COLLECT;
                    }

                    break;
                }

                case SCORE_CYCLE_GROUND: {
                    if (score.seconds() > 0.95) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP_YELLOW;
                        park_systems.reset();
                        status = STROBOT.PARK;
                    }

                    break;
                }

                case GO_COLLECT: {

                    if (redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.NOTHING) {
                        if (forced == false) {
                            switch (nrcicluri) {
                                case 1:
                                    switch (caz) {
                                        case 0:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_LEFT);
                                            break;
                                        case 1:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_CENTER);
                                            break;
                                        case 2:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_RIGHT);
                                            break;
                                    }
                                    break;

                                case 2:
                                    switch (caz) {
                                        case 0:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_LEFT);
                                            break;
                                        case 1:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_CENTER);
                                            break;
                                        case 2:
                                            drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_RIGHT);
                                            break;
                                    }
                                    break;

                                default:
                                    drive.followTrajectorySequenceAsync(ParkBun);
                                    break;

                            }

                            if ((nrcicluri < 3 && park.seconds() <= 24) || nrcicluri < 2) {
                                prepare_collect.reset();
                                status = STROBOT.PREPARE_COLLECT;
                            } else {
                                drive.followTrajectorySequenceAsync(ParkBun);
                                park_systems.reset();
                                status = STROBOT.PARK;
                            }
                        } else {
                            drive.followTrajectorySequenceAsync(ParkBun);
                            status = STROBOT.PARK;
                        }
                    }
                    break;
                }

                case PARK: {
                    if (park_systems.seconds() > 0.5) {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        status = STROBOT.NOTHING;
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
            extendo.update(r, extendopos, 1, voltage, sensorPublisher);
            collectAngle.update(r);
            redFarAutoController.update(r, lift, fourbar, clawAngle, clawFlip, collectAngle, door, extendo, latchLeft, latchRight);
            failsafecontroller.update(r, lift, fourbar, clawAngle, clawFlip, collectAngle, door, extendo, latchLeft, latchRight);
            droneLatch.update(r);


            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
//            telemetry.addData("robotcontroller", RedFarAutoController.CurrentStatus);
            // telemetry.addData("realpoz", r.extendoLeft.getCurrentPosition());
            // telemetry.addData("targetpoz", extendo.activePID.targetValue);
            telemetry.addData("targetAprilTag", Globals.desieredtag);
            //   telemetry.addData("did it see?", yellowPipeline.diditsee);
            //telemetry.addData("side", Globals.side);
            telemetry.addData("extedopower", r.extendoLeft.getPower());
            telemetry.addData("sensorPublisher", sensorPublisher.getSensorState());
            telemetry.addData("extendostatus", extendo.CS);
//

            loopTime = loop;

            telemetry.update();

        }
        sensorPublisher.stopPublishing();

    }
}
