package org.firstinspires.ftc.teamcode.Auto.AutoControllers;


import static org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE;
import static org.firstinspires.ftc.teamcode.Auto.Recognition.Globals.yellow_drop.right;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Auto.BlueFar;

import org.firstinspires.ftc.teamcode.Auto.Recognition.Globals;


import org.firstinspires.ftc.teamcode.Auto.RedFarMTI;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import java.io.BufferedReader;


public class RedFarAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        PURPLE,
        PURPLE_REDFAR,
        PURPLE_funny,
        PURPLE_funnyred,
        PURPLE_DONE,

        PURPLE_DROP,
        PURPLE_DRIVE,
        PURPLE_DRIVE_NEAR,
        PURPLE_DROP_DONE,

        PURPLE_DROPnear,
        PURPLE_DRIVEnear,
        PURPLE_DROPnearnucentru,
        PURPLE_DRIVE_NEARnucentru,

        PURPLE_DROPnearred,
        PURPLE_DRIVEnearred,
        PURPLE_DROPnearnucentrured,
        PURPLE_DRIVE_NEARnucentrured,



        TRANSFER_BEGIN,
        TRANSFER_FOURBAR,
        TRANSFER_CLAW,
        TRANSFER_LATCHES,
        TRANSFER_DRIVE_POSE,
        TRANSFER_DONE,

        SCORE_YELLOW_BEGIN,
        TIMER_RESET,
        SCORE_YELLOW_LIFT,
        SCORE_YELLOW_CLAW,
        SCORE_YELLOW_DONE,

        SCORE_YELLOW_BEGIN_BLUE,
        TIMER_RESET_BLUE,
        SCORE_YELLOW_LIFT_BLUE,
        SCORE_YELLOW_CLAW_BLUE,
        SCORE_YELLOW_DONE_BLUE,
        LIFT_DOWN_BABY,
        LIFT_DOWN_BABY_OK,

        SCORE_YELLOW_BEGIN_BLUEnear,
        TIMER_RESET_BLUEnear,
        SCORE_YELLOW_LIFT_BLUEnear,
        SCORE_YELLOW_CLAW_BLUEnear,
        SCORE_YELLOW_NEAR_BAZI_NEBUN,

        COLLECT_PREPARE,
        COLLECT_PREPARE_CLAW,
        COLLECT_PREPARE_LIFT,
        COLLECT_PREPARE_DONE,

        SCORE_CYCLE_BEGIN,
        TIMER_CYCLE_RESET,
        SCORE_CYCLE_LIFT,
        SCORE_CYCLE_CLAW,
        SCORE_CYCLE_DONE,

        SCORE_ON_GROUND,

        FAIL_SAFE,
        FAIL_SAFE_CHECK_DISTANCE,
        FAIL_SAFE_HEADER,
        FAIL_SAFE_HEADER_TIMER_RESET,
        FAIL_SAFE_DONE,
        FAIL_SAFE_WRONG_HEADING,

        FAIL_SAFE_ONE_PIXEL,
        FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL,
        FAIL_SAFE_HEADER_ONE_PIXEL,
        FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL,
        FAIL_SAFE_DONE_ONE_PIXEL,
        FAIL_SAFE_WRONG_HEADING_ONE_PIXEL,
        FUNNY_JAVA,
        FUNNY_JAVA_ONE_PIXEL,


        LATCH_DROP,
        LATCH_DROP_UNGHI,
        LATCH_DROP_YELLOW,
        LATCH_DROP_YELLOW2,
        LATCH_DROP_YELLOWnear,
        Reset_claw_timer,
       Reset_claw_timer_done,
        LIFT_ALILBITUP,
        LIFT_ALILBITUP_DONE,

        DIMA_O_SUGE_done,
    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    ElapsedTime purple_drive = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();
    public ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime latches_timer = new ElapsedTime();
    ElapsedTime funny_timer = new ElapsedTime();
    ElapsedTime failsafe_header = new ElapsedTime();
    ElapsedTime funny_java = new ElapsedTime();
    ElapsedTime alibitup = new ElapsedTime();
    ElapsedTime lift_down_baby = new ElapsedTime();
    ElapsedTime ceva = new ElapsedTime();


    double distance_error = 14;
    double correct_distance = 0;
    public double nr_cycle = 350;
    public static int claw_caz = 0;
    public static boolean funny_or_not = false;
    public static boolean funny_or_notblue = false;



    public void update(robotMap r, liftController lift, fourbarController fourbar, clawAngleController clawAngle, clawFlipController clawFlip, collectAngleController collectAngle, doorController door, extendoController extendo, latchLeftController latchLeft, latchRightController latchRight)
    {
        liftController.cycle = nr_cycle;
        double[] rawReadingsExtendo = {36.5, 33.1, 34.6, 34.4, 36.5, 37.1, 38.6, 30.8, 23.3, 40.2, 25.5, 27.8, 28.9, 29.6, 27.6, 26.3, 24.9, 36.8, 35.6, 37.3};
        double[] actualDistancesExtendo = {27, 25, 26, 25.5, 26.5, 27.5, 29, 24, 20, 30, 21, 22, 23, 23.5, 22.5, 21.5, 20, 28, 28.5, 29.5};

        //calibratorExtendo = new DistanceSensorCalibrator(rawReadingsExtendo, actualDistancesExtendo);

//        double rawReading = r.extendoDistance.getDistance(DistanceUnit.CM);
//        double calibratedDistance = calibratorExtendo.calibrate(rawReading);

//        double distance = calibratedDistance;

        switch (CurrentStatus)
        {

            /**
             * PURPLE
             */

            case PURPLE:
            {

                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                clawAngle.CS = clawAngleController.clawAngleStatus.PURPLE;
//                if(BlueNear.caz == 1)
//                {
//                    clawAngle.clawAngle_i = 1;
//                } else



//                if(RedFarBun.caz == 0){
//                    clawAngleController.clawAngle_i = 0;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                }
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }

            case PURPLE_REDFAR:
            {

                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;

//                if(RedFarBun.caz == 0){
//                    clawAngleController.clawAngle_i = 0;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                }
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }


            case PURPLE_funny:
            {

                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                if(FUNNYBlueNear.caz == 1)
//                {
//                    clawAngle.clawAngle_i = 1;
//                } else
//                {
//                    clawAngle.clawAngle_i = 2;
//                }
//                if(RedFarBun.caz == 0){
//                    clawAngleController.clawAngle_i = 0;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                }
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }

            case PURPLE_funnyred:
            {

                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                if(FUNNYRedNear.caz == 1)
//                {
//                    clawAngle.clawAngle_i = 1;
//                } else
//                {
//                    clawAngle.clawAngle_i = 2;
//                }
//                if(RedFarBun.caz == 0){
//                    clawAngleController.clawAngle_i = 0;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                }
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }

            case PURPLE_DROP:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE;
                break;
            }

            case PURPLE_DROPnear:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE_NEAR;
                break;
            }

            case PURPLE_DROPnearnucentru:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE_NEARnucentru;
                break;
            }

            case PURPLE_DROPnearred:
            {
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE_NEAR;
                break;
            }

            case PURPLE_DROPnearnucentrured:
            {
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE_NEARnucentru;
                break;
            }

            case PURPLE_DRIVE:
            {

                if(purple_drive.seconds() > 0.6)
                { fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                clawAngle.clawAngle_i = 2;
                CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }

            case PURPLE_DRIVE_NEAR:
            {
                if(purple_drive.seconds() > 0.15)
                {
                    lift.CS = liftController.liftStatus.YELLOW_NEAR;
                }
                if(purple_drive.seconds() > 0.25)
                { fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }

            case PURPLE_DRIVE_NEARnucentru:
            {
                if(purple_drive.seconds() > 0.15)
                {
                    CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }

            case PURPLE_DRIVEnear:
            {
                if(purple_drive.seconds() > 0.4)
                { fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }

            /**
             * TRANSFER
             */

            case TRANSFER_BEGIN:
            {
                extendo.CS = extendoController.extendoStatus.TRANSFER;
                lift.CS = liftController.liftStatus.TRANSFER;
                if(r.extendoLeft.getCurrentPosition() < 5)
                {
                    door.CS = doorController.doorStatus.OPENED;
                    fourbar_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_FOURBAR;
                }
                break;
            }

            case TRANSFER_FOURBAR:
            {
                if(fourbar_timer.seconds() > 0.35)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    fourbar.CS = fourbarController.fourbarStatus.COLLECT;
                   CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                latches_timer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_LATCHES;
                break;
            }

            case TRANSFER_LATCHES:
            {
                if(latches_timer.seconds() > 0.25)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
                    latchRight.CS = latchRightController.LatchRightStatus.SECURED;
                    fourbar_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_DRIVE_POSE;
                }
                break;
            }

            case TRANSFER_DRIVE_POSE:
            {
                if(fourbar_timer.seconds() > 0.2)
                {
                    lift.CS = liftController.liftStatus.DOWN;
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.225)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.23)
                {
                  CurrentStatus = autoControllerStatus.TRANSFER_DONE;
                }
                break;

            }

            /**
             * SCORE YELLOW
             */

            case SCORE_YELLOW_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_RESET;
                break;
            }

            case SCORE_YELLOW_BEGIN_BLUE:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_RESET_BLUE;
                break;
            }

            case SCORE_YELLOW_BEGIN_BLUEnear:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_RESET_BLUEnear;
                break;
            }

            case TIMER_RESET_BLUEnear:
            {
                funny_timer.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_LIFT_BLUEnear;
                break;
            }


            case TIMER_RESET:
            {
                funny_timer.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_LIFT;
                break;
            }

            case TIMER_RESET_BLUE:
            {
                funny_timer.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_LIFT_BLUE;
                break;
            }

            case SCORE_YELLOW_LIFT:
            {
                if(funny_timer.seconds() > 0.3)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    lift.pid = 1;
                    lift.CS = liftController.liftStatus.PRELOAD_YELLOW;
                    clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_CLAW;
                }
                break;
            }

            case SCORE_YELLOW_LIFT_BLUE:
            {
                if(funny_timer.seconds() > 0.3)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    clawAngle.clawAngle_i = 5;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                    lift.pid = 1;
                    clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                    lift.CS = liftController.liftStatus.PRELOAD_YELLOW;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_CLAW;
                }
                break;
            }

            case SCORE_YELLOW_LIFT_BLUEnear:
            {
                if(funny_timer.seconds() > 0.3)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    lift.pid = 1;
                    lift.CS = liftController.liftStatus.YELLOW_NEAR;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_CLAW_BLUEnear;
                }
                break;
            }

            case SCORE_YELLOW_CLAW:
            {
                if(claw_timer.seconds() > 0.2)
                {
                    clawAngle.CS = clawAngleController.clawAngleStatus.AUTO;
                    CurrentStatus = SCORE_YELLOW_DONE;
                    }
                break;
            }

            case LIFT_DOWN_BABY:
            {
                if(Globals.waitformiddle == true)
                {  lift.CS = liftController.liftStatus.YELLOW_MID;
                lift_down_baby.reset();
                CurrentStatus = autoControllerStatus.LIFT_DOWN_BABY_OK;}
                break;
            }

            case LIFT_DOWN_BABY_OK:
            {
                if(lift_down_baby.seconds() > 0.3)
                { CurrentStatus =  SCORE_YELLOW_DONE;}
                break;
            }



            case SCORE_YELLOW_CLAW_BLUE:
            {
                if(claw_timer.seconds() > 0.2)
                {
                    switch (claw_caz)
                    {
                        case 0:
                        { clawAngle.clawAngle_i = 1; //1
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }

                        case 1:
                        {
                            if(funny_or_notblue == false)
                            {  clawAngle.clawAngle_i = 0;}
                            else
                            {
                                clawAngle.clawAngle_i = 1;
                            }
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }

                        case 2:
                        {
                            clawAngle.clawAngle_i = 0;
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }
                    }
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_DONE_BLUE;
                }
                break;
            }

            case SCORE_YELLOW_CLAW_BLUEnear:
            {
                if(claw_timer.seconds() > 0.2)
                {
//                    switch (BlueNear.caz)
//                    {
//                        case 0:
//                        { clawAngle.clawAngle_i = 1; //1
//                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                            break;
//                        }
//
//                        case 1:
//                        { clawAngle.clawAngle_i = 2;
//                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                            break;
//                        }
//
//                        case 2:
//                        {
//                            clawAngle.clawAngle_i = 0;
//                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                            break;
//                        }
//                    }
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_DONE_BLUE;
                }
                break;
            }


            case LATCH_DROP_UNGHI:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                alibitup.reset();
                        CurrentStatus = autoControllerStatus.LIFT_ALILBITUP;
                        break;
            }

            case LATCH_DROP:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    alibitup.reset();
                CurrentStatus = autoControllerStatus.NOTHING;
                break;
            }

            case LATCH_DROP_YELLOW:
            {
                if(Globals.yellow_drop_side == right)
                {                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;}
                else
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;

                }
                alibitup.reset();
                CurrentStatus = autoControllerStatus.LATCH_DROP_YELLOW2;
                break;
            }

            case LATCH_DROP_YELLOW2:
            {
                if(alibitup.seconds() > 0.1)
                {  latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                CurrentStatus = autoControllerStatus.NOTHING;}
                break;
            }

            case LATCH_DROP_YELLOWnear:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                alibitup.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_NEAR_BAZI_NEBUN;
                break;
            }

            case SCORE_YELLOW_NEAR_BAZI_NEBUN:
            {
                if(alibitup.seconds() > 0.1)
                {
                    fourbar.CS = fourbarController.fourbarStatus.SCORE_BAZI;
                    clawFlip.CS = clawFlipController.clawFlipStatus.YELLO_BAZI;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.DIMA_O_SUGE_done;
                }
                break;
            }

            case Reset_claw_timer:
            {
                if(claw_timer.seconds() > 0.1)
                {
                    CurrentStatus = autoControllerStatus.Reset_claw_timer_done;
                }
                break;
            }

            /**
             * PREPARECOLLECT
             */

            case COLLECT_PREPARE:
            {
                fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                claw_timer.reset();
                CurrentStatus = autoControllerStatus.COLLECT_PREPARE_CLAW;
                break;
            }

            case COLLECT_PREPARE_CLAW:
            {
                if(claw_timer.seconds() > 0.1)
                {
                    clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                }
                if(claw_timer.seconds() > 0.2)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    funny_timer.reset();
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_LIFT;
                }
                break;
            }

            case COLLECT_PREPARE_LIFT:
            {
                if(funny_timer.seconds() > 0.2)
                {
                    lift.pid = 0;
                    lift.CS = liftController.liftStatus.DOWN;
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_DONE;
                }
                break;
            }

            /**
             * SCORE_CYCLE
             */

            case SCORE_CYCLE_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                lift.pid = 1;
                lift.CS = liftController.liftStatus.CYCLE;
                clawAngle.clawAngle_i = 2;
                clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_CYCLE_RESET;
                break;
            }

            case TIMER_CYCLE_RESET:
            {
                funny_timer.reset();
                CurrentStatus = autoControllerStatus.SCORE_CYCLE_LIFT;
                break;
            }

            case SCORE_CYCLE_LIFT:
            {
                if(funny_timer.seconds() > 0.15)

                {clawAngle.clawAngle_i = 6;

                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                door.CS = doorController.doorStatus.CLOSED;
                CurrentStatus = autoControllerStatus.SCORE_CYCLE_DONE;}
                break;
            }

            case SCORE_ON_GROUND:
            {
                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                CurrentStatus = autoControllerStatus.NOTHING;
                break;
            }


            /**
             * FAIL_SAFE
             */

            case FAIL_SAFE:
            {
                extendo.CS = extendoController.extendoStatus.FAIL_SAFE;
                failsafe_header.reset();
                CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
                break;
            }

//            case FAIL_SAFE_CHECK_DISTANCE:
//            {
//                if(distance > distance_error && (r.pixelRight.getState() && r.pixelLeft.getState()))
//                {
//                    extendo.x += 10;
//                    funny_java.reset();
//                    CurrentStatus = autoControllerStatus.FUNNY_JAVA;
//                }
//                else
//                {
//                    if(!r.pixelRight.getState() || !r.pixelLeft.getState())
//                    {
//
//                        //  extendo.cycle = extendo.failsafe + extendo.x;
//                        //  extendo.CS = extendoController.extendoStatus.CYCLE;
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE;
//                    }
//                    else
//                    {
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
//                    }
//                }
//                break;
//            }
//
//            case FUNNY_JAVA:
//            {
//                if(funny_java.seconds() > 0.01)
//                {
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE;
//                }
//                break;
//            }

            case FAIL_SAFE_HEADER:
            {
                if(failsafe_header.seconds() > 0.2 && collectAngle.collectAngle_i >= 1)
                {
                    collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                    collectAngle.collectAngle_i -= 1;
                    extendo.CS = extendoController.extendoStatus.CYCLE;
                    funny_java.reset();
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE;
                }

                break;
            }

            case FAIL_SAFE_HEADER_TIMER_RESET:
            {
                if(failsafe_header.seconds() > 0.01)
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
                }
                break;
            }


//            case FAIL_SAFE_ONE_PIXEL:
//            {
//                if(distance > distance_error)
//                {
//                    extendo.CS = extendoController.extendoStatus.FAIL_SAFE;
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL;
//                } else
//                {
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
//                }
//                break;
//
//            }
//
//            case FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL:
//            {
//                if(distance > distance_error && (r.pixelRight.getState() || r.pixelLeft.getState()))
//                {
//                    extendo.x += 30;
//                    funny_java.reset();
//                    CurrentStatus = autoControllerStatus.FUNNY_JAVA_ONE_PIXEL;
//                }
//                else
//                {
//                    if(!r.pixelRight.getState() && !r.pixelLeft.getState())
//                    {
//
//                        //  extendo.cycle = extendo.failsafe + extendo.x;
//                        // extendo.CS = extendoController.extendoStatus.CYCLE;
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE_ONE_PIXEL;
//                    }
//                    else
//                    {
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
//                    }
//                }
//                break;
//            }

            case FUNNY_JAVA_ONE_PIXEL:
            {
                if(funny_java.seconds() >0.01)
                {
                    CurrentStatus =autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL;
                }
                break;
            }

            case FAIL_SAFE_HEADER_ONE_PIXEL:
            {
                if((r.pixelRight.getState() || r.pixelLeft.getState()) && collectAngle.collectAngle_i != 0)
                {
                    collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
                    failsafe_header.reset();
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL;
                } else if(!r.pixelRight.getState() && !r.pixelLeft.getState())
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE_ONE_PIXEL;
                } else
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_WRONG_HEADING_ONE_PIXEL;
                }

                break;
            }

            case FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL:
            {
                if(failsafe_header.seconds() > 0.5)
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
                }
                break;
            }

            case LIFT_ALILBITUP:
            {
                if(alibitup.seconds() > 0.2)
                {lift.CS = liftController.liftStatus.ALILBIT_UP;}
                if(alibitup.seconds() > 0.35)
                { CurrentStatus = autoControllerStatus.LIFT_ALILBITUP_DONE;}
                break;
            }

//            case SCORE_CYCLE_CLAW:
//            {
//                if(claw_timer.seconds() > 0.05)
//                {
//                    clawAngle.clawAngle_i = 5;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
//                    CurrentStatus = autoControllerStatus.SCORE_CYCLE_DONE;
//                }
//                break;
//            }



        }
        PreviousStatus = CurrentStatus;
    }
}

