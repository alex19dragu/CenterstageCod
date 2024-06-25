package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.CYCLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.FAIL_SAFE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.PURPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RERTRY_PURPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
import static org.firstinspires.ftc.teamcode.system_controllers.funnyExtendoController.funnyExtendoStatus.AUTO;


import com.acmerobotics.dashboard.config.Config;


//import org.firstinspires.ftc.teamcode.Auto.RedFarBun;
//import org.firstinspires.ftc.teamcode.Auto.BlueFar;
//import org.firstinspires.ftc.teamcode.Auto.FUNNYBlueFar;
//import org.firstinspires.ftc.teamcode.Auto.RedFar;
import org.firstinspires.ftc.teamcode.Auto.Recognition.Globals;
import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class extendoController {

    private boolean analog_value;
     private SensorPublisher sensorPublisher;

    public enum extendoStatus {
        INITIALIZE,
        RETRACTED,
        EXTENDED,
        EXTENDED_NEAR,

        PURPLE,
        PURPLEredfar,

        CYCLE,
        DRIVE,
        FAIL_SAFE,
        TRANSFER,
        RETRY,

        RERTRY_PURPLE,
        FAIL_SAFE_PURPLE,

        RERTRY_PURPLEredfar,
        FAIL_SAFE_PURPLEredfar,

        FAIL_SAFE_NEAR,
        RETRY_NEAR,
    }

    // PID constants for extension
    public static double Kp_extend = 0.004;
    public static double Ki_extend = 0.0002;
    public static double Kd_extend = 0.001;

    public static double Kp_purple = 0.0015;
    public static double Ki_purple = 0.0002;
    public static double Kd_purple = 0.002;

    // PID constants for retraction
    public static double Kp_retract = 0.015;
    public static double Ki_retract = 0;
    public static double Kd_retract = 0;

    public static double Kp_drive = 0.004;
    public static double Ki_drive = 0.0002;
    public static double Kd_drive = 0.001;

    public static double Kp_short = 0.017;
    public static double Ki_short = 0.002;
    public static double Kd_short = 0;




    SimplePIDController extendoPIDExtend;
    SimplePIDController extendoPIDRetract;
    SimplePIDController extendoPIDDrive;
    SimplePIDController SHORTPID;
    SimplePIDController purplepid;

//
//    public static double Kp = 0.0031;
//    public static double Ki = 0;
//    public static double Kd = 0;


    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    //   SimplePIDController extendoPID = null;

    public static double CurrentPosition = 0;
    public static double retracted = -5;
    public static double extended = 800;
    public static double drive = 800;
    public static double failsafe = 625;
    public static double purple[] = {750, 315, 0};
    public static double purpleredfar[] = {0, 280, 505};
    public static double cycle = 840;
    public static double cycle_far = 835;
    public static double x = 10;
    public static int caz = 0;
    public static double transfer = -20;
    public static double retry = 800;
    public static double max = 900;
    public static double purple_max = 550;
    public static double extendedmax = 750;

    public static double retry_near = 800;
    public static double failsafe_near = 800;

    public static double retry_purple[] = {445, 230, 0};
    public static double fail_purple[] = {420, 150, 0};

    public static double retry_purpleredfar[] = {0, 230, 445};
    public static double fail_purpleredfar[] = {0, 150, 420};

    //    public static double retry_purple[] = {0, 290, 580};
//    public static double fail_purple[] = {0, 150, 480};
    public SimplePIDController activePID;

    public static double extend_multiply_index = 0;

    public extendoController() {
        extendoPIDExtend = new SimplePIDController(Kp_extend, Ki_extend, Kd_extend);
        extendoPIDRetract = new SimplePIDController(Kp_retract, Ki_retract, Kd_retract);
        extendoPIDDrive = new SimplePIDController(Kp_drive, Ki_drive, Kd_drive);
        SHORTPID = new SimplePIDController(Kp_short, Ki_short, Kd_short);
        purplepid = new SimplePIDController(Kp_purple, Ki_purple, Kd_purple);

        // Initially, we can set to retract as a default or based on your initial state
        extendoPIDExtend.targetValue = retracted; // Assuming you start with retraction
        extendoPIDExtend.maxOutput = maxSpeed;

        extendoPIDRetract.targetValue = retracted;
        extendoPIDRetract.maxOutput = maxSpeed;

        extendoPIDDrive.targetValue = retracted;
        extendoPIDDrive.maxOutput = maxSpeed;
    }

    public void update(robotMap r, int position, double powerCap, double voltage, SensorPublisher sensorPublisher) {
         this.sensorPublisher = sensorPublisher;
         analog_value = sensorPublisher.getSensorState();

        //  SimplePIDController activePID;
        switch (CS) {
            case INITIALIZE:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED: // Define your conditions
                activePID = extendoPIDDrive;
                break;
            case RETRACTED:
                activePID = extendoPIDRetract;
                break;
            case PURPLE:
                activePID = extendoPIDExtend;
                break;
            case PURPLEredfar:
                activePID = extendoPIDExtend;
                break;
            case CYCLE:
                activePID = extendoPIDExtend;
                break;
            case DRIVE:
                activePID = extendoPIDDrive;
                break;
            case FAIL_SAFE:
                activePID = SHORTPID;
                break;
            case RETRY:
                activePID = SHORTPID;
                break;
            case FAIL_SAFE_PURPLE:
                activePID = SHORTPID;
                break;
            case RERTRY_PURPLE:
                activePID = SHORTPID;
                break;
            default:
                activePID = extendoPIDRetract; // Default to the first one or any you prefer
                break;
        }

        // Use the active PID controller for calculations
        double powerColectare = activePID.update(position);
        powerColectare = Math.max(-powerCap, Math.min(powerColectare * 14 / voltage, powerCap));

//        if (r.leftStopper.getState() || r.rightStopper.getState()) {
//            analog_value = true;
//        } else {
//            analog_value = false;
//        }

        if(CS != CYCLE && CS != PURPLE && CS != RERTRY_PURPLE)
        { r.extendoLeft.setPower(powerColectare);
        r.extendoRight.setPower(powerColectare);}

        analog_value = sensorPublisher.getSensorState();

//        if ( CS == DRIVE) {
//            activePID.targetValue = max;
//            if (analog_value) {
//               max = position;
//            }
//            else
//            {
//                max = 800;
//            }
//        }

        if(CS == PURPLE || CS == RERTRY_PURPLE)
        {
            activePID.targetValue = purple_max;
           if(!Globals.is_left)
           {  if (analog_value) {
               r.extendoLeft.setPower(0);
               r.extendoRight.setPower(0);
            }
            else
            {  r.extendoLeft.setPower(powerColectare);
                r.extendoRight.setPower(powerColectare);
                purple_max = 550;
            }}
           else
           {
               purple_max = 0;
           }
        }


        if(CS ==CYCLE)
        {
            activePID.targetValue = extendedmax;

            if (analog_value) {
                r.extendoLeft.setPower(0);
                r.extendoRight.setPower(0);
            }
            else
            {
                r.extendoLeft.setPower(powerColectare);
                r.extendoRight.setPower(powerColectare);
            }
        }


        if (CS == RETRACTED || CS == INITIALIZE) {
            activePID.targetValue = retracted;
        }

//          if(CS == SENSOR && CurrentPosition >= auto[ciclu] - 10)
//          {
//              extendoPID.targetValue = difference;
//              extendoPID.maxOutput = 0.6;
//          }

        if (CS != PS || CS == INITIALIZE || CS == EXTENDED || CS == RETRACTED || CS == PURPLE || CS == FAIL_SAFE || CS == DRIVE || CS == CYCLE) {
            switch (CS) {
                case INITIALIZE: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED: {
                    activePID.targetValue = extendedmax;
                    activePID.maxOutput = 0.8;
                    break;
                }

                case EXTENDED_NEAR: {
                    activePID.targetValue = 990;
                    activePID.maxOutput = 1;
                    break;
                }

                case RETRY: {
                    activePID.targetValue = retry;
                    activePID.maxOutput = 1;
                    break;
                }

                case RETRACTED: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case PURPLE: {
                    activePID.targetValue = purple_max;
                    activePID.maxOutput = 0.45;
                    //CS = SENSOR;
                    break;
                }

                case PURPLEredfar: {
                    activePID.targetValue = purpleredfar[caz];
                    activePID.maxOutput = 0.9;
                    //CS = SENSOR;
                    break;
                }

                case CYCLE: {
                    activePID.targetValue = extendedmax;
                    activePID.maxOutput = 1;
                    break;
                }

                case DRIVE: {
                    activePID.targetValue = max;
                    activePID.maxOutput = 1;
                    break;
                }

                case FAIL_SAFE: {
                    activePID.targetValue = failsafe;
                    activePID.maxOutput = 1;
                    break;
                }

                case TRANSFER: {
                    activePID.targetValue = transfer;
                    activePID.maxOutput = 1;
                    break;
                }

                case RERTRY_PURPLE: {
                    activePID.targetValue = purple_max;
                    activePID.maxOutput = 1;
                    break;
                }

                case FAIL_SAFE_PURPLE: {
                    activePID.targetValue = fail_purple[caz];
                    activePID.maxOutput = 1;
                    break;
                }

                case RERTRY_PURPLEredfar: {
                    activePID.targetValue = retry_purpleredfar[caz];
                    activePID.maxOutput = 1;
                    break;
                }

                case FAIL_SAFE_PURPLEredfar: {
                    activePID.targetValue = fail_purpleredfar[caz];
                    activePID.maxOutput = 1;
                    break;
                }

                case FAIL_SAFE_NEAR: {
                    activePID.targetValue = failsafe_near;
                    activePID.maxOutput = 1;
                    break;
                }

                case RETRY_NEAR: {
                    activePID.targetValue = retry_near;
                    activePID.maxOutput = 1;
                    break;
                }
            }

            PS = CS;
        }

    }

}