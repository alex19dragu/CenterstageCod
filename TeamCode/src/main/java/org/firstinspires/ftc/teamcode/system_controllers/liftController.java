package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.opmode.BackAndForth;
import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class liftController {

    public enum liftStatus
    {
        INITIALIZE,
        DOWN,
        UP,
        CYCLE,
        HANG,

        PRELOAD_YELLOW,
        TRANSFER,
    }

    public double CurrentSpeed = 0;

    /**
     * UP
     */


    public static double PAUTO = 0.002;
    public static double IAUTO = 0;
    public static double DAUTO = 0.001;
    //
    public static double PDRIVE = 0.0075;
    public static double IDRIVE = 0.0012;
    public static double DDRIVE = 0.001;

    public static double PDOWN = 0.0075;
    public static double IDOWN = 0;
    public static double DDOWN = 0;

    public double pid = 0;

    public double Kg = 0;
    public double maxSpeedUp = 1;

    public static liftStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController LiftPID_AUTO = null;
    SimplePIDController LiftPIDDOWN = null;
    SimplePIDController LiftPID_DRIVE = null;



    public static double base = -5;

    public static double down = 108;

    public static int i_up = 0;
    public static double i_multiplication = 108;

    public static double hang = 740;

    public int CurrentPosition = 0;
    public static double transfer = -30;

    public liftController()
    {
        LiftPID_AUTO = new SimplePIDController(PAUTO,IAUTO,DAUTO);
        LiftPID_AUTO.targetValue = base;
        LiftPID_AUTO.maxOutput = maxSpeedUp;

        LiftPIDDOWN = new SimplePIDController(PDOWN,IDOWN,DDOWN);
        LiftPIDDOWN.targetValue = base;
        LiftPIDDOWN.maxOutput = maxSpeedUp;

        LiftPID_DRIVE = new SimplePIDController(PDRIVE,IDRIVE,DDRIVE);
        LiftPID_DRIVE.targetValue = base;
        LiftPID_DRIVE.maxOutput = maxSpeedUp;
    }

    public void update(robotMap r, int position, double voltage)
    {
        SimplePIDController activePID;
        switch (CS) {
            case UP: // Define your conditions
                activePID = LiftPID_DRIVE;
                break;
            case DOWN:
                activePID = LiftPIDDOWN;
                break;
            case CYCLE:
                activePID = LiftPID_AUTO;
                break;
            case PRELOAD_YELLOW:
                activePID = LiftPID_AUTO;
                break;
            case HANG:
                activePID = LiftPID_DRIVE;
                break;
            default:
                activePID = LiftPIDDOWN; // Default to the first one or any you prefer
                break;
        }

        CurrentPosition = position;
        double powerLift = activePID.update(position) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift* 14 / voltage,1));
        CurrentSpeed=powerLift;


        r.lift.setPower(powerLift);

        if(CS == liftStatus.UP) activePID.targetValue = down + i_up * i_multiplication;

if(CS == INITIALIZE || CS == DOWN)
    activePID.targetValue = base;

        if(CS != PS || CS == INITIALIZE )
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    activePID.targetValue = base;
                    break;
                }

                case UP:
                {
                    activePID.targetValue = down + i_up * i_multiplication;
                    break;
                }

                case DOWN:
                {
                    activePID.targetValue = base;
                    break;
                }

                case PRELOAD_YELLOW:
                {
                    activePID.targetValue = 300;
                    break;
                }

                case CYCLE:
                {
                    activePID.targetValue = 370;
                    break;
                }

                case HANG:
                {
                    activePID.targetValue = hang;
                    break;
                }

                case TRANSFER:
                {
                    activePID.targetValue = transfer;
                    break;
                }
            }
        }

        PS = CS;
    }

}