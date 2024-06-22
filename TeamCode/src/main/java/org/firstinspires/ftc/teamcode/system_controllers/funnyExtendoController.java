package org.firstinspires.ftc.teamcode.system_controllers;






import static org.firstinspires.ftc.teamcode.system_controllers.funnyExtendoController.funnyExtendoStatus.AUTO;
import static org.firstinspires.ftc.teamcode.system_controllers.funnyExtendoController.funnyExtendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.funnyExtendoController.funnyExtendoStatus.RETRACTED;

import com.acmerobotics.dashboard.config.Config;



import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class funnyExtendoController {

    private boolean analog_value;

    public enum funnyExtendoStatus
    {
        INITIALIZE,
        RETRACTED,
        EXTENDED,

        AUTO,
    }

    public static double Kp = 0.0015;
    public static double Ki = 0.0002;
    public static double Kd = 0.002;



    SimplePIDController PID;


//
//    public static double Kp = 0.0031;
//    public static double Ki = 0;
//    public static double Kd = 0;



    public static double maxSpeed = 1;

    public funnyExtendoStatus CS = INITIALIZE, PS = INITIALIZE;



    public static double retracted = -2;

    public SimplePIDController pid;


    public funnyExtendoController(boolean analog_value)
    {
        pid = new SimplePIDController(Kp, Ki, Kd);
        this.analog_value = analog_value;

        pid.targetValue = retracted;
        pid.maxOutput = maxSpeed;
    }

    public void update(robotMap r, int position, double powerCap, double voltage)
    {

        double powerColectare = pid.update(position);
        powerColectare = Math.max(-powerCap, Math.min(powerColectare * 14 / voltage, powerCap));

        r.extendoLeft.setPower(powerColectare);
        r.extendoRight.setPower(powerColectare);


        if(CS == AUTO)
        {
            if(position < 650)
            {
                r.extendoLeft.setPower(1);
                r.extendoRight.setPower(1);
            } else if(!analog_value && position < 850)
            {
                r.extendoLeft.setPower(0.6);
                r.extendoRight.setPower(0.6);
            } else
            {
                pid.targetValue = position;
                r.extendoLeft.setPower(powerColectare);
                r.extendoRight.setPower(powerColectare);
            }
        }

        if(CS != PS || CS == INITIALIZE )
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    pid.targetValue = retracted;
                    pid.maxOutput = 1;
                    break;
                }

            }

            PS = CS;
        }

    }

}
