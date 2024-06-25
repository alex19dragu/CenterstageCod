package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Created by Dryw Wade
 *
 * OpMode for testing Adafruit's MCP9808 temperature sensor driver
 */
@TeleOp(name = "lol", group = "Tests")
public class customi2c extends LinearOpMode
{
    private urm09_customdriver tempSensor;

    public void runOpMode() throws InterruptedException
    {
        tempSensor = hardwareMap.get(urm09_customdriver.class, "tempSensor");
        tempSensor.setMeasureRange(300);

       boolean isinit = tempSensor.doInitialize();

        telemetry.addData("isinit", isinit);

        // Uncomment to use parameter version of driver class. This will require you to respecify
        // the sensor type from MCP9808 to MCP9808Params
//        MCP9808Params.Parameters parameters = new MCP9808Params.Parameters();
//        parameters.hysteresis = MCP9808Params.Hysteresis.HYST_1_5;
//        parameters.alertControl = MCP9808Params.AlertControl.ALERT_ENABLE;
//        tempSensor.initialize(parameters);


        waitForStart();

        while(opModeIsActive())
        {

                telemetry.addData("idk", tempSensor.getDistanceCM());
            telemetry.update();
         //   idle();
        }
    }
}