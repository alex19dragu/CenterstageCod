package org.firstinspires.ftc.teamcode.system_controllers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.globals.robotMap;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class SensorPublisher {
    private DigitalChannel analogSensor1;
    private DigitalChannel analogSensor2;
    private ScheduledExecutorService scheduler;
    private boolean sensorState1;
    private boolean sensorState2;

    public SensorPublisher(robotMap hardwareMap) {
        analogSensor1 = hardwareMap.leftStopper;
        analogSensor2 = hardwareMap.rightStopper;
        scheduler = Executors.newScheduledThreadPool(1);
    }

    public void startPublishing() {
        scheduler.scheduleAtFixedRate(() -> {
                sensorState1 = analogSensor1.getState();
                sensorState2 = analogSensor2.getState();
            // Notify subscribers here (could use an event bus or similar mechanism)
        }, 0, 10, TimeUnit.MILLISECONDS); // Adjust the period as necessary
    }

    public boolean getSensorState() {
        return sensorState1 || sensorState2;
    }

    public void stopPublishing() {
        scheduler.shutdown();
    }
}
