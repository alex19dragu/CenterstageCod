//package org.firstinspires.ftc.teamcode.Auto.Recognition;
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.AnalogSensor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.teamcode.Auto.urm09_customdriver;
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//
//import java.util.concurrent.Executors;
//import java.util.concurrent.ScheduledExecutorService;
//import java.util.concurrent.TimeUnit;
//
//public class DistanceSensorPublisher {
//    private urm09_customdriver left;
//    private urm09_customdriver right;
//    private ScheduledExecutorService scheduler;
//    private int leftDistance;
//    private int rightDistance;
//
//    public DistanceSensorPublisher(robotMap hardwareMap) {
//          left = hardwareMap.left;
//        right = hardwareMap.right;
//        scheduler = Executors.newScheduledThreadPool(1);
//    }
//
//    public void startPublishing() {
//        scheduler.scheduleAtFixedRate(() -> {
//            leftDistance = left.getDistanceCM();
//            rightDistance = right.getDistanceCM();
//            // Notify subscribers here (could use an event bus or similar mechanism)
//        }, 0, 10, TimeUnit.MILLISECONDS); // Adjust the period as necessary
//    }
//
//    public int getLeftDistance() {
//        return left.getDistanceCM();
//    }
//
//    public int getRightDistance()
//    {
//
//    }
//
//    public void stopPublishing() {
//        scheduler.shutdown();
//    }
//}
