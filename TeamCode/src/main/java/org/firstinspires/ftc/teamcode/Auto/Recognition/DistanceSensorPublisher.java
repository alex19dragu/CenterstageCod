//package org.firstinspires.ftc.teamcode.Auto.Recognition;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
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
//    private double y = 0;
//
//    public DistanceSensorPublisher(robotMap hardwareMap) {
//         // left = hardwareMap.left;
//        right = hardwareMap.right;
//        scheduler = Executors.newScheduledThreadPool(1);
//    }
//
//    public void startPublishing() {
//        scheduler.scheduleAtFixedRate(() -> {
//           // leftDistance = left.getDistanceCM();
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
//        return right.getDistanceCM();
//    }
//
//    public double convertToFiledPose(double x)
//    {
//        x /= 2.54;
//        return 70.5 - x;
//    }
//
//    public Pose2d correctY(Pose2d pose2d)
//    {
//        y = getRightDistance();
//        y += 16.2;
//
//        return new Pose2d(pose2d.getX(), convertToFiledPose(y), pose2d.getHeading());
//    }
//
//    public void stopPublishing() {
//        scheduler.shutdown();
//    }
//}
