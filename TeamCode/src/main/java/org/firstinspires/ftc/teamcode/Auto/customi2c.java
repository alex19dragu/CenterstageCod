//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Auto.Recognition.DistanceSensorPublisher;
//import org.firstinspires.ftc.teamcode.Auto.urm09_customdriver;
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//
//@TeleOp(name = "URM09 Test", group = "Sensor")
//public class customi2c extends LinearOpMode {
//   // private urm09_customdriver urm09;
//   double loopTime = 0;
//
//    @Override
//    public void runOpMode() {
//        robotMap r = new robotMap(hardwareMap);
//
//        DistanceSensorPublisher distanceSensorPublisher = new DistanceSensorPublisher(r);
//
//        Pose2d pose2d = new Pose2d(0, 0 , Math.PI);
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        distanceSensorPublisher.startPublishing();
//        // Set the measurement range to 300 cm and automatic mode
//
//        while (opModeIsActive()) {
//            double loop = System.nanoTime();
//          //  float temperature = urm09.getTemperature();
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            loopTime = loop;
//telemetry.addData("pose", distanceSensorPublisher.correctY(pose2d));
//           // telemetry.addData("Temperature (C)", temperature);
//            telemetry.update();
//
//        }
//    }
//}
