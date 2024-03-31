package org.firstinspires.ftc.teamcode.Auto.P2P;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Photon
@TeleOp(name="testpidtopoint", group="testpidtopoint")
public class testpidtopoint extends LinearOpMode {

    @Override
    public void runOpMode() {
        funnypidtopoint controller = new funnypidtopoint(hardwareMap);

        Pose targetPose = new Pose(0, 0, Math.toRadians(0));
        Pose currentPose = new Pose(0, 0, Math.toRadians(0));

      //  controller.drive.setPose(currentPose);

        waitForStart();

     controller.execute(50, -50, Math.toRadians(180));

        while (opModeIsActive() ) {


            Pose2d currentpose = controller.drive.getPoseEstimate();


//            double errorX = targetPose.getX() - currentpose.getX();
//            double errorY = targetPose.getY() - currentpose.getY();
//            double errorHeading = targetPose.heading - currentpose.getHeading();

            // Update telemetry with the current pose and error values
           // telemetry.addData("Current Pose", currentpose.toString());
            telemetry.addData("Error X",  controller.drive.returnPose().x);
            telemetry.addData("Error Y",  controller.drive.returnPose().y);
            telemetry.addData("Error Y", Math.toDegrees(controller.drive.returnPose().heading) );
           // telemetry.addData("Error Heading (degrees)", Math.toDegrees(errorHeading));
            telemetry.update();

            controller.drive.update();
            // Optional: Adjust the loop frequency if necessary
            sleep(50);
        }

        // Optionally, stop motors after the movement is finished
        controller.stopMotors();

        // Indicate that the target has been reached
        telemetry.addData("Status", "Target Reached");
        telemetry.update();
    }
}
