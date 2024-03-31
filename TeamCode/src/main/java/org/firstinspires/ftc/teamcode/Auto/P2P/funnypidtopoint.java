package org.firstinspires.ftc.teamcode.Auto.P2P;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;

@Photon
public class funnypidtopoint {
    public SampleMecanumDrive drive;
    private PIDFController pidX, pidY, pidHeading;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime stable = new ElapsedTime();

    private final double MAX_TRANSLATIONAL_SPEED = 1.0;
    private final double MAX_ROTATIONAL_SPEED = 1.0;
   // private final double K_STATIC = 1.0;
    private static final double ALLOWED_TRANSLATIONAL_ERROR = 0.35;
    private static final double ALLOWED_HEADING_ERROR = Math.toRadians(2);
    private static final long STABLE_MS = 250;
    private static final long DEAD_MS = 350;

    public boolean hasReachedProximity;

    public funnypidtopoint(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        pidX = new PIDFController(0.097, 0.003, 0.012, 0);
        pidY = new PIDFController(0.125, 0.001, 0.011, 0);
        pidHeading = new PIDFController(0.9, 0, 0.032, 0);
    }



    public void execute(Pose targetPose) {
        hasReachedProximity = false;

        timer.reset();
        stable.reset();

        while (!isFinished(targetPose)) {
            Pose robotPose = drive.returnPose();
            double[] motorPowers = getMotorPowers(robotPose, targetPose);
            drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            drive.update();


            if (isCloseToTarget(robotPose, targetPose)) {
                if (!hasReachedProximity) {
                    hasReachedProximity = true;
                    timer.reset();
                }
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }


    public boolean isFinished(Pose targetPose) {
        Pose robotPose = drive.returnPose();
        Pose delta = targetPose.subtract(robotPose);


        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR ||
                Math.abs(delta.getY()) > ALLOWED_TRANSLATIONAL_ERROR ||
                Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        } else if (!hasReachedProximity) {

            hasReachedProximity = true;
            timer.reset();
        }

        return hasReachedProximity && (timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS);
    }


    private boolean isCloseToTarget(Pose robotPose, Pose targetPose) {
        Pose delta = targetPose.subtract(robotPose);
        return delta.toVec2D().magnitude() <= (ALLOWED_TRANSLATIONAL_ERROR + 2) &&
                Math.abs(delta.getY()) <= (ALLOWED_TRANSLATIONAL_ERROR + 2) &&
                Math.abs(delta.heading) <= (ALLOWED_HEADING_ERROR + Math.toRadians(5));
    }


    public double[] getMotorPowers(Pose robotPose, Pose targetPose) {

        double headingDifference = targetPose.heading - robotPose.heading;
        if (headingDifference > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (headingDifference < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = pidX.calculate(robotPose.getX(), targetPose.getX());
        double yPower = -pidY.calculate(robotPose.getY(), targetPose.getY());
        double hPower = -pidHeading.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
        double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        double frontLeftPower = x_rotated + y_rotated + hPower;
        double backLeftPower = x_rotated - y_rotated + hPower;
        double backRightPower = x_rotated + y_rotated - hPower;
        double frontRightPower = x_rotated - y_rotated - hPower;

        double maxPower = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        frontLeftPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;
        frontRightPower /= maxPower;

        return new double[] {frontLeftPower, backLeftPower, backRightPower, frontRightPower};
    }

    public void stopMotors() {
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
