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
    private final double K_STATIC = 1.0;
    private static final double ALLOWED_TRANSLATIONAL_ERROR = 0.75;
    private static final double ALLOWED_HEADING_ERROR = Math.toRadians(5);
    private static final long STABLE_MS = 250;
    private static final long DEAD_MS = 2500;

    public funnypidtopoint(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        pidX = new PIDFController(0.08, 0, 0.012, 0);
        pidY = new PIDFController(0.08, 0, 0.012, 0);
        pidHeading = new PIDFController(0.8, 0, 0.045, 0);
    }

   public void execute(double targetX, double targetY, double targetHeadingRadians) {
        Pose targetPose = new Pose(targetX, targetY, targetHeadingRadians);
        timer.reset();
        stable.reset();

        while (!isFinished(targetPose)) {
            Pose robotPose = drive.returnPose();
            double[] motorPowers = getMotorPowers(robotPose, targetPose);
            drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            drive.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        stopMotors();
    }

    public boolean isFinished(Pose targetPose) {
        Pose robotPose = drive.returnPose();
        Pose delta = targetPose.subtract(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR ||
                Math.abs(delta.getY()) > ALLOWED_TRANSLATIONAL_ERROR ||
                Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
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
