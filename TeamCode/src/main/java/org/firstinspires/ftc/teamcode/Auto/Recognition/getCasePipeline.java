package org.firstinspires.ftc.teamcode.Auto.Recognition;

import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.nothing;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Config
public class getCasePipeline implements VisionProcessor {

   private Rect leftRect ;
  private  Rect centerRect;
  private  Rect rightRect;

    public static double lowhueblue = 90, lowsaturationblue = 35, lowvalueblue = 180;
    public static double highhueblue = 130, highsaturationblue = 75, highvalueblue = 255;

    public static double lowhuered = 160, lowsaturationred = 100, lowvaluered = 20;
    public static double highhuered = 180, highsaturationred = 255, highvaluered = 255;

    public static double leftAverage, centerAverage, rightAverage;

    public double[] averageHSV;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        try {

            if(Globals.rec != null)
            {switch (Globals.rec)
            {
                case blue_far:
                {
                    leftRect = new Rect(1, 160, 160, 200);
                    centerRect = new Rect(206, 160, 150, 200);
                    rightRect = new Rect(380, 160, 162, 200);
                    break;
                }

                case blue_near:
                {
                    leftRect = new Rect(150, 160, 154, 200);
                    centerRect = new Rect(315, 160, 160, 200);
                    rightRect = new Rect(531, 160, 109, 200);
                    break;
                }

                case red_far:
                {
                     leftRect = new Rect(150, 160, 154, 200);
                    centerRect = new Rect(315, 160, 160, 200);
                     rightRect = new Rect(531, 160, 109, 200);
                    break;
                }

                case red_near:
                {
                     leftRect = new Rect(1, 160, 160, 200);
                   centerRect = new Rect(206, 160, 150, 200);
                  rightRect = new Rect(380, 160, 162, 200);

                    break;
                }

                default:
                {
                    leftRect = new Rect(1, 160, 160, 200);
                    centerRect = new Rect(206, 160, 150, 200);
                    rightRect = new Rect(380, 160, 162, 200);
                    break;
                }
            }

            Imgproc.rectangle(frame, leftRect, new Scalar(0, 255, 0), 5);
            Imgproc.rectangle(frame, centerRect, new Scalar(0, 255, 0), 5);
            Imgproc.rectangle(frame, rightRect, new Scalar(0, 255, 0), 5);

            leftAverage = meanColor(frame, leftRect, Globals.rec);
            centerAverage = meanColor(frame, centerRect, Globals.rec);
            rightAverage = meanColor(frame, rightRect, Globals.rec);

                averageHSV = calculateAverageHSV(frame, centerRect);

            double maxOneTwo = Math.max(leftAverage, centerAverage);
           double max = Math.max(maxOneTwo, rightAverage);

           if(max == leftAverage)
           {
               Globals.rec_side = Globals.Rec_Side.left;
           } else if(max == centerAverage)
           {
               Globals.rec_side = Globals.Rec_Side.center;
           } else
           {
               Globals.rec_side = Globals.Rec_Side.right;
           }}


        } catch (Exception e) {
            e.printStackTrace();
        }


        return null;
    }

    public int meanColor(Mat frame, Rect inclusionRect, Globals.Recognision rec) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        Scalar lower;
        Scalar upper;

        Mat goodarea = new Mat(frame, inclusionRect);

        Mat hsvImage = new Mat();
        Imgproc.cvtColor(goodarea, hsvImage, Imgproc.COLOR_BGR2HSV);

        if (rec == Globals.Recognision.red_far || rec == Globals.Recognision.red_near)
        { lower = new Scalar(lowhuered, lowsaturationred, lowvaluered);
         upper = new Scalar(highhuered, highsaturationred, highvaluered);}
        else
        {
             lower = new Scalar(lowhueblue, lowsaturationblue, lowvalueblue);
             upper = new Scalar(highhueblue, highsaturationblue, highvalueblue);
        }

        Mat mask = new Mat();
        Core.inRange(hsvImage, lower, upper, mask);

        int Count = Core.countNonZero(mask);



        return Count;

    }

    public static double[] calculateAverageHSV(Mat image, Rect rect) {
        // Crop the image to the specified rectangle
        Mat croppedImage = new Mat(image, rect);

        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(croppedImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Calculate the sum of HSV values
        double sumH = 0, sumS = 0, sumV = 0;
        int pixelCount = 0;

        for (int row = 0; row < hsvImage.rows(); row++) {
            for (int col = 0; col < hsvImage.cols(); col++) {
                double[] hsv = hsvImage.get(row, col);
                sumH += hsv[0];
                sumS += hsv[1];
                sumV += hsv[2];
                pixelCount++;
            }
        }

        // Calculate the average HSV values
        double averageH = sumH / pixelCount;
        double averageS = sumS / pixelCount;
        double averageV = sumV / pixelCount;

        return new double[]{averageH, averageS, averageV};
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

}