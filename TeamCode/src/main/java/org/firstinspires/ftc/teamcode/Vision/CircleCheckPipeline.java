package org.firstinspires.ftc.teamcode.Vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CircleCheckPipeline extends OpenCvPipeline {

    public Rect rect = VisionConstants.rectLowROI;

    Telemetry telemetry;
    Mat HSVImage = new Mat();
    Mat GrayImage = new Mat();
    Mat Blur = new Mat();
    Mat Circles = new Mat();

    Mat MaskedMat = new Mat();

    Mat BinaryMat = new Mat();
    Mat Overlay = new Mat();

    Mat ROI = new Mat();

    public float sigmaX = 1.5f;
    public float sigmaY = 1.5f;

    public double minDist = 150f;
    public double param1 = 130;
    public double param2 = 30;

    public int minRadius = 15;
    public int maxRadius = 1000;

    Scalar lowRed = VisionConstants.lowRedThreshold;
    Scalar highRed = VisionConstants.highRedThreshold;

    Scalar lowBlue = VisionConstants.lowBlueThreshold;
    Scalar highBlue = VisionConstants.highBlueThreshold;

    Size Kernel = new Size(7,7);

    boolean isRed;


    public CircleCheckPipeline(Telemetry telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;
    }

    public CircleCheckPipeline() {
        this.isRed = false;
    }

    @Override
    public void init(Mat firstFrame) {
    }

    @Override
    public Mat processFrame(Mat input) {

        //releases matrices
        ROI.release();
        MaskedMat.release();
        Overlay.release();
        Circles.release();

        //convert to HSV
        Imgproc.cvtColor(input, HSVImage, Imgproc.COLOR_RGB2HSV);

        //make sumat of HSV image
        ROI = HSVImage.submat(rect);

        //Threshold based off of isRed
        if(isRed){
            Core.inRange(ROI, lowRed, highRed, BinaryMat);
        } else{
            Core.inRange(ROI, lowBlue, highBlue, BinaryMat);
        }


        //add matrix values back to MaskedMat with the mask of BinaryMat
        Core.bitwise_and(ROI, ROI, MaskedMat, BinaryMat);

        //Convert to gray
        Imgproc.cvtColor(MaskedMat, GrayImage, Imgproc.COLOR_RGB2GRAY);

        //blur
        Imgproc.GaussianBlur(GrayImage, Blur, Kernel, sigmaX, sigmaY);


        //feature extraction
        Imgproc.HoughCircles(Blur, Circles,Imgproc.CV_HOUGH_GRADIENT,  1, minDist, param1, param2, minRadius, maxRadius);

        //find number of circles
        int numCircles = Circles.cols();

        ROI.copyTo(Overlay);
        Point center;

        //draw every circle's center and outline
        for(int i=0; i < numCircles; i++)
        {
            double[] data = Circles.get(0, i);
            center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(Overlay, center, 1, new Scalar(255, 0, 0), 2, 8, 0 );

            // circle outline
            int radius = (int) Math.round(data[2]);

            //The first circle has the most votes, highlight it
            if(i<1){
                Imgproc.circle(Overlay, center, radius, new Scalar(30,255,255), 8, 8, 0 );
            }else{
                Imgproc.circle(Overlay, center, radius, new Scalar(0,0,255), 2, 8, 0 );
            }

        }


        Overlay.copyTo(ROI);

        //convert back to rgb
        Imgproc.cvtColor(HSVImage, HSVImage, Imgproc.COLOR_HSV2RGB);
        //draw ROI
        Imgproc.rectangle(HSVImage, rect,new Scalar(0,255,0), 5, 8);

        return HSVImage;

    }

}