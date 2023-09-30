package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DetectionPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public Rect rectLeft = new Rect(20, 600, 600, 450);
    public Rect rectRight = new Rect(1150, 450, 600, 450);
    public Rect rectMid = new Rect(550, 450, 600, 450);

    public Scalar low = new Scalar(0, 100, 50);
     public Scalar high = new Scalar(50, 255, 255);
    Mat HSVimage = new Mat();
    Mat BinaryMatLeft = new Mat();
    Mat BinaryMatRight = new Mat();
    Mat BinaryMatMid = new Mat();
    Mat ROILeft = new Mat();
    Mat ROIRight = new Mat();
    Mat ROIMid = new Mat();

    String spikePosition;

    double pixLeft, pixRight, pixMid;

    public DetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        ROILeft.release();
        ROIRight.release();
        ROIMid.release();

        BinaryMatLeft.release();
        BinaryMatRight.release();
        BinaryMatMid.release();

        Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);

        ROILeft = HSVimage.submat(rectLeft);
        ROIRight = HSVimage.submat(rectRight);
        ROIMid = HSVimage.submat(rectMid);

        Core.inRange(ROILeft, low, high, BinaryMatLeft);
        Core.inRange(ROIRight, low, high, BinaryMatRight);
        Core.inRange(ROIMid, low, high, BinaryMatMid);

        pixLeft = Core.sumElems(BinaryMatLeft).val[0];
        pixRight = Core.sumElems(BinaryMatRight).val[0];
        pixMid = Core.sumElems(BinaryMatMid).val[0];

        spikePosition = "Mid";

        if(pixLeft > pixRight && pixLeft > pixMid){
            spikePosition = "LEFT";
        }

        if(pixRight > pixLeft && pixRight > pixMid){
            spikePosition = "RIGHT";
        }

        if(pixMid > pixLeft && pixMid > pixRight){
            spikePosition = "MID";
        }

        telemetry.addData("Spike position: ", spikePosition);
        telemetry.update();

        return input;

    }

    public String getSpikePosition(){
        return spikePosition;
    }

}