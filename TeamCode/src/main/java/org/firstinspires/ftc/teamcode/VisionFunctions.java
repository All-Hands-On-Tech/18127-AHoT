package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Vision.CombinedVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionFunctions {

    public boolean isTeamPropDetected = false;
    private static final double STRAFE_GAIN = 0.0339;
    private static final double FORWARD_GAIN = 0.0227;
    private static final double ROTATION_GAIN = 0.0448;
    private LinearOpMode linearOpMode;
    private CombinedVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    public final int RED_1_TAG = 4;
    public final int RED_2_TAG = 5;
    public final int RED_3_TAG = 6;

    public final int BLUE_1_TAG = 1;
    public final int BLUE_2_TAG = 2;
    public final int BLUE_3_TAG = 3;

    private final float TIMEOUT = 5;

    public AprilTagDetection detectedTag = null;

    public List<AprilTagDetection> currentDetections = null;
    ElapsedTime time = new ElapsedTime();

    public VisionFunctions(LinearOpMode l){
        linearOpMode = l;
        Initialize();
    }

    private void Initialize()
    {
        // Create the AprilTag processor by using a builder.
        visionProcessor = new CombinedVisionProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        visionProcessor.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCameraResolution(new Size(VisionConstants.RESWIDTH,VisionConstants.RESHEIGHT))
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(visionProcessor)
                .build();

        if(visionPortal != null){
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        linearOpMode.telemetry.addData("Camera State: ", visionPortal.getCameraState());
    }

    public List<AprilTagDetection> getDetections() {
        return visionProcessor.getDetections();
    }
    public int numberOfDetections() {
        List<AprilTagDetection> currentDetections = visionProcessor.getDetections();
        return currentDetections.size();
    }


    private void setManualExposure(int exposureMS, int gain) {
        time.reset();
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            linearOpMode.telemetry.addData("Camera", "Waiting");
            linearOpMode.telemetry.update();
            while (time.seconds() < TIMEOUT && !linearOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                linearOpMode.sleep(20);
            }
            linearOpMode.telemetry.addData("Camera", "Ready");
            linearOpMode.telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!linearOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                linearOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            linearOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            linearOpMode.sleep(20);
        }
    }

    public void startDetectingApriltags(){
        visionProcessor.setProcessorState(false);
    }
    public void startDetectingProp(){
        visionProcessor.setProcessorState(true);
    }

    public boolean DetectAprilTag(int desiredTag) {
        boolean targetFound = false;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = visionProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTag < 0) || (detection.id == desiredTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    linearOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                linearOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        linearOpMode.telemetry.update();
        return targetFound;
    }

    public String DetectTeamProp(){
        if(visionProcessor.isTeamPropDetected()){
            linearOpMode.telemetry.addData("Team Prop Detection Function", visionProcessor.isTeamPropDetected());
            return visionProcessor.getPropDetection();
        }
        return null;
    }

    public void setRobotStartPosition(boolean isRed, boolean isBackstage){
        visionProcessor.setStartPosition(isRed, isBackstage);
    }

    public boolean checkIfPropIsDetected(){
        return visionProcessor.isTeamPropDetected();
    }

    public float[] moveToTag(int tag, float distance){
        if(DetectAprilTag(tag)){
            linearOpMode.telemetry.addData("Found", "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
            linearOpMode.telemetry.addData("Range",  "%5.1f inches", detectedTag.ftcPose.range);
            linearOpMode.telemetry.addData("Bearing","%3.0f degrees", detectedTag.ftcPose.bearing);
            linearOpMode.telemetry.addData("Yaw","%3.0f degrees", detectedTag.ftcPose.yaw);
            linearOpMode.telemetry.addData("X delta","%3.0f inches", detectedTag.ftcPose.x);

                double x = STRAFE_GAIN * detectedTag.ftcPose.yaw;
                double y = -FORWARD_GAIN * (detectedTag.ftcPose.range - distance);
                double bearing = -ROTATION_GAIN * detectedTag.ftcPose.bearing;

                linearOpMode.telemetry.addData("x: ", x);
                linearOpMode.telemetry.addData("y: ", y);
                linearOpMode.telemetry.addData("bearing: ", bearing);

                return new float[] {(float)x, (float)y, (float)bearing};
    }
        return new float[3];
}


}


/*

package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagsFunctions {
    private LinearOpMode lom = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    public static final int TAG_BLUE_LEFT = 1;
    public static final int TAG_BLUE_CENTER = 2;
    public static final int TAG_BLUE_RIGHT = 3;
    public static final int TAG_RED_LEFT = 4;
    public static final int TAG_RED_CENTER = 5;
    public static final int TAG_RED_RIGHT = 6;
    // Used to hold the data for a detected AprilTag
    public AprilTagDetection detectedTag = null;
    public AprilTagsFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    public boolean DetectAprilTag(int desiredTag)
    {
        boolean targetFound = false;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTag < 0) || (detection.id == desiredTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    lom.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                lom.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        lom.telemetry.update();
        return targetFound;
    }
    private void Initialize()
    {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(lom.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */

    /*
private void setManualExposure(int exposureMS, int gain) {
    // Wait for the camera to be open, then use the controls
    if (visionPortal == null) {
        return;
    }
    // Make sure camera is streaming before we try to set the exposure controls
    if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        lom.telemetry.addData("Camera", "Waiting");
        lom.telemetry.update();
        while (!lom.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            lom.sleep(20);
        }
        lom.telemetry.addData("Camera", "Ready");
        lom.telemetry.update();
    }
    // Set camera controls unless we are stopping.
    if (!lom.isStopRequested())
    {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            lom.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        lom.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        lom.sleep(20);
    }
}
}



 */
