package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ObeliskProcessor {

    public enum DetectedColorPattern {
        GPP,  // Green, Purple, Purple
        PGP,  // Purple, Green, Purple
        PPG,  // Purple, Purple, Green
        NONE  // A default value if no tag is seen
    }

    private final OpenCvCamera camera;
    private final AprilTagDetectionPipeline pipeline;

    // Lens intrinsics for a specific camera model (Logitech C270, 640x480)
    // You will need to tune these for your specific camera.
    private static final double FX = 822.317;
    private static final double FY = 822.317;
    private static final double CX = 319.495;
    private static final double CY = 242.502;

    // Tag size in meters
    private static final double TAG_SIZE = 0.166; // Default size for FTC tags

    public ObeliskProcessor(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
            }
        });
    }

    public DetectedColorPattern getDetectedPattern() {
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

        if (detections.isEmpty()) {
            return DetectedColorPattern.NONE;
        }

        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 21:
                    return DetectedColorPattern.GPP;
                case 22:
                    return DetectedColorPattern.PGP;
                case 23:
                    return DetectedColorPattern.PPG;
            }
        }

        return DetectedColorPattern.NONE;
    }

    // The new, correct pipeline implementation
    private static class AprilTagDetectionPipeline extends OpenCvPipeline {
        private long nativeAprilTagPtr;
        private final Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private final Object detectionsSync = new Object();

        private final double tagSize;
        private final double fx;
        private final double fy;
        private final double cx;
        private final double cy;

        public AprilTagDetectionPipeline(double tagSize, double fx, double fy, double cx, double cy) {
            this.tagSize = tagSize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        protected void finalize() {
            // Might be null if createApriltagDetector() throws an exception
            if (nativeAprilTagPtr != 0) {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
                nativeAprilTagPtr = 0;
            } else {
                try {
                    super.finalize();
                } catch (Throwable e) {
                    // This is required by the base class, but we don't have a good way to log it here.
                }
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            // Run AprilTag detection
            ArrayList<AprilTagDetection> newDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, grey, tagSize, fx, fy, cx, cy);

            synchronized (detectionsSync) {
                detections = newDetections;
            }

            return input;
        }

        public ArrayList<AprilTagDetection> getLatestDetections() {
            synchronized (detectionsSync) {
                return new ArrayList<>(detections);
            }
        }
    }
}
