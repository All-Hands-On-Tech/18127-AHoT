package org.firstinspires.ftc.teamcode.Trowel.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionLocalization {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private int detectedTagCount = 0;
    private long lastDetectionTime = 0;
    private boolean isInitialized = false;
    private double detectionConfidence = 0.0;

    private static final String DEFAULT_CAMERA_NAME = "Webcam 1";

    public VisionLocalization(HardwareMap hardwareMap) {
        this(hardwareMap, 0, 0, 0, 0, -90, 0);
    }

    public VisionLocalization(HardwareMap hardwareMap,
                              double camX, double camY, double camZ,
                              double camYaw, double camPitch, double camRoll) {
        cameraPosition = new Position(DistanceUnit.INCH, camX, camY, camZ, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, camYaw, camPitch, camRoll, 0);
        initializeVision(hardwareMap, DEFAULT_CAMERA_NAME);
    }

    public VisionLocalization(HardwareMap hardwareMap, String cameraName,
                              double camX, double camY, double camZ,
                              double camYaw, double camPitch, double camRoll) {
        cameraPosition = new Position(DistanceUnit.INCH, camX, camY, camZ, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, camYaw, camPitch, camRoll, 0);
        initializeVision(hardwareMap, cameraName);
    }

    private void initializeVision(HardwareMap hardwareMap, String cameraName) {
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            aprilTagProcessor.setDecimation(2);

            VisionPortal.Builder builder = new VisionPortal.Builder();

            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, cameraName));
            } catch (Exception e) {
                builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
            }

            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);

            visionPortal = builder.build();
            isInitialized = true;

        } catch (Exception e) {
            System.err.println("VisionLocalization initialization failed: " + e.getMessage());
            isInitialized = false;
        }
    }

    public void update() {
        if (!isInitialized || aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        detectedTagCount = detections == null ? 0 : detections.size();

        if (detectedTagCount > 0) {
            double sumX = 0, sumY = 0, sumHeading = 0;
            int validCount = 0;

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.robotPose != null) {
                    if (detection.metadata.name != null && detection.metadata.name.contains("Obelisk")) {
                        continue;
                    }

                    Position pos = detection.robotPose.getPosition();
                    YawPitchRollAngles orientation = detection.robotPose.getOrientation();

                    sumX += pos.x;
                    sumY += pos.y;
                    sumHeading += orientation.getYaw(AngleUnit.DEGREES);
                    validCount++;
                }
            }

            if (validCount > 0) {
                robotX = sumX / validCount;
                robotY = sumY / validCount;
                robotHeading = sumHeading / validCount;
                lastDetectionTime = System.currentTimeMillis();
                detectionConfidence = Math.min(1.0, validCount / 2.0);
            }
        } else {
            long timeSinceDetection = System.currentTimeMillis() - lastDetectionTime;
            if (timeSinceDetection > 1000) {
                detectionConfidence = Math.max(0.0, detectionConfidence - 0.1);
            }
        }
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    /**
     * Returns a best-effort range (in inches) to the nearest/averaged detection.
     * If detections provide an explicit range (via ftcPose), use the average of those.
     * Otherwise fall back to computing range from the averaged robot X/Y position.
     */
    public double getRobotRange() {
        if (aprilTagProcessor != null) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (detections != null && !detections.isEmpty()) {
                double sumRange = 0.0;
                int count = 0;
                for (AprilTagDetection d : detections) {
                    try {
                        if (d != null && d.ftcPose != null && !Double.isNaN(d.ftcPose.range)) {
                            sumRange += d.ftcPose.range;
                            count++;
                        }
                    } catch (Exception ignored) {
                    }
                }
                if (count > 0) {
                    return sumRange / count; // average range from detections
                }
            }
        }
        // Fallback: Euclidean distance from origin (0,0) using robotX/Y
        return Math.hypot(robotX, robotY);
    }

    public double getRobotHeading() {
        return robotHeading;
    }

    public int getDetectedTagCount() {
        return detectedTagCount;
    }

    public double getConfidence() {
        return detectionConfidence;
    }

    public boolean hasRecentDetection() {
        return (System.currentTimeMillis() - lastDetectionTime) < 1000;
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return null;
        return aprilTagProcessor.getDetections();
    }

    public String getDetectionInfo(AprilTagDetection detection) {
        if (detection == null) return "No detection";

        StringBuilder info = new StringBuilder();
        info.append(String.format("Tag ID: %d", detection.id));

        if (detection.metadata != null) {
            info.append(String.format(" (%s)", detection.metadata.name));

            if (detection.robotPose != null) {
                Position pos = detection.robotPose.getPosition();
                YawPitchRollAngles orient = detection.robotPose.getOrientation();

                info.append(String.format("\nPos: X=%.1f\" Y=%.1f\" Z=%.1f\"",
                        pos.x, pos.y, pos.z));
                info.append(String.format("\nYaw=%.1f° Pitch=%.1f° Roll=%.1f°",
                         orient.getYaw(AngleUnit.DEGREES),
                         orient.getPitch(AngleUnit.DEGREES),
                         orient.getRoll(AngleUnit.DEGREES)));
            }
        }

        return info.toString();
    }

    public void setEnabled(boolean enabled) {
        if (visionPortal != null && aprilTagProcessor != null) {
            visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
        }
    }

    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    public boolean isReady() {
        return isInitialized && visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public String getCameraState() {
        if (visionPortal == null) return "NOT_INITIALIZED";
        return visionPortal.getCameraState().toString();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void setDecimation(int decimation) {
        if (aprilTagProcessor != null) {
            aprilTagProcessor.setDecimation(Math.max(1, Math.min(3, decimation)));
        }
    }

    public String getTelemetry() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== Vision Localization ===\n");
        sb.append(String.format("Camera: %s\n", getCameraState()));
        sb.append(String.format("Tags Detected: %d\n", detectedTagCount));
        sb.append(String.format("Confidence: %.1f%%\n", detectionConfidence * 100));
        sb.append(String.format("Position: X=%.1f\" Y=%.1f\"\n", robotX, robotY));
        sb.append(String.format("Heading: %.1f°\n", robotHeading));
        sb.append(String.format("Recent Detection: %s\n", hasRecentDetection() ? "Yes" : "No"));

        // Add explicit range telemetry for commonly used tags (blue=20, red=24)
        double rangeBlue = getRangeToTag(20);
        double rangeRed = getRangeToTag(24);
        sb.append(String.format("Range (avg detections): %.1f\"\n", getRobotRange()));
        sb.append(String.format("Range to Blue (ID 20): %s\n", Double.isNaN(rangeBlue) ? "N/A" : String.format("%.1f\"", rangeBlue)));
        sb.append(String.format("Range to Red  (ID 24): %s\n", Double.isNaN(rangeRed) ? "N/A" : String.format("%.1f\"", rangeRed)));

        // Add 3D distance telemetry
        sb.append(String.format("3D Distance (avg detections): %.1f\"\n", get3dDistanceAvg()));
        sb.append(String.format("3D Distance to Blue (ID 20): %s\n", Double.isNaN(rangeBlue) ? "N/A" : String.format("%.1f\"", get3dDistanceToTag(20))));
        sb.append(String.format("3D Distance to Red  (ID 24): %s\n", Double.isNaN(rangeRed) ? "N/A" : String.format("%.1f\"", get3dDistanceToTag(24))));

        return sb.toString();
    }

    public AprilTagDetection findTagById(int tagId) {
        if (aprilTagProcessor == null) return null;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null) return null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    public double getYawToTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);
        if (detection == null || detection.ftcPose == null) {
            return Double.NaN;
        }
        return detection.ftcPose.yaw;
    }

    public double getBearingToTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);
        if (detection == null || detection.ftcPose == null) {
            return Double.NaN;
        }
        return detection.ftcPose.bearing;
    }

    public double getRangeToTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);
        if (detection == null || detection.ftcPose == null) {
            return Double.NaN;
        }
        return detection.ftcPose.range;
    }

    public boolean isTagVisible(int tagId) {
        return findTagById(tagId) != null;
    }

    /**
     * Returns the 3D euclidean distance (inches) from the camera to the given tag, using the
     * detection's robotPose position (x,y,z). Falls back to ftcPose.range if robotPose isn't available.
     */
    public double get3dDistanceToTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);
        if (detection == null) return Double.NaN;

        try {
            if (detection.robotPose != null) {
                Position p = detection.robotPose.getPosition();
                if (p != null) {
                    double x = p.x;
                    double y = p.y;
                    double z = p.z;
                    double dist = Math.sqrt(x * x + y * y + z * z);
                    return dist;
                }
            }

            if (detection.ftcPose != null && !Double.isNaN(detection.ftcPose.range)) {
                // ftcPose.range is a 2D or slant range depending on API; use it as a fallback
                return detection.ftcPose.range;
            }
        } catch (Exception ignored) {
        }

        return Double.NaN;
    }

    /**
     * Average 3D distance across all valid detections (inches). Returns NaN if none available.
     */
    public double get3dDistanceAvg() {
        if (aprilTagProcessor == null) return Double.NaN;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) return Double.NaN;

        double sum = 0.0;
        int count = 0;
        for (AprilTagDetection d : detections) {
            double dist = get3dDistanceForDetection(d);
            if (!Double.isNaN(dist)) {
                sum += dist;
                count++;
            }
        }
        return count > 0 ? (sum / count) : Double.NaN;
    }

    private double get3dDistanceForDetection(AprilTagDetection detection) {
        if (detection == null) return Double.NaN;
        try {
            if (detection.robotPose != null) {
                Position p = detection.robotPose.getPosition();
                if (p != null) return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
            if (detection.ftcPose != null && !Double.isNaN(detection.ftcPose.range)) {
                return detection.ftcPose.range;
            }
        } catch (Exception ignored) {
        }
        return Double.NaN;
    }
}
